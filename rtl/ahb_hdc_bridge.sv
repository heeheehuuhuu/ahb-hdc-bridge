// =============================================================================
// ahb_hdc_bridge.sv   (rev 2: START-ignored bug fixed)
//
// 32-bit AHB-Lite slave in front of the existing 1024-bit `hdc_compute`
// accelerator (Yatin/hdc_compute.sv -> popcount1024.sv -> compressor42.sv).
//
// Two 1024-bit vector buffers are stored as 32 x 36-bit words (four 9-bit
// byte slices: {even-parity bit, data byte}) and are parity-checked on every
// read, both for ordinary AHB reads and for the pre-launch validation sweep.
//
// Memory map (all accesses must be aligned 32-bit, HSIZE = 3'b010)
//   0x1000_0000 - 0x1000_007C  query vector words 0..31       (R/W)
//   0x1000_0080 - 0x1000_00FC  class vector words 0..31       (R/W)
//   0x1000_0100                CTRL/STATUS                    (R/W)
//   0x1000_0104                RESULT (Hamming distance)      (RO)
//
// CTRL/STATUS
//   [0] START  write-1 command, reads as 0
//   [1] BUSY   RO, high from START acceptance until completion or abort
//   [2] DONE   RO status, W1C; also cleared when a new START is accepted
//   [3] FAULT  sticky parity fault; W1C (reset also clears it)
//   [31:4]     reserved, read as 0, writes ignored
//
// Response policy: any access that cannot be performed gets the standard
// two-cycle AHB-Lite ERROR response and has NO side effect. This includes:
//   - HSIZE != word, misaligned, unmapped address, write to RESULT
//   - vector read/write while validation/compute owns the memories
//   - read of a never-written vector word
//   - read of a vector word that fails parity (also latches FAULT)
//   - START while busy, while FAULT is latched, or with incomplete vectors
// Software detects a rejected START by the ERROR response.
//
// Known limitations:
//   - HREADYOUT/HRESP depend combinationally on HWDATA[0] for CSR writes
//     (needed to reject START in the first response cycle).
//   - No watchdog on S_WAIT: relies on hdc_compute always producing op_valid.
//   - HRESETn is assumed to be released synchronously by the SoC.
// =============================================================================
module ahb_hdc_bridge (
    input  logic        HCLK,
    input  logic        HRESETn,

    input  logic        HSEL,
    input  logic [31:0] HADDR,
    input  logic [1:0]  HTRANS,
    input  logic        HWRITE,
    input  logic [2:0]  HSIZE,
    input  logic        HREADY,
    input  logic [31:0] HWDATA,

    output logic        HREADYOUT,
    output logic [31:0] HRDATA,
    output logic        HRESP,

    output logic        safety_fault_irq
);

  // ---------------------------------------------------------------------------
  // Address decode constants. Vector regions are decoded on HADDR[31:7] so the
  // word index is simply HADDR[6:2]; it can never leave the range 0..31.
  // ---------------------------------------------------------------------------
  localparam logic [24:0] Q_TAG    = 25'h020_0000;   // 0x1000_0000 >> 7
  localparam logic [24:0] C_TAG    = 25'h020_0001;   // 0x1000_0080 >> 7
  localparam logic [31:0] CSR_ADDR = 32'h1000_0100;
  localparam logic [31:0] RES_ADDR = 32'h1000_0104;
  localparam logic [2:0]  SIZE_W   = 3'b010;

  typedef enum logic [2:0] {
    S_IDLE   = 3'd0,
    S_RD_REQ = 3'd1,   // present read index to both memories
    S_RD_CHK = 3'd2,   // memory data valid: check parity, rebuild vector words
    S_LAUNCH = 3'd3,   // ip_valid high for exactly this cycle
    S_WAIT   = 3'd4    // wait for hdc_compute op_valid
  } state_e;

  state_e       state_q;
  logic [4:0]   idx_q;

  // ---------------------------------------------------------------------------
  // HDC core connection
  // ---------------------------------------------------------------------------
  logic [1023:0] quer_vect;
  logic [1023:0] class_vect;
  logic          ip_valid;
  logic          op_valid;
  logic [10:0]   hamm_dist;

  hdc_compute #(
      .D(1024)
  ) u_hdc_compute (
      .clk       (HCLK),
      .rst       (~HRESETn),     // hdc_compute: active-high async reset
      .quer_vect (quer_vect),
      .class_vect(class_vect),
      .ip_valid  (ip_valid),
      .op_valid  (op_valid),
      .hamm_dist (hamm_dist)
  );

  // Single-cycle pulse: the FSM sits in S_LAUNCH for exactly one clock, and
  // quer_vect/class_vect were fully registered on the previous edge.
  assign ip_valid = (state_q == S_LAUNCH);

  // ---------------------------------------------------------------------------
  // Architectural state
  // ---------------------------------------------------------------------------
  logic [31:0] query_word_valid;
  logic [31:0] class_word_valid;
  logic        fault_q;
  logic        done_q;
  logic [10:0] result_q;

  logic        busy;
  logic        vec_locked;     // vector memories owned by the FSM
  logic        all_valid;

  assign busy       = (state_q != S_IDLE);
  assign vec_locked = busy;
  assign all_valid  = (&query_word_valid) & (&class_word_valid);

  assign safety_fault_irq = fault_q;

  // ---------------------------------------------------------------------------
  // Parity helpers: slice i = {parity[i*9+8], data[i*9 +: 8]}, even parity,
  // i.e. XOR of all nine bits of a slice must be 0.
  // ---------------------------------------------------------------------------
  function automatic logic word_par_ok(input logic [35:0] w);
    logic ok;
    ok = 1'b1;
    for (int i = 0; i < 4; i++) ok = ok & ~(^w[i*9 +: 9]);
    return ok;
  endfunction

  function automatic logic [31:0] word_data(input logic [35:0] w);
    logic [31:0] d;
    for (int i = 0; i < 4; i++) d[i*8 +: 8] = w[i*9 +: 8];
    return d;
  endfunction

  // ---------------------------------------------------------------------------
  // AHB address phase
  // ---------------------------------------------------------------------------
  logic accept;
  assign accept = HSEL & HREADY & HTRANS[1];   // NONSEQ or SEQ; IDLE/BUSY ignored

  logic a_is_q, a_is_c;
  assign a_is_q = (HADDR[31:7] == Q_TAG);
  assign a_is_c = (HADDR[31:7] == C_TAG);

  // Captured address-phase information (valid during the data phase)
  logic        dp_valid;
  logic        dp_write;
  logic [31:0] dp_addr;
  logic [2:0]  dp_size;
  logic        dp_blocked;     // vector access attempted while FSM owned memories
  logic        err_second;     // second cycle of an ERROR response

  always_ff @(posedge HCLK or negedge HRESETn) begin
    if (!HRESETn) begin
      dp_valid   <= 1'b0;
      dp_write   <= 1'b0;
      dp_addr    <= 32'h0;
      dp_size    <= 3'b000;
      dp_blocked <= 1'b0;
    end else if (HREADY) begin
      dp_valid <= accept;
      if (accept) begin
        dp_write   <= HWRITE;
        dp_addr    <= HADDR;
        dp_size    <= HSIZE;
        dp_blocked <= vec_locked;
      end
    end
  end

  // ---------------------------------------------------------------------------
  // AHB data-phase decode (from captured address only)
  // ---------------------------------------------------------------------------
  logic       dp_is_q, dp_is_c, dp_is_csr, dp_is_res, dp_vec;
  logic [4:0] dp_idx;
  logic       dp_dec_err;

  assign dp_is_q   = (dp_addr[31:7] == Q_TAG);
  assign dp_is_c   = (dp_addr[31:7] == C_TAG);
  assign dp_is_csr = (dp_addr == CSR_ADDR);
  assign dp_is_res = (dp_addr == RES_ADDR);
  assign dp_vec    = dp_is_q | dp_is_c;
  assign dp_idx    = dp_addr[6:2];

  // unsupported size, misaligned, unmapped, or write to the read-only RESULT
  assign dp_dec_err = (dp_size != SIZE_W)
                    | (dp_addr[1:0] != 2'b00)
                    | ~(dp_vec | dp_is_csr | dp_is_res)
                    | (dp_write & dp_is_res);

  // ---------------------------------------------------------------------------
  // Protected vector memories (synchronous read, 1-cycle latency, one read port
  // and one write port each, no reset -- a drop-in shape for an SRAM macro).
  // ---------------------------------------------------------------------------
  logic [35:0] query_mem [0:31];
  logic [35:0] class_mem [0:31];
  logic [35:0] q_rdata, c_rdata;

  // write payload with per-byte even parity (spec section 6 layout)
  logic [35:0] secure_wdata;
  always_comb begin
    for (int i = 0; i < 4; i++) begin
      secure_wdata[i*9 +: 8] = HWDATA[i*8 +: 8];
      secure_wdata[i*9 + 8]  = ^HWDATA[i*8 +: 8];
    end
  end

  // FSM read request (both memories read the same index each step)
  logic fsm_rd_en;
  assign fsm_rd_en = (state_q == S_RD_REQ);

  // AHB read request is issued in the address phase so that data returns in the
  // data phase with zero wait states. The FSM has priority on the read port;
  // an AHB vector read that collides with the FSM is rejected in its data
  // phase (dp_blocked / vec_locked), so its data is never used.
  logic ahb_q_rd, ahb_c_rd;
  assign ahb_q_rd = accept & ~HWRITE & a_is_q;
  assign ahb_c_rd = accept & ~HWRITE & a_is_c;

  logic        q_rd_en, c_rd_en;
  logic [4:0]  rd_addr;
  assign q_rd_en = fsm_rd_en | ahb_q_rd;
  assign c_rd_en = fsm_rd_en | ahb_c_rd;
  assign rd_addr = fsm_rd_en ? idx_q : HADDR[6:2];

  // ---------------------------------------------------------------------------
  // Error detection in the data phase
  // ---------------------------------------------------------------------------
  logic [35:0] dp_rword;
  logic        dp_written;
  logic        dp_lock_err;
  logic        dp_unwritten_err;
  logic        dp_par_bad;
  logic        dp_start_rej;
  logic        err_cond;
  logic        err_now;

  assign dp_rword   = dp_is_q ? q_rdata : c_rdata;
  assign dp_written = dp_is_q ? query_word_valid[dp_idx] : class_word_valid[dp_idx];

  // vector memories are locked while validating/computing
  assign dp_lock_err      = dp_vec & (dp_blocked | vec_locked);
  // never hand out the contents of a location nobody wrote
  assign dp_unwritten_err = dp_vec & ~dp_write & ~dp_written;
  // corrupted word on an ordinary read (only meaningful when we really read it)
  assign dp_par_bad       = dp_vec & ~dp_write & dp_written & ~dp_dec_err
                          & ~dp_lock_err & ~word_par_ok(dp_rword);
  // START rejected: already busy, fault latched, or vectors incomplete
  assign dp_start_rej     = dp_is_csr & dp_write & HWDATA[0]
                          & (vec_locked | fault_q | ~all_valid);

  assign err_cond = dp_dec_err | dp_lock_err | dp_unwritten_err | dp_par_bad
                  | dp_start_rej;
  assign err_now  = dp_valid & ~err_second & err_cond;

  // Two-cycle ERROR: cycle 1 HREADYOUT=0/HRESP=1, cycle 2 HREADYOUT=1/HRESP=1.
  assign HREADYOUT = ~err_now;
  assign HRESP     = err_now | err_second;

  always_ff @(posedge HCLK or negedge HRESETn) begin
    if (!HRESETn)       err_second <= 1'b0;
    else if (err_now)   err_second <= 1'b1;
    else if (HREADY)    err_second <= 1'b0;
  end

  // A data phase takes effect exactly once: on the edge where it completes
  // (HREADY high) and it is not an error.
  logic dp_commit, wr_commit;
  assign dp_commit = dp_valid & HREADY & ~err_now & ~err_second;
  assign wr_commit = dp_commit & dp_write;

  logic q_we, c_we;
  assign q_we = wr_commit & dp_is_q;
  assign c_we = wr_commit & dp_is_c;

  // Accepted START: only reachable when state_q == S_IDLE, no fault, vectors
  // complete (anything else was rejected with an ERROR and has no commit).
  logic start_go;
  assign start_go = wr_commit & dp_is_csr & HWDATA[0];

  // ---------------------------------------------------------------------------
  // Memory arrays. Read-during-write to the same word returns the new data.
  // ---------------------------------------------------------------------------
  always_ff @(posedge HCLK) begin
    if (q_we) query_mem[dp_idx] <= secure_wdata;
    if (q_rd_en) begin
      if (q_we && (dp_idx == rd_addr)) q_rdata <= secure_wdata;
      else                             q_rdata <= query_mem[rd_addr];
    end
  end

  always_ff @(posedge HCLK) begin
    if (c_we) class_mem[dp_idx] <= secure_wdata;
    if (c_rd_en) begin
      if (c_we && (dp_idx == rd_addr)) c_rdata <= secure_wdata;
      else                             c_rdata <= class_mem[rd_addr];
    end
  end

  // ---------------------------------------------------------------------------
  // Read data
  // ---------------------------------------------------------------------------
  always_comb begin
    HRDATA = 32'h0;
    if (dp_valid && !dp_write && !err_now && !err_second) begin
      if (dp_is_q)        HRDATA = word_data(q_rdata);
      else if (dp_is_c)   HRDATA = word_data(c_rdata);
      else if (dp_is_csr) HRDATA = {28'h0, fault_q, done_q, busy, 1'b0};
      else if (dp_is_res) HRDATA = {21'h0, result_q};
    end
  end

  // ---------------------------------------------------------------------------
  // Control: bridge state, validity, fault, done, result
  // state_q / idx_q / done_q are each driven from the FSM case below with a
  // single, explicit priority order (later assignments win).
  // ---------------------------------------------------------------------------
  logic q_ok, c_ok;
  assign q_ok = word_par_ok(q_rdata);
  assign c_ok = word_par_ok(c_rdata);

  always_ff @(posedge HCLK or negedge HRESETn) begin
    if (!HRESETn) begin
      state_q          <= S_IDLE;
      idx_q            <= 5'd0;
      query_word_valid <= 32'h0;
      class_word_valid <= 32'h0;
      fault_q          <= 1'b0;
      done_q           <= 1'b0;
      result_q         <= 11'h0;
    end else begin
      // ---- AHB-side effects (writes are blocked while busy, so these never
      //      race the validity bitmaps with the FSM) --------------------------
      if (q_we) query_word_valid[dp_idx] <= 1'b1;
      if (c_we) class_word_valid[dp_idx] <= 1'b1;

      if (wr_commit && dp_is_csr) begin
        if (HWDATA[2]) done_q  <= 1'b0;          // W1C
        if (HWDATA[3]) fault_q <= 1'b0;          // W1C
      end

      // ---- start / validation / launch / completion FSM ---------------------
      case (state_q)
        S_IDLE: begin
          if (start_go) begin
            state_q <= S_RD_REQ;
            idx_q   <= 5'd0;
            done_q  <= 1'b0;                     // new START clears DONE
          end
        end

        S_RD_REQ: state_q <= S_RD_CHK;

        S_RD_CHK: begin
          if (!(q_ok && c_ok)) begin
            fault_q <= 1'b1;                     // abort: never launch a bad word
            state_q <= S_IDLE;
          end else if (idx_q == 5'd31) begin
            state_q <= S_LAUNCH;
          end else begin
            idx_q   <= idx_q + 5'd1;
            state_q <= S_RD_REQ;
          end
        end

        S_LAUNCH: state_q <= S_WAIT;

        S_WAIT: begin
          if (op_valid) begin
            result_q <= hamm_dist;
            done_q   <= 1'b1;                    // wins over a same-cycle W1C
            state_q  <= S_IDLE;
          end
        end

        default: state_q <= S_IDLE;
      endcase

      // ---- sticky fault set (wins over a same-cycle W1C clear) --------------
      if (err_now && dp_par_bad) fault_q <= 1'b1;
    end
  end

  // Vector reconstruction (no reset: validity bitmaps gate any use)
  always_ff @(posedge HCLK) begin
    if (state_q == S_RD_CHK && q_ok && c_ok) begin
      quer_vect [{idx_q, 5'b00000} +: 32] <= word_data(q_rdata);
      class_vect[{idx_q, 5'b00000} +: 32] <= word_data(c_rdata);
    end
  end

endmodule
