// This is far from final, I consider this just an initial version for those who want to start write their modules' codes and rely on the SPI

// =============================================================================
// spi_if.v -- SPI slave interface for the CAN controller (Normal mode ).
// Full 3 TX buffers (TXB0/1/2) / 2 RX buffers (RXB0/RXB1), matching the real
// MCP2515's Normal-mode behavior. Configuration mode (CNF1-3, filters,
// masks, mode-switching) is out of scope -- those addresses still pass
// through ordinary WRITE unrestricted; reg_bank.v owns whether
// and how they're stored. Mode 0,0 only.
// Reference: MCP2515 datasheet, Section 12.0 "SPI Interface".
// =============================================================================

module spi_if (
    input  wire        clk,            // system clock
    input  wire        rst_n,          // active-low async reset

    // SPI pins -- asynchronous to clk
    input  wire        sck,
    input  wire        si,
    output wire        so,
    input  wire        cs_n,           // active-low chip select

    // Generic register bus to reg_bank.v. Carries ANY address byte
    // 0x00-0xFF unrestricted for READ/WRITE -- spi_if.v never decides which
    // addresses "exist"; that's reg_bank.v's call (Module 4 SOW 4.1/4.2).
    output reg  [7:0]  addr,
    output reg  [7:0]  wdata,
    input  wire [7:0]  rdata,          // assumed combinational from addr 
    output reg          we,

    // Direct one-shot pulses -- bypass the generic addr/we/wdata path,
    // because the datasheet defines these as dedicated SPI commands.
    output reg          reset_pulse,    // RESET (Sec 12.2)
    output reg  [2:0]   rts_pulse,      // RTS (Sec 12.7): bit0/1/2 = TXB0/1/2's TXREQ request
    output reg          rxbuf_done,     // pulses on CS rise, only after READ RX BUFFER (Sec 12.4) 
    output reg          rxbuf_sel,      // which buffer rxbuf_done refers to: 0=RXB0, 1=RXB1
    output reg  [7:0]   bitmod_mask,
    output reg          bitmod_we,      // reg_bank applies (rdata & ~mask) | (data & mask) at addr

    // Live status bytes, assembled elsewhere (control_logic/irq_ctrl,
    // Module 2/4) and re-sampled by this module on every byte, not just
    // once per transaction -- see S_SEND_STATUS below.
    input  wire [7:0]  status_byte,
    input  wire [7:0]  rxstatus_byte
);

    // =========================================================================
    // OPCODES -- Table 12-1
    // =========================================================================
    localparam [7:0] OP_RESET    = 8'hC0;             // 1100_0000 -- single byte, no data phase
    localparam [7:0] OP_READ     = 8'h03;             // 0000_0011 -- opcode + addr + N data bytes
    localparam [7:0] OP_WRITE    = 8'h02;             // 0000_0010 -- opcode + addr + N data bytes
    localparam [7:0] OP_READSTAT = 8'hA0;             // 1010_0000 -- opcode + repeating status byte
    localparam [7:0] OP_RXSTAT   = 8'hB0;             // 1011_0000 -- opcode + repeating status byte
    localparam [7:0] OP_BITMOD   = 8'h05;             // 0000_0101 -- opcode + addr + mask + data
    localparam [7:0] LOADTX_MASK = 8'hF8, LOADTX_ID = 8'h40;  // 0100_0abc
    localparam [7:0] RXBUF_MASK  = 8'hF9, RXBUF_ID  = 8'h90;  // 1001_0nm0
    localparam [7:0] RTS_MASK    = 8'hF8, RTS_ID    = 8'h80;  // 1000_0nnn

    // ---- Registers this design honors a REAL (host-supplied) mask on for
    //      BIT MODIFY ----
    // Per Section 12.10: "Executing the BIT MODIFY command on registers
    // that are not bit-modifiable will force the mask to FFh. This will
    // allow byte writes to the registers, not BIT MODIFY." So addresses
    // OUTSIDE this list are never rejected -- S_GET_MASK below just forces
    // mask=FFh for them instead, which reduces the update to a plain
    // overwrite (data replaces the whole byte).
    //
    // Includes all 5 buffer CTRL registers (all 3 TX + both RX buffers are
    // implemented), CANINTE/CANINTF/EFLG, and CANCTRL. CANCTRL is included
    // because Module 4's own SOW (4.1) explicitly initializes it as a real
    // register ("Initialize all internal control and status registers
    // (CANCTRL, TXBnCTRL, etc.)") -- it exists in reg_bank.v even though
    // mode-switching (REQOP) is pinned to Normal mode. In practice only
    // ABAT is meaningfully mutable here; the protocol engine should ignore
    // REQOP/OSM/CLKEN writes regardless of what BIT MODIFY lets through.
    // Excluded: CNF1-3/BFPCTRL/TXRTSCTRL -- Configuration mode and the
    // physical RXnBF/TXnRTS pins remain fully out of scope.
    //
    // NOTE on the CANCTRL address: the datasheet aliases CANCTRL at every
    // 0x_F address (0x0F, 0x1F, ... 0x7F -- see Table 11-1, row 1111).
    // 0x0F is used here as the canonical address; confirm this matches
    // whichever single alias reg_bank.v actually decodes.
    localparam [7:0] ADDR_TXB0CTRL = 8'h30;
    localparam [7:0] ADDR_TXB1CTRL = 8'h40;
    localparam [7:0] ADDR_TXB2CTRL = 8'h50;
    localparam [7:0] ADDR_RXB0CTRL = 8'h60;
    localparam [7:0] ADDR_RXB1CTRL = 8'h70;
    localparam [7:0] ADDR_CANINTE  = 8'h2B;
    localparam [7:0] ADDR_CANINTF  = 8'h2C;
    localparam [7:0] ADDR_EFLG     = 8'h2D;
    localparam [7:0] ADDR_CANCTRL  = 8'h0F;   // one of 8 aliases -- confirm against reg_bank.v

    // ---- Quick-address entry points, verified against the datasheet's own
    //      tables ----
    //   LOAD TX BUFFER (Sec 12.6, Fig 12-5), opcode 0100_0abc:
    //     a b c | target      addr        a b c | target      addr
    //     0 0 0 | TXB0 SIDH   0x31        0 1 0 | TXB1 SIDH   0x41
    //     0 0 1 | TXB0 D0     0x36        0 1 1 | TXB1 D0     0x46
    //     1 0 0 | TXB2 SIDH   0x51        1 0 1 | TXB2 D0     0x56
    //     1 1 x | reserved (not in the datasheet's table)
    //   {a,b} (= rx_byte[2:1]) selects the buffer; c (= rx_byte[0]) selects
    //   the entry point.
    //
    //   READ RX BUFFER (Sec 12.4, Fig 12-3), opcode 1001_0nm0:
    //     n m | target       addr
    //     0 0 | RXB0 SIDH    0x61
    //     0 1 | RXB0 D0      0x66
    //     1 0 | RXB1 SIDH    0x71
    //     1 1 | RXB1 D0      0x76
    //   n (= rx_byte[2]) selects the buffer; m (= rx_byte[1]) selects the
    //   entry point. All 4 combinations are defined (no reserved code here).
    localparam [7:0] ADDR_TXB0SIDH = 8'h31, ADDR_TXB0D0 = 8'h36;
    localparam [7:0] ADDR_TXB1SIDH = 8'h41, ADDR_TXB1D0 = 8'h46;
    localparam [7:0] ADDR_TXB2SIDH = 8'h51, ADDR_TXB2D0 = 8'h56;
    localparam [7:0] ADDR_RXB0SIDH = 8'h61, ADDR_RXB0D0 = 8'h66;
    localparam [7:0] ADDR_RXB1SIDH = 8'h71, ADDR_RXB1D0 = 8'h76;

    // =========================================================================
    // STAGE 1 -- Clock-domain crossing.
    // SCK/CS are driven by an external host, asynchronous to clk. Sampling
    // them directly risks metastability. 3-flop synchronize each, derive
    // every edge from the synchronized copy only. Requires clk >> SCK.
    // =========================================================================
    reg [2:0] cs_sync, sck_sync;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cs_sync  <= 3'b111;   // reset state = CS high = deselected
            sck_sync <= 3'b000;
        end else begin
            cs_sync  <= {cs_sync[1:0],  cs_n};
            sck_sync <= {sck_sync[1:0], sck};
        end
    end
    wire cs_n_s      = cs_sync[2];                   // synchronized, de-glitched CS level
    wire sck_rising  = (sck_sync[2:1] == 2'b01);      // one-cycle pulse, synchronized SCK 0->1
    wire sck_falling = (sck_sync[2:1] == 2'b10);      // one-cycle pulse, synchronized SCK 1->0
    wire cs_falling  = (cs_sync[2:1]  == 2'b10);      // one-cycle pulse, synchronized CS 1->0 (select)
    wire cs_rising   = (cs_sync[2:1]  == 2'b01);      // one-cycle pulse, synchronized CS 0->1 (deselect)

    // =========================================================================
    // STAGE 2 -- Byte framing, shared by every opcode/address/data/mask byte.
    // =========================================================================
    reg [2:0] bit_cnt;
    wire      byte_done_raw = (bit_cnt == 3'd7) && sck_rising && !cs_n_s;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            bit_cnt <= 3'd0;
        else if (cs_falling)
            bit_cnt <= 3'd0;                          // every new transaction starts a fresh byte
        else if (sck_rising && !cs_n_s)
            bit_cnt <= byte_done_raw ? 3'd0 : bit_cnt + 3'd1;
    end

    // rx_shreg's 8th bit lands via a nonblocking assign in a SEPARATE
    // module on this same edge -- not externally visible until the
    // following clk cycle. byte_done delays byte_done_raw by exactly one
    // cycle so rx_byte is guaranteed complete before anything reads it.
    // (Verified in simulation: without this delay, decode silently misread
    // the last bit of every byte.)
    reg byte_done;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) byte_done <= 1'b0;
        else        byte_done <= byte_done_raw;
    end

    // =========================================================================
    // STAGE 3 -- RX shift register: captures SI, MSB-first (Mode 0,0: sample
    // on SCK rising). Never loads a parallel value -- just continuously
    // shifts; the FSM only trusts rx_byte the instant byte_done pulses.
    // =========================================================================
    wire [7:0] rx_byte;
    shift_reg #(.WIDTH(8)) rx_shreg (
        .clk(clk), .rst_n(rst_n),
        .load(1'b0), .shift_en(sck_rising && !cs_n_s),
        .serial_in(si), .parallel_in(8'h00),
        .serial_out(), .parallel_out(rx_byte)
    );

    // =========================================================================
    // STAGE 4 -- FSM state declarations (ahead of Stage 5 since tx_active
    // needs `state` and the S_* names).
    // =========================================================================
    localparam S_IDLE        = 4'd0,
               S_OPCODE      = 4'd1,   // "Receive opcode" + "Decode opcode" merged: decode
                                        // is purely combinational once byte_done fires
               S_ADDR        = 4'd2,   // shared address-byte capture: READ/WRITE/BIT MODIFY
               S_READ_DATA   = 4'd3,   // READ_DATA+SHIFT_OUT loop; also serves READ RX BUFFER
               S_WRITE_DATA  = 4'd4,   // GET_WRITE_DATA+WRITE_REGISTER loop; also LOAD TX BUFFER
               S_SEND_STATUS = 4'd5,   // SEND_STATUS/SEND_RX_STATUS repeating loop
               S_GET_MASK    = 4'd6,   // BIT MODIFY: mask byte (Fig 12-7 order: addr,mask,data)
               S_GET_DATA    = 4'd7,   // BIT MODIFY: data byte
               S_WAIT_CS     = 4'd8;   // parked, waiting for CS to rise -- every single-shot op

    localparam OP_T_READ = 2'd0, OP_T_WRITE = 2'd1, OP_T_BITMOD = 2'd2;

    reg [3:0] state;

    // =========================================================================
    // STAGE 5 -- TX shift register: drives SO, MSB-first (Mode 0,0: SO
    // changes on SCK falling, sampled by the host on the following rising).
    // =========================================================================
    reg        tx_load;
    reg [7:0]  tx_load_val;
    wire       tx_serial;

    // Only these two states ever legitimately drive SO with real data.
    wire tx_active = (state == S_READ_DATA) || (state == S_SEND_STATUS);

    // SUBTLE BUG THIS CODE HAD TO SOLVE (found in simulation): the SCK
    // falling edge immediately following a load is the SAME edge that's
    // supposed to present the first output bit -- the load already makes
    // it visible combinationally (serial_out = MSB). If that same edge
    // ALSO shifts, the just-loaded MSB is discarded one bit-time before the
    // host ever samples it. suppress_first_shift blocks exactly one
    // shift_en pulse immediately following every load.
    reg suppress_first_shift;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            suppress_first_shift <= 1'b0;
        end else if (tx_load) begin
            suppress_first_shift <= 1'b1;
        end else if (sck_falling && !cs_n_s && tx_active && suppress_first_shift) begin
            suppress_first_shift <= 1'b0;
        end
    end
    wire tx_shift_en = sck_falling && !cs_n_s && tx_active && !suppress_first_shift;

    shift_reg #(.WIDTH(8)) tx_shreg (
        .clk(clk), .rst_n(rst_n),
        .load(tx_load), .shift_en(tx_shift_en),
        .serial_in(1'b0), .parallel_in(tx_load_val),   // constant 0 in -- only the MSB side matters
        .serial_out(tx_serial), .parallel_out()
    );

    reg so_r;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) so_r <= 1'b0;
        else        so_r <= tx_serial;
    end
    // High-Z whenever we're not in an active data-output state, matching
    // the datasheet's own timing diagrams (e.g. Fig 12-2 READ: "Data Out
    // High-Impedance" spans the instruction+address bytes, only switching
    // to real data once the data-out phase begins) -- not just when
    // deselected. tx_active already captures exactly that condition.
    assign so = (cs_n_s || !tx_active) ? 1'bz : so_r;

    // =========================================================================
    // STAGE 6 -- Main FSM
    // =========================================================================
    function is_bitmod_legal(input [7:0] a);
        is_bitmod_legal = (a == ADDR_TXB0CTRL) || (a == ADDR_TXB1CTRL) || (a == ADDR_TXB2CTRL) ||
                           (a == ADDR_RXB0CTRL) || (a == ADDR_RXB1CTRL) ||
                           (a == ADDR_CANINTE)  || (a == ADDR_CANINTF)  || (a == ADDR_EFLG) ||
                           (a == ADDR_CANCTRL);
    endfunction

    reg [1:0] op_type;        // which instruction is using the shared S_ADDR/S_READ_DATA/S_WRITE_DATA states
    reg       rxbuf_active;   // was the CURRENT transaction a READ RX BUFFER? (for the RXnIF-clear pulse)
    reg       we_d;           // we, delayed one cycle -- see S_WRITE_DATA below
    reg       pending_load;   // addr not settled yet -- see the end of this block
    reg       status_sel;     // 0=status_byte, 1=rxstatus_byte -- which live signal S_SEND_STATUS re-samples

    // we_d exists purely to fix a same-cycle race: see the S_WRITE_DATA
    // comment below.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) we_d <= 1'b0;
        else        we_d <= we;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= S_IDLE;
            addr         <= 8'h00;
            we           <= 1'b0;
            reset_pulse  <= 1'b0;
            rts_pulse    <= 3'b000;
            rxbuf_done   <= 1'b0;
            rxbuf_sel    <= 1'b0;
            rxbuf_active <= 1'b0;
            tx_load      <= 1'b0;
            pending_load <= 1'b0;
            bitmod_we    <= 1'b0;
            status_sel   <= 1'b0;
        end else begin
            // Every one-shot pulse defaults low each cycle; branches below
            // re-assert whichever ones actually fire this cycle.
            we          <= 1'b0;
            reset_pulse <= 1'b0;
            rts_pulse   <= 3'b000;
            rxbuf_done  <= 1'b0;
            tx_load     <= 1'b0;
            bitmod_we   <= 1'b0;

            // ---------------------------------------------------------------
            // GLOBAL ABORT: checked with top priority, EVERY cycle,
            // regardless of state. Makes "CS=1 -> Idle from any state"
            // (Sec 12.1: "CS must be raised and then lowered again to
            // invoke another command"; Sec 12.5: an in-progress WRITE byte
            // is aborted, not left undefined) actually true in hardware.
            // ---------------------------------------------------------------
            if (cs_rising) begin
                state <= S_IDLE;
                // RXnIF only clears here, for READ RX BUFFER specifically
                // (Sec 12.4). rxbuf_sel (latched at decode) says which
                // buffer's flag to clear.
                if (rxbuf_active) rxbuf_done <= 1'b1;
                rxbuf_active <= 1'b0;
            end else begin
                case (state)

                    S_IDLE: begin
                        if (cs_falling) state <= S_OPCODE;
                    end

                    // -------------------------------------------------------
                    // Decode: one branch per Table 12-1 row. Order doesn't
                    // affect correctness (the bit patterns don't overlap),
                    // only readability.
                    // -------------------------------------------------------
                    S_OPCODE: begin
                        if (byte_done) begin

                            // RESET (Sec 12.2): single byte, no address/data
                            // phase. Fires immediately -- nothing further to
                            // wait for once the opcode is known.
                            if (rx_byte == OP_RESET) begin
                                reset_pulse <= 1'b1;
                                state <= S_WAIT_CS;

                            // RTS (Sec 12.7): single byte, no address/data
                            // phase. "Any or all of the last three bits can
                            // be set in a single command" -- each bit
                            // independently requests its own TX buffer, so
                            // the whole 3-bit field passes straight through.
                            // nnn=000 naturally produces no pulses, matching
                            // "if nnn=000, the command will be ignored" with
                            // no extra logic needed.
                            end else if ((rx_byte & RTS_MASK) == RTS_ID) begin
                                rts_pulse <= rx_byte[2:0];
                                state <= S_WAIT_CS;

                            // READ STATUS (Sec 12.8): opcode only. status_sel
                            // remembers WHICH signal to keep re-sampling;
                            // the first sample loads immediately.
                            end else if (rx_byte == OP_READSTAT) begin
                                status_sel  <= 1'b0;
                                tx_load_val <= status_byte;
                                tx_load     <= 1'b1;
                                state       <= S_SEND_STATUS;

                            // RX STATUS (Sec 12.9): same repeating behavior,
                            // different live signal.
                            end else if (rx_byte == OP_RXSTAT) begin
                                status_sel  <= 1'b1;
                                tx_load_val <= rxstatus_byte;
                                tx_load     <= 1'b1;
                                state       <= S_SEND_STATUS;

                            // LOAD TX BUFFER (Sec 12.6, Fig 12-5): NO
                            // separate address byte -- {a,b}=rx_byte[2:1]
                            // selects TXB0/1/2, c=rx_byte[0] selects the
                            // entry point, exactly per the table above the
                            // localparams. ab=11 isn't in that table -- the
                            // real chip leaves it undefined; treated here as
                            // a defined no-op rather than a guess.
                            end else if ((rx_byte & LOADTX_MASK) == LOADTX_ID) begin
                                op_type <= OP_T_WRITE;
                                case (rx_byte[2:1])
                                    2'b00: begin addr <= rx_byte[0] ? ADDR_TXB0D0 : ADDR_TXB0SIDH; state <= S_WRITE_DATA; end
                                    2'b01: begin addr <= rx_byte[0] ? ADDR_TXB1D0 : ADDR_TXB1SIDH; state <= S_WRITE_DATA; end
                                    2'b10: begin addr <= rx_byte[0] ? ADDR_TXB2D0 : ADDR_TXB2SIDH; state <= S_WRITE_DATA; end
                                    default: state <= S_WAIT_CS;   // ab=11: reserved, not in the table
                                endcase

                            // READ RX BUFFER (Sec 12.4, Fig 12-3): NO
                            // separate address byte -- n=rx_byte[2] selects
                            // RXB0/RXB1, m=rx_byte[1] selects the entry
                            // point. All 4 combinations are defined.
                            end else if ((rx_byte & RXBUF_MASK) == RXBUF_ID) begin
                                addr         <= rx_byte[2] ? (rx_byte[1] ? ADDR_RXB1D0 : ADDR_RXB1SIDH)
                                                            : (rx_byte[1] ? ADDR_RXB0D0 : ADDR_RXB0SIDH);
                                op_type      <= OP_T_READ;
                                rxbuf_active <= 1'b1;
                                rxbuf_sel    <= rx_byte[2];   // which buffer's flag to clear later
                                pending_load <= 1'b1;
                                state        <= S_READ_DATA;

                            // READ (Sec 12.3): needs a real address byte next.
                            end else if (rx_byte == OP_READ) begin
                                op_type <= OP_T_READ;
                                state   <= S_ADDR;

                            // WRITE (Sec 12.5): needs a real address byte next.
                            end else if (rx_byte == OP_WRITE) begin
                                op_type <= OP_T_WRITE;
                                state   <= S_ADDR;

                            // BIT MODIFY (Sec 12.10): address, then mask,
                            // then data (Fig 12-7 order). Proceeds
                            // unconditionally -- legality of the address
                            // only decides the mask value later, in
                            // S_GET_MASK; it never causes rejection.
                            end else if (rx_byte == OP_BITMOD) begin
                                op_type <= OP_T_BITMOD;
                                state   <= S_ADDR;

                            // Anything else: not one of the 9 defined
                            // instructions. Park in S_WAIT_CS instead of
                            // leaving the FSM with no defined next state --
                            // garbage input can't hang the interface.
                            end else begin
                                state <= S_WAIT_CS;
                            end
                        end
                    end

                    // -------------------------------------------------------
                    // Shared address-byte capture for READ / WRITE / BIT MODIFY.
                    // -------------------------------------------------------
                    S_ADDR: begin
                        if (byte_done) begin
                            addr <= rx_byte;
                            case (op_type)
                                OP_T_READ: begin
                                    pending_load <= 1'b1;
                                    state <= S_READ_DATA;
                                end
                                OP_T_WRITE: state <= S_WRITE_DATA;
                                OP_T_BITMOD: state <= S_GET_MASK;   // always proceed -- see S_GET_MASK
                                default: state <= S_WAIT_CS;
                            endcase
                        end
                    end

                    // -------------------------------------------------------
                    // BIT MODIFY: mask byte, then data byte (Fig 12-7 order).
                    // Per Sec 12.10: legal target -> honor the host's mask.
                    // Illegal target -> force mask=FFh, which reduces
                    // (old & ~FF) | (data & FF) to exactly `data` -- a full
                    // overwrite, never a rejection.
                    // -------------------------------------------------------
                    S_GET_MASK: begin
                        if (byte_done) begin
                            bitmod_mask <= is_bitmod_legal(addr) ? rx_byte : 8'hFF;
                            state <= S_GET_DATA;
                        end
                    end

                    S_GET_DATA: begin
                        if (byte_done) begin
                            wdata     <= rx_byte;
                            bitmod_we <= 1'b1;   // reg_bank.v applies the masked update
                            state     <= S_WAIT_CS;   // single register only -- no auto-increment loop
                        end
                    end

                    // -------------------------------------------------------
                    // READ / READ RX BUFFER: loops for as many bytes as the
                    // host wants, auto-incrementing addr each time. The host
                    // ends this simply by raising CS whenever satisfied
                    // (Sec 12.3: "The READ operation is terminated by
                    // raising the CS pin") -- no fixed byte count required.
                    // -------------------------------------------------------
                    S_READ_DATA: begin
                        if (byte_done) begin
                            addr         <= addr + 8'd1;
                            pending_load <= 1'b1;   // reload for the NEXT byte, once addr settles
                        end
                    end

                    // -------------------------------------------------------
                    // WRITE / LOAD TX BUFFER: same loop in the write direction.
                    //
                    // SAME-CYCLE RACE THIS CODE HAD TO SOLVE (found in
                    // simulation): reg_bank.v's own always @(posedge clk)
                    // block reads our addr/we/wdata outputs. Incrementing
                    // addr in the SAME cycle we assert we=1 would make
                    // reg_bank see (we=1, addr=the NEXT address) instead of
                    // the address this byte was actually meant for, writing
                    // every byte one register too late. we_d (we delayed
                    // one cycle) gates the increment so reg_bank always
                    // sees a stable (correct addr, we=1) pair first.
                    // -------------------------------------------------------
                    S_WRITE_DATA: begin
                        if (byte_done) begin
                            wdata <= rx_byte;
                            we    <= 1'b1;
                        end else if (we_d) begin
                            addr <= addr + 8'd1;
                        end
                    end

                    // -------------------------------------------------------
                    // READ STATUS / RX STATUS: re-SAMPLES the live
                    // status_byte/rxstatus_byte input on every repeat (not
                    // just once at decode time), and reloads the shreg from
                    // it -- so a status change mid-transaction is reflected
                    // on the next byte, matching a real register-mapped
                    // status rather than a value frozen at the start
                    // (Sec 12.8/12.9: "will continue to output the status
                    // bits as long as CS is held low and clocks provided").
                    // -------------------------------------------------------
                    S_SEND_STATUS: begin
                        if (byte_done) begin
                            tx_load_val <= status_sel ? rxstatus_byte : status_byte;
                            tx_load     <= 1'b1;
                        end
                    end

                    // -------------------------------------------------------
                    // Catch-all parking state for every single-shot
                    // instruction (RESET, RTS, a completed BIT MODIFY,
                    // unrecognized opcodes, reserved LOAD TX BUFFER codes).
                    // Does nothing -- waits for the global cs_rising check
                    // above to return to Idle.
                    // -------------------------------------------------------
                    S_WAIT_CS: begin
                    end

                    default: state <= S_IDLE;
                endcase

                // -------------------------------------------------------
                // SAME-CYCLE RACE, READ DIRECTION (found in simulation):
                // addr changes combinationally this cycle (S_ADDR,
                // S_OPCODE's quick-address branches, or S_READ_DATA's
                // increment), but rdata is EXTERNAL combinational logic off
                // addr and needs one cycle to settle. pending_load defers
                // the tx_shreg load by exactly that one cycle -- set the
                // cycle addr changes, consumed the cycle after -- so
                // tx_load_val always captures the byte at the CORRECT,
                // already-settled address.
                // -------------------------------------------------------
                if (pending_load) begin
                    tx_load_val  <= rdata;
                    tx_load      <= 1'b1;
                    pending_load <= 1'b0;
                end
            end
        end
    end

endmodule
