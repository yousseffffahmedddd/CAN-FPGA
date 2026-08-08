// =============================================================================
// reg_bank.v -- MCP2515 register file (Module 5, SOW 8.4.1 - 8.4.4, 8.4.7,
// 8.4.9). Top of the ctrl_int_regs group; instantiates irq_ctrl.v and
// mode_fsm.v.
//
// spi_if.v deliberately decides nothing about the register map -- it presents
// a raw addr/wdata/we/rdata byte bus and passes every address 0x00-0xFF
// through unfiltered. Everything about WHICH registers exist, what they reset
// to, which bits are writable, and what feeds them lives here.
//
// Reference: MCP2515 datasheet DS20001801, Table 11-1 (register map) and
// Registers 10-1 through 10-4.
//
// Two structural notes:
//
//   1. rdata is COMBINATIONAL from addr, with no register stage. This is not a
//      style choice -- spi_if.v's `pending_load` mechanism defers its shift
//      register load by exactly one cycle to let this path settle, and
//      inserting a flop here would shift every read by one byte.
//
//   2. The BIT MODIFY read-modify-write is built on that same combinational
//      rdata (see wr_val below), so it needs no separate read port.
// =============================================================================

module reg_bank (
    input  wire         clk,
    input  wire         rst_n,            // active-low async reset

    // ---- Register bus from spi_if.v ------------------------------------
    input  wire [7:0]   addr,
    input  wire [7:0]   wdata,
    input  wire         we,
    output reg  [7:0]   rdata,            // COMBINATIONAL -- see note 1 above
    input  wire [7:0]   bitmod_mask,
    input  wire         bitmod_we,
    input  wire         reset_pulse,      // SPI RESET instruction (Sec 12.2)
    input  wire [2:0]   rts_pulse,        // SPI RTS instruction  (Sec 12.7)
    input  wire         rxbuf_done,       // SPI READ RX BUFFER done (Sec 12.4)
    input  wire         rxbuf_sel,        // 0 = RXB0, 1 = RXB1
    output wire [7:0]   status_byte,      // SPI READ STATUS  (Fig 12-8)
    output wire [7:0]   rxstatus_byte,    // SPI RX STATUS    (Fig 12-9)

    // ---- Interrupt pin --------------------------------------------------
    output wire         int_n,

    // ---- Protocol engine (Module 1) -------------------------------------
    input  wire         bus_idle,         // no frame in progress
    input  wire [7:0]   tec,              // from error_mgmt.v
    input  wire [7:0]   rec,              // from error_mgmt.v
    input  wire [1:0]   err_state,        // 00 active, 01 passive, 10 bus-off
    input  wire [2:0]   tx_done,          // per TX buffer, one-cycle pulse
    input  wire         msg_err,          // message error, one-cycle pulse
    output wire [2:0]   opmod,            // mode in effect
    output wire         config_mode,
    output wire         normal_mode,

    // ---- Control logic (Module 4) ---------------------------------------
    input  wire [2:0]   txreq_clr,        // control_logic clears TXREQ

    // ---- TX buffers (Module 2), 3 instances -----------------------------
    // Flattened: slice n is [n*W +: W], n = 0,1,2 for TXB0/1/2.
    output wire [32:0]  txb_id,           // 3 x 11
    output wire [191:0] txb_data,         // 3 x 64, D0 in the low byte
    output wire [11:0]  txb_dlc,          // 3 x 4, clamped to 8
    output wire [2:0]   txb_rtr,
    output wire [2:0]   txb_wr,           // pulse: load this message
    output wire [2:0]   txb_txreq,        // TXBnCTRL.TXREQ, level
    input  wire [2:0]   txb_ready,        // tx_buffer.ready

    // ---- RX buffers (Module 2), 2 instances -----------------------------
    input  wire [21:0]  rxb_id,           // 2 x 11
    input  wire [127:0] rxb_data,         // 2 x 64
    input  wire [7:0]   rxb_dlc,          // 2 x 4
    input  wire [1:0]   rxb_rtr,
    input  wire [1:0]   rxb_full,
    input  wire [1:0]   rx_ovr,           // overrun pulse, see EFLG note below
    output wire [1:0]   rxb_cpu_read,

    // ---- Acceptance filter (Module 2) -----------------------------------
    // Port names match accept_filter.v exactly.
    output wire [10:0]  rxm0_mask,
    output wire [10:0]  rxf0_id,
    output wire [10:0]  rxf1_id,
    output wire         rxb0_accept_all,
    output wire         bukt,
    output wire [10:0]  rxm1_mask,
    output wire [10:0]  rxf2_id,
    output wire [10:0]  rxf3_id,
    output wire [10:0]  rxf4_id,
    output wire [10:0]  rxf5_id,
    output wire         rxb1_accept_all,
    input  wire         filhit0,
    input  wire [2:0]   filhit1,
    input  wire         accept_rxb0,
    input  wire         accept_rxb1
);

    // =========================================================================
    // STORAGE
    // =========================================================================
    reg [7:0] rxf_sidh [0:5];      // RXF0..RXF5
    reg [7:0] rxf_sidl [0:5];
    reg [7:0] rxf_eid8 [0:5];      // addressable so auto-increment reads work;
    reg [7:0] rxf_eid0 [0:5];      // never used -- standard IDs only
    reg [7:0] rxm_sidh [0:1];      // RXM0..RXM1
    reg [7:0] rxm_sidl [0:1];
    reg [7:0] rxm_eid8 [0:1];
    reg [7:0] rxm_eid0 [0:1];
    reg [7:0] cnf1, cnf2, cnf3;    // storage only -- bit_clk_gen replaces the DPLL
    reg [7:0] canctrl;
    reg [7:0] caninte;
    reg [1:0] eflg_ovr;            // {RX1OVR, RX0OVR} -- latched, host-clearable
    reg [7:0] txb_ctrl [0:2];
    reg [7:0] txb_sidh [0:2];
    reg [7:0] txb_sidl [0:2];
    reg [7:0] txb_eid8 [0:2];
    reg [7:0] txb_eid0 [0:2];
    reg [7:0] txb_dlcr [0:2];
    reg [7:0] txb_dat  [0:23];     // index = n*8 + k, k = D0..D7
    reg [7:0] rxb_ctrl [0:1];      // only RXM[6:5] and BUKT[2] are stored here
    reg       filhit0_r;           // latched on accept, per datasheet 4.5.3
    reg [2:0] filhit1_r;

    integer k;

    // =========================================================================
    // ADDRESS DECODE -- Table 11-1
    // =========================================================================
    // CANCTRL and CANSTAT are each aliased at all eight `0x_F` / `0x_E`
    // addresses. This resolves the open question spi_if.v leaves in its source
    // ("confirm this matches whichever single alias reg_bank.v actually
    // decodes"): all eight are decoded, exactly as the real device does, so no
    // canonical-alias agreement is needed.
    wire is_canstat = (addr[7] == 1'b0) && (addr[3:0] == 4'hE);
    wire is_canctrl = (addr[7] == 1'b0) && (addr[3:0] == 4'hF);

    // Filters: RXF0-2 at 0x00/04/08, RXF3-5 at 0x10/14/18. addr[3:2]==11
    // excludes 0x0C-0x0F (BFPCTRL/TXRTSCTRL/CANSTAT/CANCTRL) and 0x1C-0x1F
    // (TEC/REC/CANSTAT/CANCTRL) from the filter region.
    wire       filt_region = (addr[7:5] == 3'b000) && (addr[3:2] != 2'b11);
    wire [2:0] filt_idx    = (addr[4] ? 3'd3 : 3'd0) + {1'b0, addr[3:2]};
    wire [1:0] filt_byte   = addr[1:0];

    wire       mask_region = (addr[7:3] == 5'b00100);   // 0x20-0x27
    wire       mask_idx    = addr[2];
    wire [1:0] mask_byte   = addr[1:0];

    wire is_tec     = (addr == 8'h1C);
    wire is_rec     = (addr == 8'h1D);
    wire is_cnf3    = (addr == 8'h28);
    wire is_cnf2    = (addr == 8'h29);
    wire is_cnf1    = (addr == 8'h2A);
    wire is_caninte = (addr == 8'h2B);
    wire is_canintf = (addr == 8'h2C);
    wire is_eflg    = (addr == 8'h2D);

    // TXB0/1/2 at 0x30/0x40/0x50, 14 bytes each (CTRL, SIDH, SIDL, EID8,
    // EID0, DLC, D0..D7).
    wire       txb_region = (addr[7] == 1'b0) && (addr[6:4] >= 3'd3) &&
                            (addr[6:4] <= 3'd5) && (addr[3:0] <= 4'hD);
    wire [2:0] txb_idx3   = addr[6:4] - 3'd3;
    wire [1:0] txb_idx    = txb_idx3[1:0];
    wire [3:0] txb_off    = addr[3:0];

    // RXB0/1 at 0x60/0x70, same 14-byte layout.
    wire       rxb_region = (addr[7] == 1'b0) && (addr[6:5] == 2'b11) &&
                            (addr[3:0] <= 4'hD);
    wire       rxb_idx    = addr[4];
    wire [3:0] rxb_off    = addr[3:0];

    // Data-byte selects. addr[3:0] runs 0x6..0xD for D0..D7, so subtract 6.
    wire [3:0] dat_sel    = addr[3:0] - 4'd6;
    wire       is_txb_dat = txb_region && (txb_off >= 4'd6);
    wire       is_rxb_dat = rxb_region && (rxb_off >= 4'd6);

    // =========================================================================
    // RX BUFFER VIEW -- the received message lives in rx_buffer.v; this module
    // presents it through the register map rather than duplicating it.
    // =========================================================================
    wire [10:0] rxb0_id_w = rxb_id[10:0];
    wire [10:0] rxb1_id_w = rxb_id[21:11];
    wire [10:0] rxb_id_sel  = rxb_idx ? rxb1_id_w : rxb0_id_w;
    wire [3:0]  rxb_dlc_sel = rxb_idx ? rxb_dlc[7:4] : rxb_dlc[3:0];
    wire        rxb_rtr_sel = rxb_rtr[rxb_idx];

    // Indexed part-select base: rxb_idx*64 + dat_sel*8, built by concatenation
    // so no multiplier is inferred.
    wire [6:0]  rxb_dat_base = {rxb_idx, dat_sel[2:0], 3'b000};
    wire [7:0]  rxb_dat_byte = rxb_data[rxb_dat_base +: 8];

    // RXBnSIDL layout (Register 4-4): [7:5] SID[2:0], [4] SRR, [3] IDE,
    // [2] unimplemented, [1:0] EID[17:16]. Standard frames only, so IDE and
    // EID are 0 and the remote-frame indication rides on SRR.
    wire [7:0] rxb_sidh_v = rxb_id_sel[10:3];
    wire [7:0] rxb_sidl_v = {rxb_id_sel[2:0], rxb_rtr_sel, 1'b0, 1'b0, 2'b00};
    wire [7:0] rxb_dlcr_v = {1'b0, 1'b0, 2'b00, rxb_dlc_sel};

    // RXB0CTRL (Register 4-1): [6:5] RXM, [3] RXRTR ro, [2] BUKT, [1] BUKT1
    // read-only copy of BUKT, [0] FILHIT0.
    wire [7:0] rxb0_ctrl_v = {1'b0, rxb_ctrl[0][6:5], 1'b0, rxb_rtr[0],
                              rxb_ctrl[0][2], rxb_ctrl[0][2], filhit0_r};
    // RXB1CTRL (Register 4-2): [6:5] RXM, [3] RXRTR ro, [2:0] FILHIT.
    wire [7:0] rxb1_ctrl_v = {1'b0, rxb_ctrl[1][6:5], 1'b0, rxb_rtr[1], filhit1_r};

    // =========================================================================
    // EFLG (Register 6-3)
    // =========================================================================
    // Bits 5..0 TRACK the counters rather than latching -- they clear on their
    // own when the condition lifts, so they are not host-writable. Only the two
    // overrun bits latch an event, and only those two are host-clearable.
    //
    // rx_ovr is an input rather than something derived locally: accept_filter.v
    // gates accept_rxb0 with !rxb0_full, so the "matched but the buffer was
    // already full and rollover was off" case produces no observable signal at
    // this module's boundary. It has to be reported by the receive path.
    wire eflg_ewarn = (tec >= 8'd96)  || (rec >= 8'd96);
    wire eflg_rxwar = (rec >= 8'd96);
    wire eflg_txwar = (tec >= 8'd96);
    wire eflg_rxep  = (rec >= 8'd128);
    wire eflg_txep  = (tec >= 8'd128);
    wire eflg_txbo  = (err_state == 2'b10);

    wire [7:0] eflg = {eflg_ovr[1], eflg_ovr[0], eflg_txbo, eflg_txep,
                       eflg_rxep,   eflg_txwar,  eflg_rxwar, eflg_ewarn};

    // ERRIF fires on any 0 -> 1 edge in EFLG (SOW 8.4.5.31).
    reg [7:0] eflg_d;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) eflg_d <= 8'h00;
        else        eflg_d <= eflg;
    end
    wire set_errif = |(eflg & ~eflg_d);

    // =========================================================================
    // READ MUX -- combinational, unimplemented reads as 0x00 (SOW 8.4.2.10/11)
    // =========================================================================
    wire [7:0] canintf_v;
    wire [2:0] icod_v;

    always @(*) begin
        rdata = 8'h00;
        if      (is_canstat)   rdata = {opmod, 1'b0, icod_v, 1'b0};
        else if (is_canctrl)   rdata = canctrl;
        else if (is_tec)       rdata = tec;
        else if (is_rec)       rdata = rec;
        else if (is_caninte)   rdata = caninte;
        else if (is_canintf)   rdata = canintf_v;
        else if (is_eflg)      rdata = eflg;
        else if (is_cnf1)      rdata = cnf1;
        else if (is_cnf2)      rdata = cnf2;
        else if (is_cnf3)      rdata = cnf3;
        else if (filt_region) begin
            case (filt_byte)
                2'd0: rdata = rxf_sidh[filt_idx];
                2'd1: rdata = rxf_sidl[filt_idx];
                2'd2: rdata = rxf_eid8[filt_idx];
                2'd3: rdata = rxf_eid0[filt_idx];
            endcase
        end
        else if (mask_region) begin
            case (mask_byte)
                2'd0: rdata = rxm_sidh[mask_idx];
                2'd1: rdata = rxm_sidl[mask_idx];
                2'd2: rdata = rxm_eid8[mask_idx];
                2'd3: rdata = rxm_eid0[mask_idx];
            endcase
        end
        else if (txb_region) begin
            if (is_txb_dat) rdata = txb_dat[{txb_idx, dat_sel[2:0]}];
            else case (txb_off)
                4'd0: rdata = txb_ctrl[txb_idx];
                4'd1: rdata = txb_sidh[txb_idx];
                4'd2: rdata = txb_sidl[txb_idx];
                4'd3: rdata = txb_eid8[txb_idx];
                4'd4: rdata = txb_eid0[txb_idx];
                4'd5: rdata = txb_dlcr[txb_idx];
                default: rdata = 8'h00;
            endcase
        end
        else if (rxb_region) begin
            if (is_rxb_dat) rdata = rxb_dat_byte;
            else case (rxb_off)
                4'd0: rdata = rxb_idx ? rxb1_ctrl_v : rxb0_ctrl_v;
                4'd1: rdata = rxb_sidh_v;
                4'd2: rdata = rxb_sidl_v;
                4'd3: rdata = 8'h00;          // EID8 -- standard IDs only
                4'd4: rdata = 8'h00;          // EID0 -- standard IDs only
                4'd5: rdata = rxb_dlcr_v;
                default: rdata = 8'h00;
            endcase
        end
        // Everything else -- including BFPCTRL (0x0C) and TXRTSCTRL (0x0D),
        // both out of scope, and the whole 0x80-0xFF half -- reads 0x00.
    end

    // =========================================================================
    // WRITE PATH
    // =========================================================================
    // BIT MODIFY reduces to an ordinary write of a merged value. spi_if.v has
    // already forced bitmod_mask to 0xFF for addresses outside its
    // bit-modifiable list, so that case degrades to a plain overwrite here with
    // no special handling (datasheet Sec 12.10).
    wire [7:0] wr_val = bitmod_we ? ((rdata & ~bitmod_mask) | (wdata & bitmod_mask))
                                  : wdata;
    wire       wr_en  = we | bitmod_we;

    // Configuration registers are writable only in Configuration mode
    // (SOW 8.4.8.56). Every other register is writable in both modes.
    wire cfg_wr = wr_en && config_mode;

    // CANINTF is owned by irq_ctrl.v; the write is forwarded, not applied here.
    wire canintf_we_i = wr_en && is_canintf;

    // TXREQ: set by an SPI RTS instruction or a host write, cleared by
    // control_logic or by the protocol engine reporting the frame went out.
    wire [2:0] txreq_set_rts = rts_pulse;
    reg  [2:0] txreq_q;
    wire [2:0] txreq_now = {txb_ctrl[2][3], txb_ctrl[1][3], txb_ctrl[0][3]};

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) txreq_q <= 3'b000;
        else        txreq_q <= txreq_now;
    end
    // One-cycle pulse on TXREQ 0 -> 1: this is when tx_buffer.v latches the
    // message that has been sitting in the TXBn registers.
    assign txb_wr = txreq_now & ~txreq_q;

    always @(posedge clk or negedge rst_n) begin
        // Async power-on reset and the synchronous SPI RESET land on exactly
        // the same defaults, so they share one branch (SOW 8.4.1).
        if (!rst_n || reset_pulse) begin
            for (k = 0; k < 6; k = k + 1) begin
                rxf_sidh[k] <= 8'h00; rxf_sidl[k] <= 8'h00;
                rxf_eid8[k] <= 8'h00; rxf_eid0[k] <= 8'h00;
            end
            for (k = 0; k < 2; k = k + 1) begin
                rxm_sidh[k] <= 8'h00; rxm_sidl[k] <= 8'h00;
                rxm_eid8[k] <= 8'h00; rxm_eid0[k] <= 8'h00;
                rxb_ctrl[k] <= 8'h00;
            end
            for (k = 0; k < 3; k = k + 1) begin
                txb_ctrl[k] <= 8'h00;      // TXREQ clear
                txb_sidh[k] <= 8'h00; txb_sidl[k] <= 8'h00;
                txb_eid8[k] <= 8'h00; txb_eid0[k] <= 8'h00;
                txb_dlcr[k] <= 8'h00;
            end
            for (k = 0; k < 24; k = k + 1) txb_dat[k] <= 8'h00;
            cnf1      <= 8'h00;
            cnf2      <= 8'h00;
            cnf3      <= 8'h00;
            canctrl   <= 8'h87;            // REQOP=100 Config, CLKEN=1, CLKPRE=11
            caninte   <= 8'h00;
            eflg_ovr  <= 2'b00;
            filhit0_r <= 1'b0;
            filhit1_r <= 3'b000;
        end else begin
            // ---- Host writes ------------------------------------------
            if (wr_en) begin
                // Configuration-mode-gated registers.
                if (filt_region && config_mode) begin
                    case (filt_byte)
                        2'd0: rxf_sidh[filt_idx] <= wr_val;
                        2'd1: rxf_sidl[filt_idx] <= wr_val;
                        2'd2: rxf_eid8[filt_idx] <= wr_val;
                        2'd3: rxf_eid0[filt_idx] <= wr_val;
                    endcase
                end
                if (mask_region && config_mode) begin
                    case (mask_byte)
                        2'd0: rxm_sidh[mask_idx] <= wr_val;
                        2'd1: rxm_sidl[mask_idx] <= wr_val;
                        2'd2: rxm_eid8[mask_idx] <= wr_val;
                        2'd3: rxm_eid0[mask_idx] <= wr_val;
                    endcase
                end
                if (is_cnf1 && config_mode) cnf1 <= wr_val;
                if (is_cnf2 && config_mode) cnf2 <= wr_val;
                if (is_cnf3 && config_mode) cnf3 <= wr_val;

                // Always-writable registers.
                if (is_canctrl) canctrl <= wr_val;
                if (is_caninte) caninte <= wr_val;

                // EFLG: only the two overrun bits are host-writable, and only
                // to clear. Bits 5..0 track the counters and ignore the write.
                if (is_eflg) eflg_ovr <= eflg_ovr & wr_val[7:6];

                if (txb_region) begin
                    if (is_txb_dat) txb_dat[{txb_idx, dat_sel[2:0]}] <= wr_val;
                    else case (txb_off)
                        // TXBnCTRL: ABTF[6]/MLOA[5]/TXERR[4] are read-only
                        // status, so only TXREQ[3] and TXP[1:0] take the write.
                        4'd0: begin
                                  txb_ctrl[txb_idx][3]   <= wr_val[3];
                                  txb_ctrl[txb_idx][1:0] <= wr_val[1:0];
                              end
                        4'd1: txb_sidh[txb_idx] <= wr_val;
                        4'd2: txb_sidl[txb_idx] <= wr_val;
                        4'd3: txb_eid8[txb_idx] <= wr_val;
                        4'd4: txb_eid0[txb_idx] <= wr_val;
                        4'd5: txb_dlcr[txb_idx] <= wr_val;
                        default: ;
                    endcase
                end

                // RXBnCTRL: RXM[6:5] and, on RXB0 only, BUKT[2]. Everything
                // else in the register is hardware-owned. The message fields
                // (SIDH/SIDL/DLC/D0-D7) live in rx_buffer.v and are read-only
                // from the host's side.
                if (rxb_region && (rxb_off == 4'd0)) begin
                    rxb_ctrl[rxb_idx][6:5] <= wr_val[6:5];
                    if (rxb_idx == 1'b0) rxb_ctrl[0][2] <= wr_val[2];
                end
            end

            // ---- Hardware updates -------------------------------------
            // Applied AFTER the host-write block so that a same-cycle
            // collision resolves in hardware's favour (SOW 8.4.3.19): a status
            // event must never be lost just because an SPI transaction
            // happened to touch the same register on that edge.
            if (|txreq_set_rts) begin
                for (k = 0; k < 3; k = k + 1)
                    if (txreq_set_rts[k]) txb_ctrl[k][3] <= 1'b1;
            end
            for (k = 0; k < 3; k = k + 1)
                if (txreq_clr[k] || tx_done[k]) txb_ctrl[k][3] <= 1'b0;

            // FILHIT latches the filter that matched, per datasheet 4.5.3.
            if (accept_rxb0) filhit0_r <= filhit0;
            if (accept_rxb1) filhit1_r <= filhit1;

            // Overrun flags latch and stay until the host clears them.
            if (rx_ovr[0]) eflg_ovr[0] <= 1'b1;
            if (rx_ovr[1]) eflg_ovr[1] <= 1'b1;
        end
    end

    // =========================================================================
    // OUTPUTS TO THE BUFFERS AND THE ACCEPTANCE FILTER
    // =========================================================================
    genvar g;
    generate
        for (g = 0; g < 3; g = g + 1) begin : TX_PACK
            // Standard ID = {SIDH[7:0], SIDL[7:5]}, MSB first (SOW 8.4.4.20).
            assign txb_id[g*11 +: 11] = {txb_sidh[g], txb_sidl[g][7:5]};
            // DLC > 8 is illegal on the wire; clamp rather than let a bad host
            // value produce an over-length frame (SOW 8.4.4.28).
            assign txb_dlc[g*4 +: 4]  = (txb_dlcr[g][3:0] > 4'd8) ? 4'd8
                                                                  : txb_dlcr[g][3:0];
            assign txb_rtr[g]         = txb_dlcr[g][6];
            assign txb_data[g*64 +: 64] = {txb_dat[g*8+7], txb_dat[g*8+6],
                                           txb_dat[g*8+5], txb_dat[g*8+4],
                                           txb_dat[g*8+3], txb_dat[g*8+2],
                                           txb_dat[g*8+1], txb_dat[g*8+0]};
        end
    endgenerate

    // TXREQ is presented as a level, but deliberately masked on the tx_done
    // cycle. tx_buffer.v prioritises `if (txreq) ready <= 1` over
    // `else if (tx_done) ready <= 0`, so holding txreq high through the
    // completion edge would leave its ready flag stuck set forever. Dropping
    // txreq for that one cycle lets the buffer's clear branch run. Fixing it
    // here rather than in tx_buffer.v keeps Module 2's file untouched.
    assign txb_txreq = txreq_now & ~tx_done;

    // rxbuf_done is spi_if.v telling us the host finished a READ RX BUFFER, so
    // the buffer can be released.
    assign rxb_cpu_read[0] = rxbuf_done && !rxbuf_sel;
    assign rxb_cpu_read[1] = rxbuf_done &&  rxbuf_sel;

    assign rxf0_id = {rxf_sidh[0], rxf_sidl[0][7:5]};
    assign rxf1_id = {rxf_sidh[1], rxf_sidl[1][7:5]};
    assign rxf2_id = {rxf_sidh[2], rxf_sidl[2][7:5]};
    assign rxf3_id = {rxf_sidh[3], rxf_sidl[3][7:5]};
    assign rxf4_id = {rxf_sidh[4], rxf_sidl[4][7:5]};
    assign rxf5_id = {rxf_sidh[5], rxf_sidl[5][7:5]};
    assign rxm0_mask = {rxm_sidh[0], rxm_sidl[0][7:5]};
    assign rxm1_mask = {rxm_sidh[1], rxm_sidl[1][7:5]};

    // RXM[1:0] == 11 means "accept everything, bypass mask and filters"
    // (datasheet 4.2.2).
    assign rxb0_accept_all = (rxb_ctrl[0][6:5] == 2'b11);
    assign rxb1_accept_all = (rxb_ctrl[1][6:5] == 2'b11);
    assign bukt            = rxb_ctrl[0][2];

    // =========================================================================
    // STATUS BYTES -- combinational, because spi_if.v re-samples them on every
    // repeated byte of a READ STATUS / RX STATUS transaction and must observe
    // live values rather than a snapshot (SOW 8.4.9.60).
    // =========================================================================
    // READ STATUS, Figure 12-8.
    assign status_byte = {canintf_v[4], txb_ctrl[2][3],    // TX2IF, TXB2 TXREQ
                          canintf_v[3], txb_ctrl[1][3],    // TX1IF, TXB1 TXREQ
                          canintf_v[2], txb_ctrl[0][3],    // TX0IF, TXB0 TXREQ
                          canintf_v[1], canintf_v[0]};     // RX1IF, RX0IF

    // RX STATUS, Figure 12-9: [7:6] which buffers hold a message,
    // [4:3] message type {IDE, SRR}, [2:0] which filter matched.
    wire       rxs_use1 = canintf_v[1] && !canintf_v[0];   // RXB0 reported first
    wire       rxs_rtr  = rxs_use1 ? rxb_rtr[1] : rxb_rtr[0];
    // Filter codes: RXF0/RXF1 -> 0/1 for RXB0. For RXB1, filhit1 0..3 means
    // RXF2..RXF5 (codes 2..5) and 4..5 means an RXF0/RXF1 rollover (codes
    // 6..7) -- both are just filhit1 + 2.
    wire [2:0] rxs_fil  = rxs_use1 ? (filhit1_r + 3'd2) : {2'b00, filhit0_r};
    assign rxstatus_byte = {canintf_v[1], canintf_v[0],    // [7:6] received message
                            1'b0,                          // [5]   unimplemented
                            1'b0,                          // [4]   IDE - standard only
                            rxs_rtr,                       // [3]   SRR
                            rxs_fil};                      // [2:0] filter match

    // =========================================================================
    // SUB-BLOCKS
    // =========================================================================
    irq_ctrl u_irq_ctrl (
        .clk           (clk),
        .rst_n         (rst_n),
        .sync_reset    (reset_pulse),
        .caninte       (caninte),
        .canintf_we    (canintf_we_i),
        .canintf_wdata (wr_val),
        .set_rx0if     (accept_rxb0),
        .set_rx1if     (accept_rxb1),
        .set_txif      (tx_done),
        .set_errif     (set_errif),
        .set_merrf     (msg_err),
        .set_wakif     (1'b0),          // wake-up is out of scope
        .rxbuf_done    (rxbuf_done),
        .rxbuf_sel     (rxbuf_sel),
        .canintf       (canintf_v),
        .int_n         (int_n),
        .icod          (icod_v)
    );

    mode_fsm u_mode_fsm (
        .clk         (clk),
        .rst_n       (rst_n),
        .sync_reset  (reset_pulse),
        .reqop       (wr_en && is_canctrl ? wr_val[7:5] : canctrl[7:5]),
        .reqop_we    (wr_en && is_canctrl),
        .bus_idle    (bus_idle),
        .opmod       (opmod),
        .config_mode (config_mode),
        .normal_mode (normal_mode)
    );

    // NOTE: txb_ready is intentionally not consumed here. tx_buffer.v's ready
    // flag belongs to control_logic.v's transmit sequencing (Module 4, req
    // 4.4.10); this port exists so the top level has a single place to route
    // it from, and so the buffer status is available here if the split with
    // Module 4 moves later.

endmodule
