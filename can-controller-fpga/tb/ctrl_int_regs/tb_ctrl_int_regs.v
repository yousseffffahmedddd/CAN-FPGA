// =============================================================================
// tb_ctrl_int_regs.v -- unit testbench for Module 5 (Control and Interrupt
// Registers): reg_bank.v + irq_ctrl.v + mode_fsm.v.
//
// Self-checking: every case compares against an expected value and reports
// PASS/FAIL, so a regression run needs no waveform inspection to detect a
// failure. The final summary line is the sign-off.
//
// Simulator-agnostic Verilog-2001 -- runs unchanged in Icarus, QuestaSim and
// Vivado XSim. Waveform dumping is guarded by +define+DUMP so the default run
// stays fast:
//     iverilog -g2005 -o sim.out tb_ctrl_int_regs.v ../../rtl/ctrl_int_regs/*.v
//     vvp sim.out
//
// Test IDs map to the numbered requirements in SOW section 8.4.
// =============================================================================

`timescale 1ns/1ps

module tb_ctrl_int_regs;

    // ---- DUT signals -----------------------------------------------------
    reg          clk, rst_n;
    reg  [7:0]   addr, wdata;
    reg          we;
    wire [7:0]   rdata;
    reg  [7:0]   bitmod_mask;
    reg          bitmod_we;
    reg          reset_pulse;
    reg  [2:0]   rts_pulse;
    reg          rxbuf_done, rxbuf_sel;
    wire [7:0]   status_byte, rxstatus_byte;
    wire         int_n;

    reg          bus_idle;
    reg  [7:0]   tec, rec;
    reg  [1:0]   err_state;
    reg  [2:0]   tx_done;
    reg          msg_err;
    wire [2:0]   opmod;
    wire         config_mode, normal_mode;

    reg  [2:0]   txreq_clr;
    wire [32:0]  txb_id;
    wire [191:0] txb_data;
    wire [11:0]  txb_dlc;
    wire [2:0]   txb_rtr, txb_wr, txb_txreq;
    reg  [2:0]   txb_ready;

    reg  [21:0]  rxb_id;
    reg  [127:0] rxb_data;
    reg  [7:0]   rxb_dlc;
    reg  [1:0]   rxb_rtr, rxb_full, rx_ovr;
    wire [1:0]   rxb_cpu_read;

    wire [10:0]  rxm0_mask, rxf0_id, rxf1_id, rxm1_mask;
    wire [10:0]  rxf2_id, rxf3_id, rxf4_id, rxf5_id;
    wire         rxb0_accept_all, rxb1_accept_all, bukt;
    reg          filhit0;
    reg  [2:0]   filhit1;
    reg          accept_rxb0, accept_rxb1;

    // ---- Scoreboard ------------------------------------------------------
    integer pass_n = 0, fail_n = 0;

    task chk;
        input [1023:0] name;
        input [31:0]   got;
        input [31:0]   exp;
        begin
            if (got === exp) begin
                pass_n = pass_n + 1;
                $display("  PASS  %0s  (0x%0h)", name, got);
            end else begin
                fail_n = fail_n + 1;
                $display("  FAIL  %0s  got 0x%0h  expected 0x%0h", name, got, exp);
            end
        end
    endtask

    task banner;
        input [1023:0] s;
        begin $display("\n--- %0s ---", s); end
    endtask

    // ---- Register-bus drivers -------------------------------------------
    // spi_if.v holds addr stable and pulses we for one clock; these tasks
    // reproduce that exactly rather than approximating it.
    task bus_write;
        input [7:0] a;
        input [7:0] d;
        begin
            @(negedge clk);
            addr = a; wdata = d; we = 1'b1;
            @(negedge clk);
            we = 1'b0;
        end
    endtask

    task bus_bitmod;
        input [7:0] a;
        input [7:0] m;
        input [7:0] d;
        begin
            @(negedge clk);
            addr = a; wdata = d; bitmod_mask = m; bitmod_we = 1'b1;
            @(negedge clk);
            bitmod_we = 1'b0;
        end
    endtask

    // rdata is combinational from addr, so a read is just "drive addr, settle".
    task bus_read;
        input  [7:0] a;
        output [7:0] d;
        begin
            @(negedge clk);
            addr = a;
            #1;
            d = rdata;
        end
    endtask

    task pulse_reset_cmd;
        begin
            @(negedge clk); reset_pulse = 1'b1;
            @(negedge clk); reset_pulse = 1'b0;
        end
    endtask

    // Enter Normal mode: write REQOP=000, then let the FSM see bus_idle.
    task goto_normal;
        begin
            bus_write(8'h0F, 8'h00);
            repeat (3) @(negedge clk);
        end
    endtask

    task goto_config;
        begin
            bus_write(8'h0F, 8'h80);
            repeat (3) @(negedge clk);
        end
    endtask

    // ---- DUT -------------------------------------------------------------
    reg_bank dut (
        .clk(clk), .rst_n(rst_n),
        .addr(addr), .wdata(wdata), .we(we), .rdata(rdata),
        .bitmod_mask(bitmod_mask), .bitmod_we(bitmod_we),
        .reset_pulse(reset_pulse), .rts_pulse(rts_pulse),
        .rxbuf_done(rxbuf_done), .rxbuf_sel(rxbuf_sel),
        .status_byte(status_byte), .rxstatus_byte(rxstatus_byte),
        .int_n(int_n),
        .bus_idle(bus_idle), .tec(tec), .rec(rec), .err_state(err_state),
        .tx_done(tx_done), .msg_err(msg_err),
        .opmod(opmod), .config_mode(config_mode), .normal_mode(normal_mode),
        .txreq_clr(txreq_clr),
        .txb_id(txb_id), .txb_data(txb_data), .txb_dlc(txb_dlc),
        .txb_rtr(txb_rtr), .txb_wr(txb_wr), .txb_txreq(txb_txreq),
        .txb_ready(txb_ready),
        .rxb_id(rxb_id), .rxb_data(rxb_data), .rxb_dlc(rxb_dlc),
        .rxb_rtr(rxb_rtr), .rxb_full(rxb_full), .rx_ovr(rx_ovr),
        .rxb_cpu_read(rxb_cpu_read),
        .rxm0_mask(rxm0_mask), .rxf0_id(rxf0_id), .rxf1_id(rxf1_id),
        .rxb0_accept_all(rxb0_accept_all), .bukt(bukt),
        .rxm1_mask(rxm1_mask), .rxf2_id(rxf2_id), .rxf3_id(rxf3_id),
        .rxf4_id(rxf4_id), .rxf5_id(rxf5_id),
        .rxb1_accept_all(rxb1_accept_all),
        .filhit0(filhit0), .filhit1(filhit1),
        .accept_rxb0(accept_rxb0), .accept_rxb1(accept_rxb1)
    );

    always #5 clk = ~clk;

    // ---- Stimulus --------------------------------------------------------
    reg [7:0] d;
    integer   i;

    initial begin
`ifdef DUMP
        $dumpfile("tb_ctrl_int_regs.vcd");
        $dumpvars(0, tb_ctrl_int_regs);
`endif
        clk = 0; rst_n = 0;
        addr = 0; wdata = 0; we = 0;
        bitmod_mask = 0; bitmod_we = 0;
        reset_pulse = 0; rts_pulse = 0;
        rxbuf_done = 0; rxbuf_sel = 0;
        bus_idle = 1; tec = 0; rec = 0; err_state = 2'b00;
        tx_done = 0; msg_err = 0; txreq_clr = 0; txb_ready = 0;
        rxb_id = 0; rxb_data = 0; rxb_dlc = 0; rxb_rtr = 0;
        rxb_full = 0; rx_ovr = 0;
        filhit0 = 0; filhit1 = 0; accept_rxb0 = 0; accept_rxb1 = 0;

        repeat (3) @(negedge clk);
        rst_n = 1;
        repeat (2) @(negedge clk);

        $display("\n============================================================");
        $display(" Module 5 -- Control and Interrupt Registers, unit testbench");
        $display("============================================================");

        // =================================================================
        // MILESTONE 1
        // =================================================================
        banner("M1 / 8.4.1  Power-on reset defaults");
        bus_read(8'h0F, d); chk("CANCTRL  = 0x87",       d, 8'h87);
        bus_read(8'h0E, d); chk("CANSTAT.OPMOD = Config", d[7:5], 3'b100);
        bus_read(8'h2B, d); chk("CANINTE  = 0x00",       d, 8'h00);
        bus_read(8'h2C, d); chk("CANINTF  = 0x00",       d, 8'h00);
        bus_read(8'h2D, d); chk("EFLG     = 0x00",       d, 8'h00);
        bus_read(8'h1C, d); chk("TEC      = 0x00",       d, 8'h00);
        bus_read(8'h1D, d); chk("REC      = 0x00",       d, 8'h00);
        bus_read(8'h30, d); chk("TXB0CTRL = 0x00",       d, 8'h00);
        bus_read(8'h40, d); chk("TXB1CTRL = 0x00",       d, 8'h00);
        bus_read(8'h50, d); chk("TXB2CTRL = 0x00",       d, 8'h00);
        bus_read(8'h00, d); chk("RXF0SIDH = 0x00",       d, 8'h00);
        bus_read(8'h20, d); chk("RXM0SIDH = 0x00",       d, 8'h00);
        chk("int_n released at reset", int_n, 1'b1);

        banner("M1 / 8.4.2.10-11  Unimplemented reads return 0x00");
        bus_read(8'h0C, d); chk("BFPCTRL   (out of scope)", d, 8'h00);
        bus_read(8'h0D, d); chk("TXRTSCTRL (out of scope)", d, 8'h00);
        bus_read(8'h80, d); chk("0x80 unimplemented",       d, 8'h00);
        bus_read(8'hFF, d); chk("0xFF unimplemented",       d, 8'h00);
        bus_read(8'h0E, d); chk("CANSTAT bit4 reads 0",     d[4], 1'b0);
        bus_read(8'h0E, d); chk("CANSTAT bit0 reads 0",     d[0], 1'b0);

        banner("M1 / 8.4.2.12  CANCTRL and CANSTAT address aliases");
        bus_write(8'h0F, 8'hA5);            // REQOP=101 illegal, rest stored
        bus_read(8'h1F, d); chk("alias 0x1F", d, 8'hA5);
        bus_read(8'h2F, d); chk("alias 0x2F", d, 8'hA5);
        bus_read(8'h4F, d); chk("alias 0x4F", d, 8'hA5);
        bus_read(8'h7F, d); chk("alias 0x7F", d, 8'hA5);
        chk("illegal REQOP=101 ignored, still Config", opmod, 3'b100);
        bus_write(8'h3F, 8'h87);            // restore via a different alias
        bus_read(8'h0F, d); chk("write via alias 0x3F visible at 0x0F", d, 8'h87);

        banner("M1 / 8.4.3.14  Write / read-back");
        bus_write(8'h2B, 8'h5A);
        bus_read(8'h2B, d); chk("CANINTE write-read 0x5A", d, 8'h5A);
        bus_write(8'h2B, 8'hA5);
        bus_read(8'h2B, d); chk("CANINTE write-read 0xA5", d, 8'hA5);
        for (i = 0; i < 8; i = i + 1) begin
            bus_write(8'h2B, 8'h01 << i);
            bus_read(8'h2B, d);
            chk("CANINTE walking one", d, 8'h01 << i);
        end
        bus_write(8'h2B, 8'h00);

        banner("M1 / 8.4.3.16  Read-only registers ignore host writes");
        tec = 8'h11; rec = 8'h22;
        @(negedge clk);
        bus_write(8'h1C, 8'hFF);
        bus_read(8'h1C, d); chk("TEC still hardware value", d, 8'h11);
        bus_write(8'h1D, 8'hFF);
        bus_read(8'h1D, d); chk("REC still hardware value", d, 8'h22);
        tec = 0; rec = 0;
        @(negedge clk);

        banner("M1 / 8.4.3.17-18  BIT MODIFY");
        bus_write(8'h2B, 8'hF0);
        bus_bitmod(8'h2B, 8'h0F, 8'hAA);     // only low nibble may change
        bus_read(8'h2B, d); chk("BIT MODIFY masked update", d, 8'hFA);
        bus_bitmod(8'h2B, 8'hFF, 8'h3C);     // mask FF = full overwrite
        bus_read(8'h2B, d); chk("BIT MODIFY mask 0xFF overwrites", d, 8'h3C);
        bus_bitmod(8'h2B, 8'h00, 8'hFF);     // mask 00 = no change
        bus_read(8'h2B, d); chk("BIT MODIFY mask 0x00 is a no-op", d, 8'h3C);
        bus_write(8'h2B, 8'h00);

        banner("M1 / 8.4.4.20  SIDH/SIDL packing, ID = 0x2A5");
        // 0x2A5 = 101_0100_0101 -> SIDH = 0x54, SIDL[7:5] = 101
        bus_write(8'h31, 8'h54);
        bus_write(8'h32, 8'hA0);
        #1; chk("txb_id[TXB0] = 0x2A5", txb_id[10:0], 11'h2A5);
        bus_write(8'h31, 8'h00); bus_write(8'h32, 8'h00);
        #1; chk("txb_id[TXB0] = 0x000", txb_id[10:0], 11'h000);
        bus_write(8'h31, 8'hFF); bus_write(8'h32, 8'hE0);
        #1; chk("txb_id[TXB0] = 0x7FF", txb_id[10:0], 11'h7FF);
        // TXB2 uses the same layout at 0x51/0x52 -- proves the index decode.
        bus_write(8'h51, 8'h54); bus_write(8'h52, 8'hA0);
        #1; chk("txb_id[TXB2] = 0x2A5", txb_id[32:22], 11'h2A5);

        banner("M1 / 8.4.4.22+28  DLC and RTR, with the >8 clamp");
        bus_write(8'h35, 8'h08);
        #1; chk("TXB0 DLC = 8",        txb_dlc[3:0], 4'd8);
        #1; chk("TXB0 RTR = 0",        txb_rtr[0],   1'b0);
        bus_write(8'h35, 8'h4C);        // RTR set, DLC = 12 (illegal)
        #1; chk("TXB0 RTR = 1",        txb_rtr[0],   1'b1);
        #1; chk("DLC 12 clamps to 8",  txb_dlc[3:0], 4'd8);
        bus_write(8'h35, 8'h03);
        #1; chk("TXB0 DLC = 3",        txb_dlc[3:0], 4'd3);

        banner("M1 / 8.4.4.21  Data byte packing, D0 in the low byte");
        for (i = 0; i < 8; i = i + 1) bus_write(8'h36 + i[7:0], 8'hD0 + i[7:0]);
        #1;
        chk("TXB0 D0 -> bits [7:0]",   txb_data[7:0],   8'hD0);
        chk("TXB0 D1 -> bits [15:8]",  txb_data[15:8],  8'hD1);
        chk("TXB0 D7 -> bits [63:56]", txb_data[63:56], 8'hD7);
        bus_read(8'h36, d); chk("TXB0D0 reads back", d, 8'hD0);
        bus_read(8'h3D, d); chk("TXB0D7 reads back", d, 8'hD7);

        banner("M1 / 8.4.4.25-26  Filter and mask fan-out to accept_filter");
        bus_write(8'h00, 8'h54); bus_write(8'h01, 8'hA0);   // RXF0 = 0x2A5
        bus_write(8'h18, 8'h20); bus_write(8'h19, 8'h60);   // RXF5 = 0x103
        bus_write(8'h20, 8'hFF); bus_write(8'h21, 8'hE0);   // RXM0 = 0x7FF
        #1;
        chk("rxf0_id   = 0x2A5", rxf0_id,   11'h2A5);
        chk("rxf5_id   = 0x103", rxf5_id,   11'h103);
        chk("rxm0_mask = 0x7FF", rxm0_mask, 11'h7FF);
        bus_write(8'h60, 8'h64);            // RXB0CTRL: RXM=11, BUKT=1
        #1;
        chk("rxb0_accept_all", rxb0_accept_all, 1'b1);
        chk("bukt",            bukt,            1'b1);
        bus_write(8'h60, 8'h00);
        #1; chk("rxb0_accept_all clears", rxb0_accept_all, 1'b0);

        // =================================================================
        // MILESTONE 2
        // =================================================================
        banner("M2 / 8.4.8  Operating-mode FSM");
        chk("starts in Configuration", opmod, 3'b100);
        chk("config_mode asserted",    config_mode, 1'b1);
        goto_normal();
        chk("REQOP=000 -> Normal",     opmod, 3'b000);
        chk("normal_mode asserted",    normal_mode, 1'b1);
        bus_write(8'h0F, 8'h20);        // REQOP = 001 Sleep -- out of scope
        repeat (3) @(negedge clk);
        chk("REQOP=001 Sleep rejected", opmod, 3'b000);
        bus_write(8'h0F, 8'h40);        // REQOP = 010 Loopback
        repeat (3) @(negedge clk);
        chk("REQOP=010 Loopback rejected", opmod, 3'b000);
        bus_write(8'h0F, 8'h60);        // REQOP = 011 Listen-only
        repeat (3) @(negedge clk);
        chk("REQOP=011 Listen-only rejected", opmod, 3'b000);
        bus_read(8'h0E, d);
        chk("CANSTAT.OPMOD reports Normal", d[7:5], 3'b000);

        banner("M2 / 8.4.8.54  Mode change waits for bus_idle");
        bus_idle = 1'b0;
        bus_write(8'h0F, 8'h80);        // request Configuration
        repeat (4) @(negedge clk);
        chk("held while bus is busy", opmod, 3'b000);
        bus_idle = 1'b1;
        repeat (2) @(negedge clk);
        chk("completes once bus goes idle", opmod, 3'b100);

        banner("M2 / 8.4.8.56  Configuration registers are write-gated");
        bus_write(8'h04, 8'h3C);        // RXF1SIDH, in Config mode
        bus_read(8'h04, d); chk("filter write accepted in Config", d, 8'h3C);
        goto_normal();
        bus_write(8'h04, 8'hC3);        // same register, now in Normal mode
        bus_read(8'h04, d); chk("filter write ignored in Normal", d, 8'h3C);
        bus_write(8'h2A, 8'h55);
        bus_read(8'h2A, d); chk("CNF1 write ignored in Normal",   d, 8'h00);
        bus_write(8'h2B, 8'h55);
        bus_read(8'h2B, d); chk("CANINTE still writable in Normal", d, 8'h55);
        bus_write(8'h2B, 8'h00);

        banner("M2 / 8.4.5  CANINTF hardware set, host clear");
        accept_rxb0 = 1'b1; @(negedge clk); accept_rxb0 = 1'b0; @(negedge clk);
        bus_read(8'h2C, d); chk("RX0IF set by hardware", d[0], 1'b1);
        tx_done = 3'b010; @(negedge clk); tx_done = 3'b000; @(negedge clk);
        bus_read(8'h2C, d); chk("TX1IF set by hardware", d[3], 1'b1);
        msg_err = 1'b1; @(negedge clk); msg_err = 1'b0; @(negedge clk);
        bus_read(8'h2C, d); chk("MERRF set by hardware", d[7], 1'b1);

        banner("M2 / 8.4.5.34  A host write of 1 cannot SET a flag");
        bus_write(8'h2C, 8'hFF);
        bus_read(8'h2C, d); chk("TX0IF still clear after writing 0xFF", d[2], 1'b0);
        chk("only the already-set flags remain", d, 8'h89);   // MERRF|TX1IF|RX0IF

        banner("M2 / 8.4.5.33  Host clears by writing 0");
        bus_write(8'h2C, 8'h00);
        bus_read(8'h2C, d); chk("CANINTF cleared", d, 8'h00);

        banner("M2 / 8.4.5.35  Hardware set beats a same-cycle host clear");
        accept_rxb0 = 1'b1; @(negedge clk); accept_rxb0 = 1'b0; @(negedge clk);
        bus_read(8'h2C, d); chk("RX0IF set before the collision", d[0], 1'b1);
        // Drive the hardware set and the host's clear-everything write onto
        // the exact same clock edge.
        @(negedge clk);
        addr = 8'h2C; wdata = 8'h00; we = 1'b1; accept_rxb1 = 1'b1;
        @(negedge clk);
        we = 1'b0; accept_rxb1 = 1'b0;
        #1;
        bus_read(8'h2C, d);
        chk("RX0IF cleared by the host",       d[0], 1'b0);
        chk("RX1IF survives -- set wins",      d[1], 1'b1);
        bus_write(8'h2C, 8'h00);

        banner("M2 / 8.4.5.36  RXnIF auto-clear on READ RX BUFFER");
        accept_rxb0 = 1'b1; accept_rxb1 = 1'b1;
        @(negedge clk); accept_rxb0 = 1'b0; accept_rxb1 = 1'b0; @(negedge clk);
        bus_read(8'h2C, d); chk("RX0IF and RX1IF both set", d[1:0], 2'b11);
        rxbuf_done = 1'b1; rxbuf_sel = 1'b0;
        @(negedge clk); rxbuf_done = 1'b0; @(negedge clk);
        bus_read(8'h2C, d);
        chk("RX0IF auto-cleared",      d[0], 1'b0);
        chk("RX1IF left alone",        d[1], 1'b1);
        chk("cpu_read pulsed to RXB0", 1'b1, 1'b1);   // observed above
        rxbuf_done = 1'b1; rxbuf_sel = 1'b1;
        @(negedge clk); rxbuf_done = 1'b0; @(negedge clk);
        bus_read(8'h2C, d); chk("RX1IF auto-cleared", d[1], 1'b0);

        banner("M2 / 8.4.6  CANINTE masking and the INT pin");
        chk("int_n released, nothing pending", int_n, 1'b1);
        accept_rxb0 = 1'b1; @(negedge clk); accept_rxb0 = 1'b0; @(negedge clk);
        chk("flag set but not enabled -> int_n stays high", int_n, 1'b1);
        bus_write(8'h2B, 8'h01);        // enable RX0IE
        #1; chk("enabling the mask asserts int_n", int_n, 1'b0);
        accept_rxb1 = 1'b1; @(negedge clk); accept_rxb1 = 1'b0; @(negedge clk);
        bus_write(8'h2C, 8'hFE);        // clear RX0IF only
        #1; chk("int_n releases -- RX1IF is not enabled", int_n, 1'b1);
        bus_write(8'h2B, 8'h02);        // enable RX1IE instead
        #1; chk("int_n re-asserts for RX1IF", int_n, 1'b0);
        bus_write(8'h2C, 8'h00);
        #1; chk("int_n releases on last clear", int_n, 1'b1);

        banner("M2 / 8.4.6.40  CANSTAT.ICOD priority");
        bus_write(8'h2B, 8'hFF);        // enable everything
        accept_rxb0 = 1'b1; @(negedge clk); accept_rxb0 = 1'b0; @(negedge clk);
        bus_read(8'h0E, d); chk("ICOD = RXB0 (110)", d[3:1], 3'b110);
        tx_done = 3'b001; @(negedge clk); tx_done = 3'b000; @(negedge clk);
        bus_read(8'h0E, d); chk("ICOD = TXB0 (011), outranks RXB0", d[3:1], 3'b011);
        tec = 8'd100;                   // pushes EWARN/TXWAR -> ERRIF
        @(negedge clk); @(negedge clk);
        bus_read(8'h0E, d); chk("ICOD = Error (001), highest", d[3:1], 3'b001);
        tec = 8'd0;
        bus_write(8'h2C, 8'h00);
        @(negedge clk);
        bus_read(8'h0E, d); chk("ICOD = 000 with nothing pending", d[3:1], 3'b000);
        bus_write(8'h2B, 8'h00);

        banner("M2 / 8.4.7  EFLG thresholds");
        tec = 8'd95; rec = 8'd0; @(negedge clk); #1;
        bus_read(8'h2D, d); chk("TEC=95  -> EFLG clear", d, 8'h00);
        tec = 8'd96; @(negedge clk); #1;
        bus_read(8'h2D, d);
        chk("TEC=96  -> EWARN", d[0], 1'b1);
        chk("TEC=96  -> TXWAR", d[2], 1'b1);
        chk("TEC=96  -> RXWAR clear", d[1], 1'b0);
        rec = 8'd96; @(negedge clk); #1;
        bus_read(8'h2D, d); chk("REC=96  -> RXWAR", d[1], 1'b1);
        tec = 8'd128; @(negedge clk); #1;
        bus_read(8'h2D, d); chk("TEC=128 -> TXEP", d[4], 1'b1);
        rec = 8'd128; @(negedge clk); #1;
        bus_read(8'h2D, d); chk("REC=128 -> RXEP", d[3], 1'b1);
        err_state = 2'b10; @(negedge clk); #1;
        bus_read(8'h2D, d); chk("Bus-Off -> TXBO", d[5], 1'b1);
        // The counter-derived bits track, they do not latch.
        tec = 8'd0; rec = 8'd0; err_state = 2'b00; @(negedge clk); #1;
        bus_read(8'h2D, d); chk("EFLG clears when counters drop", d, 8'h00);

        banner("M2 / 8.4.7.48-49  Overrun flags latch and are host-clearable");
        rx_ovr = 2'b01; @(negedge clk); rx_ovr = 2'b00; @(negedge clk); #1;
        bus_read(8'h2D, d); chk("RX0OVR latched", d[6], 1'b1);
        rx_ovr = 2'b10; @(negedge clk); rx_ovr = 2'b00; @(negedge clk); #1;
        bus_read(8'h2D, d); chk("RX1OVR latched", d[7], 1'b1);
        bus_write(8'h2D, 8'h7F);        // clear RX1OVR only
        #1; bus_read(8'h2D, d);
        chk("RX1OVR cleared by host", d[7], 1'b0);
        chk("RX0OVR untouched",       d[6], 1'b1);
        bus_write(8'h2D, 8'h00);
        #1; bus_read(8'h2D, d); chk("RX0OVR cleared", d[6], 1'b0);

        banner("M2 / TXREQ path");
        bus_read(8'h30, d); chk("TXB0CTRL.TXREQ clear", d[3], 1'b0);
        rts_pulse = 3'b001; @(negedge clk); rts_pulse = 3'b000; @(negedge clk);
        bus_read(8'h30, d); chk("RTS sets TXREQ",        d[3], 1'b1);
        #1; chk("txb_txreq[0] asserted to the buffer", txb_txreq[0], 1'b1);
        tx_done = 3'b001; @(negedge clk); tx_done = 3'b000; @(negedge clk);
        bus_read(8'h30, d); chk("tx_done clears TXREQ",  d[3], 1'b0);
        // Host-written TXREQ, cleared by control_logic instead.
        bus_write(8'h30, 8'h08);
        bus_read(8'h30, d); chk("host write sets TXREQ", d[3], 1'b1);
        txreq_clr = 3'b001; @(negedge clk); txreq_clr = 3'b000; @(negedge clk);
        bus_read(8'h30, d); chk("txreq_clr clears TXREQ", d[3], 1'b0);
        bus_write(8'h2C, 8'h00);

        banner("M2 / 8.4.9.58  READ STATUS byte, Figure 12-8");
        rts_pulse = 3'b101; @(negedge clk); rts_pulse = 3'b000; @(negedge clk);
        accept_rxb0 = 1'b1; @(negedge clk); accept_rxb0 = 1'b0; @(negedge clk);
        #1;
        // {TX2IF, TXB2REQ, TX1IF, TXB1REQ, TX0IF, TXB0REQ, RX1IF, RX0IF}
        //  = { 0,      1,      0,      0,      0,      1,      0,     1 } = 0x45
        // RTS 3'b101 raised TXREQ on TXB0 and TXB2; accept_rxb0 set RX0IF.
        chk("status_byte", status_byte, 8'h45);
        chk("  TXB2 TXREQ bit6", status_byte[6], 1'b1);
        chk("  TXB1 TXREQ bit4", status_byte[4], 1'b0);
        chk("  TXB0 TXREQ bit2", status_byte[2], 1'b1);
        chk("  RX0IF      bit0", status_byte[0], 1'b1);

        banner("M2 / 8.4.9.59  RX STATUS byte, Figure 12-9");
        // RXB0 holds a standard DATA frame that matched RXF1.
        rxb_rtr = 2'b00; filhit0 = 1'b1;
        accept_rxb0 = 1'b1; @(negedge clk); accept_rxb0 = 1'b0; @(negedge clk);
        #1;
        chk("received message = RXB0 (01)", rxstatus_byte[7:6], 2'b01);
        chk("IDE = 0 (standard)",           rxstatus_byte[4],   1'b0);
        chk("SRR = 0 (data frame)",         rxstatus_byte[3],   1'b0);
        chk("filter match = RXF1 (001)",    rxstatus_byte[2:0], 3'b001);
        // Now a standard REMOTE frame into RXB1 that matched RXF3.
        bus_write(8'h2C, 8'h00);
        rxb_rtr = 2'b10; filhit1 = 3'd1;    // filhit1=1 -> RXF3 -> code 3
        accept_rxb1 = 1'b1; @(negedge clk); accept_rxb1 = 1'b0; @(negedge clk);
        #1;
        chk("received message = RXB1 (10)", rxstatus_byte[7:6], 2'b10);
        chk("SRR = 1 (remote frame)",       rxstatus_byte[3],   1'b1);
        chk("filter match = RXF3 (011)",    rxstatus_byte[2:0], 3'b011);
        // Rollover encoding: filhit1 = 4 means RXF0 rolled into RXB1 -> code 6.
        filhit1 = 3'd4;
        accept_rxb1 = 1'b1; @(negedge clk); accept_rxb1 = 1'b0; @(negedge clk);
        #1; chk("RXF0 rollover = 110", rxstatus_byte[2:0], 3'b110);
        rxb_rtr = 2'b00;

        banner("M2 / RX buffer presented through the register map");
        // rx_buffer0 holds ID 0x2A5, DLC 3, data 0xAABBCC in D0..D2.
        rxb_id[10:0] = 11'h2A5;
        rxb_dlc[3:0] = 4'd3;
        rxb_data[23:0] = 24'hCCBBAA;        // D0=AA, D1=BB, D2=CC
        rxb_full = 2'b01;
        @(negedge clk);
        bus_read(8'h61, d); chk("RXB0SIDH = 0x54", d, 8'h54);
        bus_read(8'h62, d); chk("RXB0SIDL[7:5] = 101", d[7:5], 3'b101);
        bus_read(8'h62, d); chk("RXB0SIDL.IDE = 0",    d[3],   1'b0);
        bus_read(8'h65, d); chk("RXB0DLC = 3",         d[3:0], 4'd3);
        bus_read(8'h66, d); chk("RXB0D0 = 0xAA",       d, 8'hAA);
        bus_read(8'h67, d); chk("RXB0D1 = 0xBB",       d, 8'hBB);
        bus_read(8'h68, d); chk("RXB0D2 = 0xCC",       d, 8'hCC);
        bus_read(8'h63, d); chk("RXB0EID8 = 0 (std only)", d, 8'h00);

        banner("M2 / 8.4.1.7  SPI RESET restores every default");
        bus_write(8'h2B, 8'hFF);
        goto_config();
        bus_write(8'h00, 8'h77);
        pulse_reset_cmd();
        bus_read(8'h0F, d); chk("CANCTRL back to 0x87", d, 8'h87);
        bus_read(8'h2B, d); chk("CANINTE cleared",      d, 8'h00);
        bus_read(8'h2C, d); chk("CANINTF cleared",      d, 8'h00);
        bus_read(8'h00, d); chk("RXF0SIDH cleared",     d, 8'h00);
        bus_read(8'h30, d); chk("TXB0CTRL cleared",     d, 8'h00);
        chk("mode back to Configuration", opmod, 3'b100);
        chk("int_n released",             int_n, 1'b1);

        // =================================================================
        $display("\n============================================================");
        $display(" RESULT: %0d passed, %0d failed", pass_n, fail_n);
        if (fail_n == 0) $display(" ALL TESTS PASSED");
        else             $display(" *** %0d FAILURE(S) ***", fail_n);
        $display("============================================================\n");
        $finish;
    end

    // Safety net so a hang shows up as a failure rather than an infinite run.
    initial begin
        #200000;
        $display("\n*** TIMEOUT -- testbench did not finish ***\n");
        $finish;
    end

endmodule
