`timescale 1ns / 1ps

module tb_control_logic;

    // ============================================================
    // INPUTS TO DUT
    // ============================================================

    reg        clk;
    reg        rst_n;

    // Host command events
    reg        rts_pulse;
    reg        reset_pulse;
    reg        rxbuf_done;

    // Current register values from Register Bank
    reg [7:0]  canctrl;
    reg [7:0]  bfpctrl;
    reg [7:0]  caninte;
    reg [7:0]  canintf;
    reg        txreq;

    // Protocol Engine status
    reg        tx_done;
    reg        tx_abort;
    reg        rx_success;

    reg [7:0]  eng_tec;
    reg [7:0]  eng_rec;

    reg        eng_err_passive;
    reg        eng_bus_off;

    // Receive buffer status
    reg        rx_full;
    reg        rx_rtr;
    reg        rx_ide;
    reg [3:0]  rx_filter_hit;


    // ============================================================
    // OUTPUTS FROM DUT
    // ============================================================

    wire        control_reset;

    wire        txreq_set;
    wire        txreq_clear;

    wire        set_rx0if;
    wire        clear_rx0if;

    wire        set_tx0if;
    wire        clear_tx0if;

    wire        set_merrf;
    wire        clear_merrf;

    wire        set_errif;

    wire        tx_start;

    wire [7:0]  status_byte;
    wire [7:0]  rxstatus_byte;

    wire [7:0]  tec;
    wire [7:0]  rec;
    wire [7:0]  eflg;

    wire        int_pin;


    // ============================================================
    // DUT
    // ============================================================

    control_logic dut (
        .clk(clk),
        .rst_n(rst_n),

        .rts_pulse(rts_pulse),
        .reset_pulse(reset_pulse),
        .rxbuf_done(rxbuf_done),

        .canctrl(canctrl),
        .bfpctrl(bfpctrl),
        .caninte(caninte),
        .canintf(canintf),
        .txreq(txreq),

        .tx_done(tx_done),
        .tx_abort(tx_abort),
        .rx_success(rx_success),

        .eng_tec(eng_tec),
        .eng_rec(eng_rec),

        .eng_err_passive(eng_err_passive),
        .eng_bus_off(eng_bus_off),

        .rx_full(rx_full),
        .rx_rtr(rx_rtr),
        .rx_ide(rx_ide),
        .rx_filter_hit(rx_filter_hit),

        .control_reset(control_reset),

        .txreq_set(txreq_set),
        .txreq_clear(txreq_clear),

        .set_rx0if(set_rx0if),
        .clear_rx0if(clear_rx0if),

        .set_tx0if(set_tx0if),
        .clear_tx0if(clear_tx0if),

        .set_merrf(set_merrf),
        .clear_merrf(clear_merrf),

        .set_errif(set_errif),

        .tx_start(tx_start),

        .status_byte(status_byte),
        .rxstatus_byte(rxstatus_byte),

        .tec(tec),
        .rec(rec),
        .eflg(eflg),

        .int_pin(int_pin)
    );


    // ============================================================
    // CLOCK
    // ============================================================

    always #5 clk = ~clk;


    // ============================================================
    // TEST
    // ============================================================

    initial begin

        // --------------------------------------------------------
        // Initial values
        // --------------------------------------------------------

        clk = 0;
        rst_n = 1;

        rts_pulse = 0;
        reset_pulse = 0;
        rxbuf_done = 0;

        canctrl = 8'h00;
        bfpctrl = 8'h00;
        caninte = 8'h00;
        canintf = 8'h00;
        txreq = 1'b0;

        tx_done = 0;
        tx_abort = 0;
        rx_success = 0;

        eng_tec = 8'h00;
        eng_rec = 8'h00;

        eng_err_passive = 0;
        eng_bus_off = 0;

        rx_full = 0;
        rx_rtr = 0;
        rx_ide = 0;
        rx_filter_hit = 4'h0;

        #10;


        // ========================================================
        // TEST 1: RESET
        // ========================================================

        $display("========================================");
        $display("TEST 1: RESET");
        $display("========================================");

        rst_n = 0;
        #1;

        if (control_reset !== 1'b1)
            $display("FAIL: control_reset should be 1");

        else
            $display("PASS: control_reset asserted");

        rst_n = 1;
        #1;


        // ========================================================
        // TEST 2: RTS
        // ========================================================

        $display("========================================");
        $display("TEST 2: RTS TXB0");
        $display("========================================");

        rts_pulse = 1;
        #1;

        if (txreq_set !== 1'b1)
            $display("FAIL: txreq_set should be 1");

        else
            $display("PASS: txreq_set asserted");

        if (tx_start !== 1'b1)
            $display("FAIL: tx_start should be 1");

        else
            $display("PASS: tx_start asserted");

        rts_pulse = 0;
        #1;


        // ========================================================
        // TEST 3: RTS WHILE BUS-OFF
        // ========================================================

        $display("========================================");
        $display("TEST 3: RTS DURING BUS-OFF");
        $display("========================================");

        eng_bus_off = 1;
        rts_pulse = 1;
        #1;

        if (txreq_set !== 1'b0)
            $display("FAIL: txreq_set should be 0 during Bus-Off");

        else
            $display("PASS: txreq_set blocked during Bus-Off");

        if (tx_start !== 1'b0)
            $display("FAIL: tx_start should be 0 during Bus-Off");

        else
            $display("PASS: tx_start blocked during Bus-Off");

        rts_pulse = 0;
        eng_bus_off = 0;
        #1;


        // ========================================================
        // TEST 4: TX DONE
        // ========================================================

        $display("========================================");
        $display("TEST 4: TX DONE");
        $display("========================================");

        tx_done = 1;
        #1;

        if (txreq_clear !== 1'b1)
            $display("FAIL: txreq_clear should be 1");

        else
            $display("PASS: txreq_clear asserted");

        if (set_tx0if !== 1'b1)
            $display("FAIL: set_tx0if should be 1");

        else
            $display("PASS: set_tx0if asserted");

        if (clear_tx0if !== 1'b0)
            $display("FAIL: clear_tx0if should be 0");

        else
            $display("PASS: clear_tx0if remains 0");

        tx_done = 0;
        #1;


        // ========================================================
        // TEST 5: TX ABORT
        // ========================================================

        $display("========================================");
        $display("TEST 5: TX ABORT");
        $display("========================================");

        tx_abort = 1;
        #1;

        if (set_merrf !== 1'b1)
            $display("FAIL: set_merrf should be 1");

        else
            $display("PASS: set_merrf asserted");

        if (set_errif !== 1'b1)
            $display("FAIL: set_errif should be 1");

        else
            $display("PASS: set_errif asserted");

        if (clear_merrf !== 1'b0)
            $display("FAIL: clear_merrf should be 0");

        else
            $display("PASS: clear_merrf remains 0");

        tx_abort = 0;
        #1;


        // ========================================================
        // TEST 6: RX SUCCESS
        // ========================================================

        $display("========================================");
        $display("TEST 6: RX SUCCESS");
        $display("========================================");

        rx_success = 1;
        #1;

        if (set_rx0if !== 1'b1)
            $display("FAIL: set_rx0if should be 1");

        else
            $display("PASS: set_rx0if asserted");

        rx_success = 0;
        #1;


        // ========================================================
        // TEST 7: RX BUFFER READ DONE
        // ========================================================

        $display("========================================");
        $display("TEST 7: RX BUFFER READ DONE");
        $display("========================================");

        rxbuf_done = 1;
        #1;

        if (clear_rx0if !== 1'b1)
            $display("FAIL: clear_rx0if should be 1");

        else
            $display("PASS: clear_rx0if asserted");

        rxbuf_done = 0;
        #1;


        // ========================================================
        // TEST 8: ERROR COUNTER MIRRORS
        // ========================================================

        $display("========================================");
        $display("TEST 8: ERROR COUNTERS");
        $display("========================================");

        eng_tec = 8'd130;
        eng_rec = 8'd10;
        #1;

        if (tec !== 8'd130)
            $display("FAIL: TEC mismatch");

        else
            $display("PASS: TEC = 130");

        if (rec !== 8'd10)
            $display("FAIL: REC mismatch");

        else
            $display("PASS: REC = 10");


        // ========================================================
        // TEST 9: ERROR FLAG STATUS
        // ========================================================

        $display("========================================");
        $display("TEST 9: ERROR FLAG STATUS");
        $display("========================================");

        eng_err_passive = 1;
        eng_bus_off = 0;
        #1;

        if (eflg !== 8'b01000000)
            $display("FAIL: EFLG error-passive value incorrect");

        else
            $display("PASS: EFLG error-passive correct");

        eng_err_passive = 0;
        eng_bus_off = 1;
        #1;

        if (eflg !== 8'b10000000)
            $display("FAIL: EFLG bus-off value incorrect");

        else
            $display("PASS: EFLG bus-off correct");

        eng_bus_off = 0;
        #1;


        // ========================================================
        // TEST 10: STATUS BYTE
        // ========================================================

        $display("========================================");
        $display("TEST 10: STATUS BYTE");
        $display("========================================");

        // RX0IF = 1
        // TX0REQ = 1
        // TX0IF = 1

        canintf = 8'b00000101;
        txreq = 1;

        #1;

        if (status_byte !== 8'b10110000)
            $display("FAIL: status_byte incorrect");

        else
            $display("PASS: status_byte correct");


        // ========================================================
        // TEST 11: RX STATUS BYTE
        // ========================================================

        $display("========================================");
        $display("TEST 11: RX STATUS BYTE");
        $display("========================================");

        canintf = 8'h01;
        rx_full = 1;
        rx_rtr = 1;
        rx_ide = 1;
        rx_filter_hit = 4'hA;

        #1;

        if (rxstatus_byte !== 8'b11111010)
            $display("FAIL: rxstatus_byte incorrect");

        else
            $display("PASS: rxstatus_byte correct");


        // ========================================================
        // TEST 12: INTERRUPT PIN
        // ========================================================

        $display("========================================");
        $display("TEST 12: INTERRUPT PIN");
        $display("========================================");

        // CANINTF bit 0 = 1
        // CANINTE bit 0 = 1
        // Therefore INT must be LOW.

        canintf = 8'h01;
        caninte = 8'h01;

        #1;

        if (int_pin !== 1'b0)
            $display("FAIL: int_pin should be LOW");

        else
            $display("PASS: int_pin asserted LOW");


        // Disable interrupt

        caninte = 8'h00;
        #1;

        if (int_pin !== 1'b1)
            $display("FAIL: int_pin should be HIGH");

        else
            $display("PASS: int_pin deasserted HIGH");


        // ========================================================
        // TEST 13: SOFTWARE RESET
        // ========================================================

        $display("========================================");
        $display("TEST 13: SOFTWARE RESET");
        $display("========================================");

        reset_pulse = 1;
        #1;

        if (control_reset !== 1'b1)
            $display("FAIL: control_reset should be asserted");

        else
            $display("PASS: software reset asserted");

        reset_pulse = 0;
        #1;


        // ========================================================
        // DONE
        // ========================================================

        $display("========================================");
        $display("SIMULATION COMPLETE");
        $display("========================================");

        $stop;

    end

endmodule