`timescale 1ns / 1ps

module tb_control_logic;

    // -------------------------------------------------------------------------
    // DUT inputs
    // -------------------------------------------------------------------------
    reg         clk;
    reg         rst_n;

    reg         rts_pulse;
    reg         reset_pulse;
    reg         rxbuf_done;

    reg  [7:0]  canctrl;
    reg  [7:0]  bfpctrl;
    reg  [7:0]  caninte;
    reg  [7:0]  canintf;
    reg         txreq;

    reg         tx_done;
    reg         tx_abort;
    reg         rx_success;
    reg  [7:0]  eng_tec;
    reg  [7:0]  eng_rec;
    reg         eng_err_passive;
    reg         eng_bus_off;

    reg         rx_full;
    reg         rx_rtr;
    reg         rx_ide;
    reg  [3:0]  rx_filter_hit;

    // -------------------------------------------------------------------------
    // DUT outputs
    // -------------------------------------------------------------------------
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

    integer checks;
    integer errors;

    // CANINTF bit positions used by the reduced design.
    localparam integer RX0IF = 0;
    localparam integer TX0IF = 2;
    localparam integer ERRIF = 5;
    localparam integer MERRF = 7;

    control_logic dut (
        .clk             (clk),
        .rst_n           (rst_n),

        .rts_pulse       (rts_pulse),
        .reset_pulse     (reset_pulse),
        .rxbuf_done      (rxbuf_done),

        .canctrl         (canctrl),
        .bfpctrl         (bfpctrl),
        .caninte         (caninte),
        .canintf         (canintf),
        .txreq           (txreq),

        .tx_done         (tx_done),
        .tx_abort        (tx_abort),
        .rx_success      (rx_success),
        .eng_tec         (eng_tec),
        .eng_rec         (eng_rec),
        .eng_err_passive (eng_err_passive),
        .eng_bus_off     (eng_bus_off),

        .rx_full         (rx_full),
        .rx_rtr          (rx_rtr),
        .rx_ide          (rx_ide),
        .rx_filter_hit   (rx_filter_hit),

        .control_reset   (control_reset),
        .txreq_set       (txreq_set),
        .txreq_clear     (txreq_clear),
        .set_rx0if       (set_rx0if),
        .clear_rx0if     (clear_rx0if),
        .set_tx0if       (set_tx0if),
        .clear_tx0if     (clear_tx0if),
        .set_merrf       (set_merrf),
        .clear_merrf     (clear_merrf),
        .set_errif       (set_errif),
        .tx_start        (tx_start),
        .status_byte     (status_byte),
        .rxstatus_byte   (rxstatus_byte),
        .tec             (tec),
        .rec             (rec),
        .eflg            (eflg),
        .int_pin         (int_pin)
    );

    always #5 clk = ~clk;

    task check;
        input condition;
        input [8*96-1:0] message;
        begin
            checks = checks + 1;
            if (condition) begin
                $display("[PASS] %0t : %0s", $time, message);
            end
            else begin
                errors = errors + 1;
                $display("[FAIL] %0t : %0s", $time, message);
            end
        end
    endtask

    task clear_events;
        begin
            rts_pulse   = 1'b0;
            reset_pulse = 1'b0;
            rxbuf_done  = 1'b0;
            tx_done     = 1'b0;
            tx_abort    = 1'b0;
            rx_success  = 1'b0;
        end
    endtask

    initial begin
        clk             = 1'b0;
        rst_n           = 1'b0;

        rts_pulse       = 1'b0;
        reset_pulse     = 1'b0;
        rxbuf_done      = 1'b0;

        canctrl         = 8'h00;
        bfpctrl         = 8'h00;
        caninte         = 8'h00;
        canintf         = 8'h00;
        txreq           = 1'b0;

        tx_done         = 1'b0;
        tx_abort        = 1'b0;
        rx_success      = 1'b0;
        eng_tec         = 8'h00;
        eng_rec         = 8'h00;
        eng_err_passive = 1'b0;
        eng_bus_off     = 1'b0;

        rx_full         = 1'b0;
        rx_rtr          = 1'b0;
        rx_ide          = 1'b0;
        rx_filter_hit   = 4'h0;

        checks = 0;
        errors = 0;

        // ---------------------------------------------------------------------
        // 1. Hardware reset and reset gating
        // ---------------------------------------------------------------------
        caninte    = 8'hFF;
        canintf    = 8'hFF;
        rts_pulse  = 1'b1;
        tx_done    = 1'b1;
        tx_abort   = 1'b1;
        rx_success = 1'b1;
        rxbuf_done = 1'b1;
        #1;

        check(control_reset === 1'b1, "hardware reset asserts control_reset");
        check(txreq_set     === 1'b0, "hardware reset blocks TXREQ set");
        check(txreq_clear   === 1'b0, "hardware reset blocks TXREQ clear event");
        check(tx_start      === 1'b0, "hardware reset blocks tx_start");
        check(set_tx0if     === 1'b0, "hardware reset blocks TX interrupt event");
        check(set_rx0if     === 1'b0, "hardware reset blocks RX interrupt event");
        check(set_merrf     === 1'b0, "hardware reset blocks MERRF event");
        check(set_errif     === 1'b0, "hardware reset blocks ERRIF event");
        check(int_pin       === 1'b1, "INT is inactive during hardware reset");

        clear_events;
        caninte = 8'h00;
        canintf = 8'h00;
        @(negedge clk);
        rst_n = 1'b1;
        @(posedge clk);
        #1;
        check(control_reset === 1'b0, "control_reset releases after hardware reset");

        // ---------------------------------------------------------------------
        // 2. Software RESET pulse
        // ---------------------------------------------------------------------
        @(negedge clk);
        caninte     = 8'hFF;
        canintf     = 8'hFF;
        reset_pulse = 1'b1;
        rts_pulse   = 1'b1;
        tx_done     = 1'b1;
        tx_abort    = 1'b1;
        rx_success  = 1'b1;
        rxbuf_done  = 1'b1;
        #1;

        check(control_reset === 1'b1, "software RESET asserts control_reset");
        check(txreq_set     === 1'b0, "software RESET blocks TXREQ set");
        check(txreq_clear   === 1'b0, "software RESET blocks TXREQ clear event");
        check(tx_start      === 1'b0, "software RESET blocks tx_start");
        check(set_tx0if     === 1'b0, "software RESET blocks TX interrupt event");
        check(set_rx0if     === 1'b0, "software RESET blocks RX interrupt event");
        check(set_merrf     === 1'b0, "software RESET blocks MERRF event");
        check(set_errif     === 1'b0, "software RESET blocks ERRIF event");
        check(int_pin       === 1'b1, "INT is inactive during software RESET");

        @(posedge clk);
        #1;
        clear_events;
        caninte = 8'h00;
        canintf = 8'h00;
        #1;
        check(control_reset === 1'b0, "software RESET releases cleanly");

        // ---------------------------------------------------------------------
        // 3. RTS in Normal mode
        // ---------------------------------------------------------------------
        @(negedge clk);
        eng_bus_off = 1'b0;
        rts_pulse   = 1'b1;
        #1;
        check(txreq_set   === 1'b1, "RTS asserts txreq_set in Normal mode");
        check(tx_start    === 1'b1, "RTS asserts tx_start in Normal mode");
        check(txreq_clear === 1'b0, "RTS alone does not clear TXREQ");

        rts_pulse = 1'b0;
        #1;
        check(txreq_set === 1'b0 && tx_start === 1'b0,
              "RTS outputs deassert when rts_pulse deasserts");

        // ---------------------------------------------------------------------
        // 4. Bus-Off entry and RTS blocking
        // ---------------------------------------------------------------------
        @(negedge clk);
        eng_bus_off = 1'b1;
        #1;
        check(set_errif === 1'b1, "Bus-Off entry raises ERRIF event");
        check(eflg      === 8'h80, "EFLG reports Bus-Off");

        @(posedge clk);
        #1;
        check(set_errif === 1'b0, "Bus-Off ERRIF event is edge-based");

        rts_pulse = 1'b1;
        #1;
        check(txreq_set === 1'b0, "Bus-Off blocks txreq_set");
        check(tx_start  === 1'b0, "Bus-Off blocks tx_start");
        rts_pulse = 1'b0;

        @(negedge clk);
        eng_bus_off = 1'b0;
        #1;

        // ---------------------------------------------------------------------
        // 5. Transmission success
        // ---------------------------------------------------------------------
        tx_done = 1'b1;
        #1;
        check(txreq_clear === 1'b1, "tx_done asserts txreq_clear");
        check(set_tx0if   === 1'b1, "tx_done sets TX0IF event");
        check(set_merrf   === 1'b0, "tx_done does not set MERRF");
        check(set_errif   === 1'b0, "tx_done does not set ERRIF");
        check(clear_tx0if === 1'b0, "Control Logic never autonomously clears TX0IF");
        tx_done = 1'b0;

        // ---------------------------------------------------------------------
        // 6. Transmission abort/error
        // ---------------------------------------------------------------------
        tx_abort = 1'b1;
        #1;
        check(txreq_clear === 1'b1, "tx_abort asserts txreq_clear");
        check(set_merrf   === 1'b1, "tx_abort sets MERRF event");
        check(set_errif   === 1'b1, "tx_abort sets ERRIF event");
        check(set_tx0if   === 1'b0, "tx_abort does not set TX0IF");
        check(clear_merrf === 1'b0, "Control Logic never autonomously clears MERRF");
        tx_abort = 1'b0;

        // ---------------------------------------------------------------------
        // 7. Same-cycle RTS and TX completion
        // ---------------------------------------------------------------------
        rts_pulse = 1'b1;
        tx_done   = 1'b1;
        #1;
        check(txreq_set   === 1'b1, "same-cycle RTS/tx_done exports TXREQ set event");
        check(txreq_clear === 1'b1, "same-cycle RTS/tx_done exports TXREQ clear event");
        check(tx_start    === 1'b1, "same-cycle RTS/tx_done still starts new request");
        check(set_tx0if   === 1'b1, "same-cycle RTS/tx_done records completed TX interrupt");
        rts_pulse = 1'b0;
        tx_done   = 1'b0;

        // Register Bank resolves simultaneous TXREQ set/clear with SET priority.

        // ---------------------------------------------------------------------
        // 8. RX set/clear events and simultaneous race
        // ---------------------------------------------------------------------
        rx_success = 1'b1;
        #1;
        check(set_rx0if   === 1'b1, "rx_success sets RX0IF event");
        check(clear_rx0if === 1'b0, "rx_success alone does not clear RX0IF");
        rx_success = 1'b0;

        rxbuf_done = 1'b1;
        #1;
        check(clear_rx0if === 1'b1, "READ RX BUFFER completion clears RX0IF");
        check(set_rx0if   === 1'b0, "RX read completion alone does not set RX0IF");
        rxbuf_done = 1'b0;

        rx_success = 1'b1;
        rxbuf_done = 1'b1;
        #1;
        check(set_rx0if   === 1'b1, "simultaneous RX success/read exports RX0IF set");
        check(clear_rx0if === 1'b1, "simultaneous RX success/read exports RX0IF clear");
        rx_success = 1'b0;
        rxbuf_done = 1'b0;

        // Register Bank resolves simultaneous CANINTF set/clear with SET priority.

        // ---------------------------------------------------------------------
        // 9. TEC/REC mirrors and passive threshold
        // ---------------------------------------------------------------------
        @(negedge clk);
        eng_tec         = 8'd127;
        eng_rec         = 8'd20;
        eng_err_passive = 1'b0;
        #1;
        check(tec  === 8'd127, "TEC mirrors Protocol Engine counter");
        check(rec  === 8'd20,  "REC mirrors Protocol Engine counter");
        check(eflg === 8'h00,  "EFLG remains clear below passive threshold");

        eng_tec = 8'd128;
        #1;
        check(eflg      === 8'h40, "TEC >= 128 reports Error-Passive in EFLG");
        check(set_errif === 1'b1,  "entering Error-Passive raises ERRIF event");

        @(posedge clk);
        #1;
        check(set_errif === 1'b0, "Error-Passive ERRIF event is edge-based");

        // Drop below threshold, clock it in, then test the engine passive input.
        @(negedge clk);
        eng_tec = 8'd0;
        eng_rec = 8'd0;
        eng_err_passive = 1'b0;
        @(posedge clk);
        #1;

        @(negedge clk);
        eng_err_passive = 1'b1;
        #1;
        check(eflg      === 8'h40, "engine Error-Passive state is mirrored in EFLG");
        check(set_errif === 1'b1,  "engine Error-Passive entry raises ERRIF event");
        @(posedge clk);
        #1;
        check(set_errif === 1'b0, "engine Error-Passive event does not retrigger continuously");
        eng_err_passive = 1'b0;

        // ---------------------------------------------------------------------
        // 10. READ STATUS mapping
        // ---------------------------------------------------------------------
        canintf = 8'h00;
        canintf[TX0IF] = 1'b1;
        canintf[RX0IF] = 1'b1;
        txreq = 1'b1;
        #1;
        check(status_byte === 8'h0D,
              "READ STATUS maps TX0IF=bit3 TXREQ=bit2 RX0IF=bit0");

        canintf = 8'h00;
        txreq   = 1'b0;
        #1;
        check(status_byte === 8'h00, "unsupported READ STATUS fields remain zero");

        // ---------------------------------------------------------------------
        // 11. RX STATUS mapping
        // ---------------------------------------------------------------------
        rx_full       = 1'b1;
        rx_ide        = 1'b0;
        rx_rtr        = 1'b0;
        rx_filter_hit = 4'h0;
        #1;
        check(rxstatus_byte === 8'h40,
              "RX STATUS reports pending RXB0 message in reduced design");

        // Exercise every implemented field in the unit-level mapping.
        rx_full       = 1'b1;
        rx_ide        = 1'b1;
        rx_rtr        = 1'b1;
        rx_filter_hit = 4'hA;
        #1;
        check(rxstatus_byte === 8'h5A,
              "RX STATUS maps FULL IDE RTR and low three filter-hit bits correctly");

        // ---------------------------------------------------------------------
        // 12. Interrupt masking and priority-independent physical INT
        // ---------------------------------------------------------------------
        canintf = 8'h00;
        caninte = 8'h00;
        #1;
        check(int_pin === 1'b1, "INT is high with no enabled pending interrupt");

        canintf[RX0IF] = 1'b1;
        #1;
        check(int_pin === 1'b1, "masked RX0IF does not assert INT");

        caninte[RX0IF] = 1'b1;
        #1;
        check(int_pin === 1'b0, "enabled RX0IF asserts active-low INT");

        caninte = 8'h00;
        canintf = 8'h00;
        canintf[ERRIF] = 1'b1;
        canintf[MERRF] = 1'b1;
        #1;
        check(int_pin === 1'b1, "masked error flags do not assert INT");

        caninte[ERRIF] = 1'b1;
        #1;
        check(int_pin === 1'b0, "enabled ERRIF asserts active-low INT");

        // RESET must override an otherwise active interrupt.
        reset_pulse = 1'b1;
        #1;
        check(int_pin === 1'b1, "software RESET forces INT inactive");
        reset_pulse = 1'b0;
        #1;
        check(int_pin === 1'b0, "INT returns low when enabled pending flag remains");

        // ---------------------------------------------------------------------
        // Final result
        // ---------------------------------------------------------------------
        $display("");
        $display("============================================================");
        $display("CONTROL LOGIC TESTS: %0d checks, %0d errors", checks, errors);
        if (errors == 0)
            $display("RESULT: PASS");
        else
            $display("RESULT: FAIL");
        $display("============================================================");

        $finish;
    end

endmodule
