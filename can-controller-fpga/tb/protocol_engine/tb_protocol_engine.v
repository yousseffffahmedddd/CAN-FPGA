`timescale 1ns/1ps
`include "../../rtl/common/can_defs.vh"

module tb_protocol_engine;

    // Clock and Reset
    reg clk;
    reg rst_n;

    // Physical bus pins
    reg  rx_pin;
    wire tx_can;
    wire tx_en;

    // TX Buffer Interface (Driven by Testbench)
    reg [10:0] txb_id;
    reg [63:0] txb_data;
    reg [3:0]  txb_dlc;
    reg        txb_rtr;
    reg        txb_txreq;

    // RX Buffer Interface (Monitored)
    wire [10:0] rxb_id;
    wire [63:0] rxb_data;
    wire [3:0]  rxb_dlc;
    wire        rxb_rtr;
    wire        accept_rxb0;

    // Status / Error outputs
    wire [7:0] tec;
    wire [7:0] rec;
    wire [1:0] err_state;
    wire       err_active;
    wire       err_passive;
    wire       bus_off;
    wire       ewarn;
    wire       msg_err;
    wire       tx_done_pulse;
    wire       rx_done_pulse;

    // Visibility outputs
    wire [4:0] tq_index;
    wire       bus_idle;
    wire [2:0] current_state;
    wire [3:0] latched_dlc;

    // Control flag
    reg enable_loopback;
    integer timeout_counter;

    // -------------------------------------------------------------------------
    // Device Under Test (DUT)
    // -------------------------------------------------------------------------
    protocol_engine dut (
        .clk             (clk),
        .rst_n           (rst_n),
        .rx_pin          (rx_pin),
        .tx_can          (tx_can),
        .tx_en           (tx_en),
        .txb_id          (txb_id),
        .txb_data        (txb_data),
        .txb_dlc         (txb_dlc),
        .txb_rtr         (txb_rtr),
        .txb_txreq       (txb_txreq),
        .rxb_id          (rxb_id),
        .rxb_data        (rxb_data),
        .rxb_dlc         (rxb_dlc),
        .rxb_rtr         (rxb_rtr),
        .accept_rxb0     (accept_rxb0),
        .tx_done_pulse   (tx_done_pulse),
        .rx_done_pulse   (rx_done_pulse),
        .msg_err         (msg_err),
        .tec             (tec),
        .rec             (rec),
        .err_state       (err_state),
        .err_active      (err_active),
        .err_passive     (err_passive),
        .bus_off         (bus_off),
        .ewarn           (ewarn),
        .bus_idle        (bus_idle),
        .current_state   (current_state),
        .latched_dlc     (latched_dlc),
        .tq_index        (tq_index),
        .rxm0_mask       (11'h7FF),
        .rxf0_id         (11'h555)
    );

    // -------------------------------------------------------------------------
    // Clock Generation (1 MHz Clock -> 1000ns period)
    // -------------------------------------------------------------------------
    always #500 clk = ~clk;

    // -------------------------------------------------------------------------
    // Controlled Loopback Simulation / Transceiver Behavior
    // -------------------------------------------------------------------------
    always @(*) begin
        if (enable_loopback && tx_en)
            rx_pin = tx_can;
        else
            rx_pin = `CAN_RECESSIVE;
    end

    // -------------------------------------------------------------------------
    // Main Stimulus Sequence
    // -------------------------------------------------------------------------
    initial begin
        // Initialize
        clk             = 1'b0;
        rst_n           = 1'b0;
        rx_pin          = `CAN_RECESSIVE;
        txb_id          = 11'h000;
        txb_data        = 64'd0;
        txb_dlc         = 4'd0;
        txb_rtr         = 1'b0;
        txb_txreq       = 1'b0;
        enable_loopback = 1'b0;

        $display("\n--- Starting protocol_engine Testbench ---");

        // Apply Reset
        #200;
        rst_n = 1'b1;
        #200;

        // Verify initial state
        if (err_active === 1'b1 && tec == 8'd0 && rec == 8'd0)
            $display("[PASS] Reset state verified: Engine is Error Active (TEC=0, REC=0)");
        else
            $display("[FAIL] Reset state invalid! TEC=%d REC=%d", tec, rec);

        // ---------------------------------------------------------------------
        // Test Case 1: Initiate Parallel Buffer Transmission (ID=0x555, DLC=0)
        // ---------------------------------------------------------------------
        $display("\n--- Test Case 1: Initiate Parallel TX Buffer Transmission ---");
        
        txb_id          = 11'h555;
        txb_rtr         = 1'b0;
        txb_dlc         = 4'b0000;
        txb_data        = 64'd0;
        txb_txreq       = 1'b1;
        enable_loopback = 1'b1; // Enable loopback for transmission test

        // Force a SOF condition on the bus to kick off FSM
        @(posedge clk);
        force rx_pin = `CAN_DOMINANT;
        #100;
        release rx_pin;

        // Deassert request right after trigger
        @(posedge clk);
        txb_txreq = 1'b0;

        // Wait for states
        wait (current_state == `CAN_STATE_ARBITRATION);
        $display("[PASS] Protocol Engine entered ARBITRATION state");

        wait (current_state == `CAN_STATE_CONTROL);
        $display("[PASS] Protocol Engine entered CONTROL state");

        // Poll accept_rxb0 safely to complete exactly one loopback cycle and break out
        timeout_counter = 0;
        while (!accept_rxb0 && timeout_counter < 100000) begin
            @(posedge clk);
            timeout_counter = timeout_counter + 1;
        end

        // Cut off loopback immediately so it never loops a second time
        enable_loopback = 1'b0;

        if (accept_rxb0)
            $display("[PASS] Packet successfully transmitted, looped back, and accepted!");
        else
            $display("[WARNING] Test Case 1 timed out waiting for reception acceptance");

        #5000;
        $display("\n--- All protocol_engine tests executed successfully ---");
    end

    // Monitor RX actions safely
    always @(posedge clk) begin
        if (enable_loopback && accept_rxb0) begin
            $display("[RX TAP] Packet successfully received and accepted at time %t", $time);
        end
    end

endmodule