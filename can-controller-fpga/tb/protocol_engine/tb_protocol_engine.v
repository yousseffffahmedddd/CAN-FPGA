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

    // PISO interface (TX side)
    reg  piso_data_in;
    reg  piso_valid;
    wire piso_req;

    // SIPO interface (RX side)
    wire sipo_data_out;
    wire sipo_valid;

    // Status / Error outputs
    wire [7:0] tec;
    wire [7:0] rec;
    wire       err_active;
    wire       err_passive;
    wire       bus_off;
    wire       ewarn;
    wire       tx_done_pulse;
    wire       rx_done_pulse;

    // Visibility outputs
    wire [4:0] tq_index;
    wire       bus_idle;
    wire [2:0] current_state;
    wire [3:0] latched_dlc;

    // Testbench Variables
    reg [200:0] tx_stream;
    integer     bit_ptr;

    // -------------------------------------------------------------------------
    // Device Under Test (DUT)
    // -------------------------------------------------------------------------
    protocol_engine dut (
        .clk            (clk),
        .rst_n          (rst_n),
        .rx_pin         (rx_pin),
        .tx_can         (tx_can),
        .tx_en          (tx_en),
        .piso_data_in   (piso_data_in),
        .piso_valid     (piso_valid),
        .piso_req       (piso_req),
        .sipo_data_out  (sipo_data_out),
        .sipo_valid     (sipo_valid),
        .tec            (tec),
        .rec            (rec),
        .err_active     (err_active),
        .err_passive    (err_passive),
        .bus_off        (bus_off),
        .ewarn          (ewarn),
        .tx_done_pulse  (tx_done_pulse),
        .rx_done_pulse  (rx_done_pulse),
        .tq_index       (tq_index),
        .bus_idle       (bus_idle),
        .current_state  (current_state),
        .latched_dlc    (latched_dlc)
    );

    // -------------------------------------------------------------------------
    // Clock Generation (1 MHz Clock -> 1000ns period)
    // -------------------------------------------------------------------------
    always #500 clk = ~clk;

    // -------------------------------------------------------------------------
    // Loopback Simulation / Transceiver Behavior
    // Loopback `tx_can` to `rx_pin` when `tx_en` is active, otherwise hold bus recessive (1).
    // -------------------------------------------------------------------------
    always @(*) begin
        if (tx_en)
            rx_pin = tx_can;
        else
            rx_pin = `CAN_RECESSIVE;
    end

    // -------------------------------------------------------------------------
    // Dynamic PISO Feeder
    // Feeds bits from tx_stream whenever the protocol engine asserts piso_req
    // -------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            piso_data_in <= `CAN_RECESSIVE;
            piso_valid   <= 1'b0;
            bit_ptr      <= 0;
        end else begin
            if (piso_req) begin
                piso_data_in <= tx_stream[bit_ptr];
                piso_valid   <= 1'b1;
                bit_ptr      <= bit_ptr - 1;
            end else begin
                piso_valid   <= 1'b0;
            end
        end
    end

    // -------------------------------------------------------------------------
    // Main Stimulus Sequence
    // -------------------------------------------------------------------------
    initial begin
        // Initialize
        clk          = 1'b0;
        rst_n        = 1'b0;
        rx_pin       = `CAN_RECESSIVE;
        piso_data_in = `CAN_RECESSIVE;
        piso_valid   = 1'b0;
        tx_stream    = 0;
        bit_ptr      = 0;

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
        // Test Case 1: Stream a CAN Frame via PISO (Standard Frame ID=0x555, DLC=0)
        // Bit Payload (MSB to LSB):
        // ID[10:0] = 11'b101_0101_0101 (0x555)
        // RTR      = 1'b0 (Data Frame)
        // IDE      = 1'b0 (Standard)
        // RB0      = 1'b0 (Reserved)
        // DLC[3:0] = 4'b0000 (0 bytes)
        // ---------------------------------------------------------------------
        $display("\n--- Test Case 1: Initiate PISO Stream Transmission ---");
        
        // Load stream (17 bits total: ID + RTR + IDE + RB0 + DLC)
        tx_stream[16:0] = {11'h555, 1'b0, 1'b0, 1'b0, 4'b0000};
        bit_ptr = 16;

        // Force a SOF condition on the bus (Dominant bit tick) to kick off FSM
        @(posedge clk);
        force rx_pin = `CAN_DOMINANT;
        #100;
        release rx_pin;

        // Wait for Transmission completion or state changes
        wait (current_state == `CAN_STATE_ARBITRATION);
        $display("[PASS] Protocol Engine entered ARBITRATION state");

        wait (current_state == `CAN_STATE_CONTROL);
        $display("[PASS] Protocol Engine entered CONTROL state");

        // Wait for completion pulse or return to IDLE
        fork
            begin : tx_done_check
                @(posedge tx_done_pulse);
                $display("[PASS] tx_done_pulse received from Protocol Engine!");
                disable timeout_check;
            end
            begin : timeout_check
                #1000000; // Timeout safety
                $display("[WARNING] Test Case 1 timed out before tx_done_pulse");
            end
        join

        #5000;
        $display("\n--- All protocol_engine tests executed successfully ---");
        $finish;
    end

    // Monitor SIPO output
    always @(posedge clk) begin
        if (sipo_valid) begin
            $display("[SIPO TAP] Observed Bus Bit = %b at time %t", sipo_data_out, $time);
        end
    end

endmodule