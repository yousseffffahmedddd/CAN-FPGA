`timescale 1ns/1ps

module tb_control_logic;

    // Testbench Signals
    reg clk;
    reg rst_n;
    reg [7:0] spi_addr;
    reg [7:0] spi_wdata;
    reg       spi_we;
    wire [7:0] spi_rdata;
    reg       rts_pulse;
    reg       rx_read_pulse;
    reg       tx_done;
    reg       tx_aborted;
    reg       error_detected;
    reg [2:0] error_type;
    reg       rx_complete;
    wire      tx_req;
    wire      tx_buffer_we;
    reg       tx_buffer_rdy;
    wire      rx_buffer_clear;
    wire      int_n;

    // Instantiate Unit Under Test (UUT)
    control_logic uut (
        .clk(clk),
        .rst_n(rst_n),
        .spi_addr(spi_addr),
        .spi_wdata(spi_wdata),
        .spi_we(spi_we),
        .spi_rdata(spi_rdata),
        .rts_pulse(rts_pulse),
        .rx_read_pulse(rx_read_pulse),
        .tx_done(tx_done),
        .tx_aborted(tx_aborted),
        .error_detected(error_detected),
        .error_type(error_type),
        .rx_complete(rx_complete),
        .tx_req(tx_req),
        .tx_buffer_we(tx_buffer_we),
        .tx_buffer_rdy(tx_buffer_rdy),
        .rx_buffer_clear(rx_buffer_clear),
        .int_n(int_n)
    );

    // Clock Generation (50 MHz)
    always #10 clk = ~clk;

    initial begin
        // Initialize Signals
        clk = 0;
        rst_n = 0;
        spi_addr = 8'h00;
        spi_wdata = 8'h01;
        spi_we = 0;
        rts_pulse = 0;
        rx_read_pulse = 0;
        tx_done = 0;
        tx_aborted = 0;
        error_detected = 0;
        error_type = 3'b000;
        rx_complete = 0;
        tx_buffer_rdy = 1;

        // Apply Reset
        #40;
        rst_n = 1;
        #20;

        // Test 1: Enable Interrupts via SPI Write to CANINTE (Address 0x2B)
        @(posedge clk);
        spi_addr = 8'h2B;
        spi_wdata = 8'h01; // Enable RX0IE
        spi_we = 1;
        @(posedge clk);
        spi_we = 0;

        // Test 2: Simulate Successful Frame Reception -> Sets RX0IF & asserts INT pin low
        #30;
        @(posedge clk);
        rx_complete = 1;
        @(posedge clk);
        rx_complete = 0;

        // Verify interrupt line dropped low
        #20;
        if (int_n === 1'b0) 
            $display("[PASS] Test 2: Interrupt pin successfully asserted low upon RX completion.");
        else 
            $error("[FAIL] Test 2: Interrupt pin failed to assert.");

        // Test 3: Issue SPI RTS Pulse -> Verify TXREQ sets high
        #30;
        @(posedge clk);
        rts_pulse = 1;
        @(posedge clk);
        rts_pulse = 0;

        #20;
        if (tx_req === 1'b1) 
            $display("[PASS] Test 3: RTS pulse successfully set TXREQ flag.");
        else 
            $error("[FAIL] Test 3: TXREQ flag did not set on RTS pulse.");

        // Test 4: Simulate Transmission Complete -> Verify TXREQ clears automatically
        #30;
        @(posedge clk);
        tx_done = 1;
        @(posedge clk);
        tx_done = 0;

        #20;
        if (tx_req === 1'b0) 
            $display("[PASS] Test 4: tx_done pulse successfully cleared TXREQ flag.");
        else 
            $error("[FAIL] Test 4: TXREQ flag remained high after tx_done.");

        // Test 5: Read RX Buffer via SPI protocol interface -> Clears interrupt flag
        #30;
        @(posedge clk);
        rx_read_pulse = 1;
        @(posedge clk);
        rx_read_pulse = 0;

        #40;
        $display("[INFO] Simulation completed successfully.");
        $finish;
    end

endmodule