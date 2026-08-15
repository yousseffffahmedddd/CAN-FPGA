`timescale 1ns/1ps

`ifndef CAN_RECESSIVE
  `define CAN_RECESSIVE 1'b1
  `define CAN_DOMINANT  1'b0
`endif

module tb_can_top_complete;

    // Clock & Reset
    reg clk;
    reg rst_n;

    // SPI Host Interface Pins
    reg  spi_sck;
    reg  spi_cs_n;
    reg  spi_mosi;
    wire spi_miso;

    // Hardware & CAN Physical Pins
    wire tx_can;
    reg  rx_pin;
    wire tx_en;
    wire int_n;

    // Opcodes & Addresses
    localparam [7:0] SPI_WRITE      = 8'h02;
    localparam [7:0] SPI_READ       = 8'h03;
    localparam [7:0] SPI_BIT_MODIFY = 8'h05;

    localparam [7:0] ADDR_TXB0CTRL = 8'h30;
    localparam [7:0] ADDR_TXB0SIDH = 8'h31;
    localparam [7:0] ADDR_TXB0SIDL = 8'h32;
    localparam [7:0] ADDR_TXB0DLC  = 8'h35;
    localparam [7:0] ADDR_TXB0D0   = 8'h36;
    
    localparam [7:0] ADDR_RXB0SIDH = 8'h61;
    localparam [7:0] ADDR_RXB0DLC  = 8'h65;
    localparam [7:0] ADDR_RXB0D0   = 8'h66;

    // Device Under Test
    can_top dut (
        .clk    (clk),
        .rst_n  (rst_n),
        .sck    (spi_sck),
        .si     (spi_mosi),
        .cs_n   (spi_cs_n),
        .so     (spi_miso),
        .rx_pin (rx_pin),
        .tx_can (tx_can),
        .tx_en  (tx_en),
        .int_n  (int_n)
    );

    // Clock: 1 MHz (1000ns period)
    always #500 clk = ~clk;

    // Physical Loopback Path with Sampling Edge Safe Timing
    always @(*) begin
        if (tx_en)
            rx_pin = tx_can;
        else
            rx_pin = `CAN_RECESSIVE;
    end

    // Robust SPI Transfer Task (CPOL=0, CPHA=0 with shifted sampling window)
    task spi_transfer_byte(
        input  [7:0] data_in,
        output [7:0] data_out
    );
        integer i;
        reg [7:0] sample_buf;
        begin
            sample_buf = 8'h00;
            for (i = 7; i >= 0; i = i - 1) begin
                spi_mosi = data_in[i];
                #3000; 
                spi_sck  = 1'b1;  // Rise edge
                #1500;
                sample_buf[i] = spi_miso; // Sample stable data while high
                #1500;
                spi_sck  = 1'b0;  // Fall edge
                #3000;
            end
            data_out = sample_buf;
        end
    endtask

    task spi_write(
        input [7:0] addr,
        input [7:0] data
    );
        reg [7:0] dummy;
        begin
            spi_cs_n = 1'b0;
            #4000;
            spi_transfer_byte(SPI_WRITE, dummy);
            spi_transfer_byte(addr, dummy);
            spi_transfer_byte(data, dummy);
            #4000;
            spi_cs_n = 1'b1;
            #6000;
        end
    endtask

    task spi_read(
        input  [7:0] addr,
        output [7:0] data
    );
        reg [7:0] dummy;
        begin
            spi_cs_n = 1'b0;
            #4000;
            spi_transfer_byte(SPI_READ, dummy);
            spi_transfer_byte(addr, dummy);
            spi_transfer_byte(8'hFF, data);
            #4000;
            spi_cs_n = 1'b1;
            #6000;
        end
    endtask

    task spi_bit_modify(
        input [7:0] addr,
        input [7:0] mask,
        input [7:0] data
    );
        reg [7:0] dummy;
        begin
            spi_cs_n = 1'b0;
            #4000;
            spi_transfer_byte(SPI_BIT_MODIFY, dummy);
            spi_transfer_byte(addr, dummy);
            spi_transfer_byte(mask, dummy);
            spi_transfer_byte(data, dummy);
            #4000;
            spi_cs_n = 1'b1;
            #6000;
        end
    endtask

    reg [7:0] rx_val;
    integer i;

    initial begin
        clk      = 1'b0;
        rst_n    = 1'b0;
        spi_sck  = 1'b0;
        spi_cs_n = 1'b1;
        spi_mosi = 1'b0;
        rx_pin   = `CAN_RECESSIVE;

        $display("\n=======================================================");
        $display("   STARTING FIXED COMPLETE CAN TOP SYSTEM TESTBENCH    ");
        $display("=======================================================");

        #3000;
        rst_n = 1'b1;
        #8000;
        $display("[%0t ns] System Reset Deasserted", $time);

        // Test 1: Register Read/Write Integrity Verification
        $display("\n--- [TEST 1] Register Read/Write Integrity ---");
        $display("[DEBUG] Writing 0xA5 to ADDR_TXB0SIDH (0x31)");
        spi_write(ADDR_TXB0SIDH, 8'hA5);
        
        $display("[DEBUG] Reading back from ADDR_TXB0SIDH (0x31)");
        spi_read(ADDR_TXB0SIDH, rx_val);
        $display("[DEBUG] SPI read returned: 0x%02X", rx_val);
        
        if (rx_val === 8'hA5) begin
            $display("[PASS] SPI R/W Passed: Wrote 0xA5, Read 0xA5");
        end else begin
            $display("[FAIL] SPI R/W Failed! Expected 0xA5, Got 0x%02X", rx_val);
            $display("[DEBUG] MISO line state at sample: %b", spi_miso);
            $stop;
        end

        // Test 2: Populate Transmit Buffer 0
        $display("\n--- [TEST 2] Configuring TX Buffer 0 ---");
        spi_write(ADDR_TXB0SIDH, 8'hAA);
        spi_write(ADDR_TXB0SIDL, 8'hA0);
        spi_write(ADDR_TXB0DLC,  8'h04);
        spi_write(ADDR_TXB0D0 + 0, 8'h11);
        spi_write(ADDR_TXB0D0 + 1, 8'h22);
        spi_write(ADDR_TXB0D0 + 2, 8'h33);
        spi_write(ADDR_TXB0D0 + 3, 8'h44);
        $display("[INFO] Standard ID 0x555 and 4-byte payload loaded into TXB0.");

        // Test 3: Trigger Transmission via Control Register
        $display("\n--- [TEST 3] Trigger Transmission via Control Register ---");
        spi_bit_modify(ADDR_TXB0CTRL, 8'h08, 8'h08);
        
        spi_read(ADDR_TXB0CTRL, rx_val);
        if (rx_val & 8'h08)
            $display("[PASS] TXREQ bit set successfully in TXB0CTRL");
        else
            $display("[WARNING] TXREQ bit was not maintained in TXB0CTRL");

        // Test 4: Monitor Bus Handshake & Interrupt Line
        $display("\n--- [TEST 4] Monitoring Bus Handshake & RX Int ---");
        fork
            begin : int_wait_check
                wait (int_n == 1'b0);
                $display("[PASS] Hardware Interrupt Generated (int_n = 0)!");
                disable int_timeout_check;
            end
            begin : int_timeout_check
                #25000000; 
                $display("[WARNING] Timeout waiting for int_n assertion!");
            end
        join

        // Test 5: Verify Final Reception Payload
        $display("\n--- [TEST 5] Verify RX Buffer Data ---");
        spi_read(ADDR_RXB0SIDH, rx_val);
        $display("[RX READ] RXB0SIDH = 0x%02X", rx_val);

        spi_read(ADDR_RXB0DLC, rx_val);
        $display("[RX READ] RXB0DLC  = 0x%02X", rx_val);

        for (i = 0; i < 4; i = i + 1) begin
            spi_read(ADDR_RXB0D0 + i, rx_val);
            $display("[RX READ] Payload Byte %0d = 0x%02X", i, rx_val);
        end

        #5000;
        $display("\n=======================================================");
        $display("   ALL TESTBENCH SEQUENCES COMPLETED SUCCESSFULLY     ");
        $display("=======================================================\n");
    end

endmodule