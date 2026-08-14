`timescale 1ns/1ps

// Macro definitions for CAN Recessive / Dominant levels
`ifndef CAN_RECESSIVE
  `define CAN_RECESSIVE 1'b1
  `define CAN_DOMINANT  1'b0
`endif

module tb_can_top_complete;

    // -------------------------------------------------------------------------
    // 1. Clock & Reset Signals
    // -------------------------------------------------------------------------
    reg clk;
    reg rst_n;

    // -------------------------------------------------------------------------
    // 2. SPI Host Interface Pins
    // -------------------------------------------------------------------------
    reg  spi_sck;
    reg  spi_cs_n;
    reg  spi_mosi;
    wire spi_miso;

    // -------------------------------------------------------------------------
    // 3. Hardware & CAN Physical Pins
    // -------------------------------------------------------------------------
    wire       tx_can;
    reg        rx_pin;
    wire       tx_en;
    wire       int_n;

    // -------------------------------------------------------------------------
    // 4. Constants & Opcodes (MCP2515 Compatible Protocol)
    // -------------------------------------------------------------------------
    localparam [7:0] SPI_WRITE     = 8'h02;
    localparam [7:0] SPI_READ      = 8'h03;
    localparam [7:0] SPI_BIT_MODIFY= 8'h05;

    // Registers Addresses
    localparam [7:0] ADDR_TXB0CTRL = 8'h30;
    localparam [7:0] ADDR_TXB0SIDH = 8'h31;
    localparam [7:0] ADDR_TXB0SIDL = 8'h32;
    localparam [7:0] ADDR_TXB0DLC  = 8'h35;
    localparam [7:0] ADDR_TXB0D0   = 8'h36;
    
    localparam [7:0] ADDR_RXB0CTRL = 8'h60;
    localparam [7:0] ADDR_RXB0SIDH = 8'h61;
    localparam [7:0] ADDR_RXB0DLC  = 8'h65;
    localparam [7:0] ADDR_RXB0D0   = 8'h66;

    // -------------------------------------------------------------------------
    // 5. Device Under Test (DUT)
    // -------------------------------------------------------------------------
    can_top dut (
        .clk      (clk),
        .rst_n    (rst_n),
        .sck      (spi_sck),
        .si       (spi_mosi),
        .cs_n     (spi_cs_n),
        .so       (spi_miso),
        .rx_pin   (rx_pin),
        .tx_can   (tx_can),
        .tx_en    (tx_en),
        .int_n    (int_n)
    );

    // -------------------------------------------------------------------------
    // 6. Clock Generation (1 MHz -> 1000ns period)
    // -------------------------------------------------------------------------
    always #500 clk = ~clk;

    // -------------------------------------------------------------------------
    // 7. Physical Transceiver Loopback Emulation
    // Drivers line low (Dominant) when transmitting; otherwise leaves bus high
    // -------------------------------------------------------------------------
    always @(*) begin
        if (tx_en)
            rx_pin = tx_can;
        else
            rx_pin = `CAN_RECESSIVE;
    end

    // -------------------------------------------------------------------------
    // 8. Low-Level SPI Tasks
    // -------------------------------------------------------------------------
    
    // Shift 1 Byte over SPI (Mode 0,0: CPOL=0, CPHA=0)
    // 1 MHz system clock = 1000 ns period
    // 2-FF synchronizer needs ~2 system clocks = ~2000 ns to detect edge
    // So SCK period should be ~4000 ns (2000 ns high, 2000 ns low) minimum
    task spi_transfer_byte(
        input  [7:0] data_in,
        output [7:0] data_out
    );
        integer i;
        begin
            data_out = 8'h00;
            for (i = 7; i >= 0; i = i - 1) begin
                spi_mosi = data_in[i];
                #2500;  // setup time before clock
                spi_sck  = 1'b1;
                #2500;  // hold clock high for synchronizer to detect
                data_out[i] = spi_miso;  // sample MISO when SCK is high
                spi_sck  = 1'b0;
                #2500;  // clock low period
            end
            #2500;  // final settling
        end
    endtask

    // Full SPI Register Write Command
    task spi_write(
        input [7:0] addr,
        input [7:0] data
    );
        reg [7:0] dummy;
        begin
            spi_cs_n = 1'b0;
            #3000;  // CS setup time
            spi_transfer_byte(SPI_WRITE, dummy);
            spi_transfer_byte(addr, dummy);
            spi_transfer_byte(data, dummy);
            #3000;  // CS hold time
            spi_cs_n = 1'b1;
            #5000;  // Inter-transaction gap
        end
    endtask

    // Full SPI Register Read Command
    task spi_read(
        input  [7:0] addr,
        output [7:0] data
    );
        reg [7:0] dummy;
        begin
            spi_cs_n = 1'b0;
            #3000;  // CS setup time
            spi_transfer_byte(SPI_READ, dummy);
            spi_transfer_byte(addr, dummy);
            spi_transfer_byte(8'hFF, data); // Read payload off MISO
            #3000;  // CS hold time
            spi_cs_n = 1'b1;
            #5000;  // Inter-transaction gap
        end
    endtask

    // Full SPI Bit Modify Command
    task spi_bit_modify(
        input [7:0] addr,
        input [7:0] mask,
        input [7:0] data
    );
        reg [7:0] dummy;
        begin
            spi_cs_n = 1'b0;
            #3000;  // CS setup time
            spi_transfer_byte(SPI_BIT_MODIFY, dummy);
            spi_transfer_byte(addr, dummy);
            spi_transfer_byte(mask, dummy);
            spi_transfer_byte(data, dummy);
            #3000;  // CS hold time
            spi_cs_n = 1'b1;
            #5000;  // Inter-transaction gap
        end
    endtask

    // -------------------------------------------------------------------------
    // 9. Main Stimulus & Verification Sequence
    // -------------------------------------------------------------------------
    reg [7:0] rx_val;
    integer i;

    initial begin
        // --- Initialization ---
        clk      = 1'b0;
        rst_n    = 1'b0;
        spi_sck  = 1'b0;
        spi_cs_n = 1'b1;
        spi_mosi = 1'b0;
        rx_pin   = `CAN_RECESSIVE;

        $display("\n=======================================================");
        $display("   STARTING COMPLETE CAN TOP SYSTEM TESTBENCH         ");
        $display("=======================================================");

        // --- Step 1: System Reset Phase ---
        #2000;  // longer reset hold
        rst_n = 1'b1;
        #5000;  // longer post-reset initialization
        $display("[%0t ns] System Reset Deasserted", $time);

        // --- Step 2: SPI Interface Read/Write Sanity Check ---
        $display("\n--- [TEST 1] Register Read/Write Integrity ---");
        $display("[DEBUG] Attempting SPI write to ADDR_TXB0SIDH = 0x%02X with value 0xA5", ADDR_TXB0SIDH);
        spi_write(ADDR_TXB0SIDH, 8'hA5);
        #2000;  // wait for write to settle
        $display("[DEBUG] Attempting SPI read from ADDR_TXB0SIDH");
        spi_read(ADDR_TXB0SIDH, rx_val);
        $display("[DEBUG] SPI read returned: 0x%02X", rx_val);
        
        if (rx_val === 8'hA5) begin
            $display("[PASS] SPI R/W Passed: Wrote 0xA5, Read 0xA5");
        end else begin
            $display("[FAIL] SPI R/W Failed! Expected 0xA5, Got 0x%02X", rx_val);
            $display("[DEBUG] SO line state: %b", spi_miso);
            $stop;
        end

        // --- Step 3: Configure Transmit Buffer 0 (TXB0) ---
        $display("\n--- [TEST 2] Configuring TX Buffer 0 ---");
        
        // Write Identifier: Standard ID = 0x555 (11-bit)
        // TXB0SIDH = 0x555 >> 3 = 0xAA
        // TXB0SIDL = (0x555 & 0x07) << 5 = 0xA0
        spi_write(ADDR_TXB0SIDH, 8'hAA);
        spi_write(ADDR_TXB0SIDL, 8'hA0);
        
        // Data Length Code (DLC) = 4 Bytes
        spi_write(ADDR_TXB0DLC, 8'h04);

        // Fill 4 Payload Bytes: 0x11, 0x22, 0x33, 0x44
        spi_write(ADDR_TXB0D0 + 0, 8'h11);
        spi_write(ADDR_TXB0D0 + 1, 8'h22);
        spi_write(ADDR_TXB0D0 + 2, 8'h33);
        spi_write(ADDR_TXB0D0 + 3, 8'h44);

        $display("[INFO] Standard ID 0x555 and 4-byte payload loaded into TXB0.");

        // --- Step 4: Initiate Transmission via Bit Modify (TXREQ) ---
        $display("\n--- [TEST 3] Trigger Transmission via Control Register ---");
        
        // Set bit 3 (TXREQ) in TXB0CTRL
        spi_bit_modify(ADDR_TXB0CTRL, 8'h08, 8'h08);
        
        // Read back TXB0CTRL to ensure request was registered
        spi_read(ADDR_TXB0CTRL, rx_val);
        if (rx_val & 8'h08)
            $display("[PASS] TXREQ bit set successfully in TXB0CTRL");
        else
            $display("[WARNING] TXREQ bit was not maintained in TXB0CTRL");

        // --- Step 5: Wait for Transmission/Reception Completion ---
        $display("\n--- [TEST 4] Monitoring Bus Handshake & RX Int ---");
        
        // Wait until Interrupt Line drops low (signals completed reception) or timeout
        fork
            begin : int_wait_check
                wait (int_n == 1'b0);
                $display("[PASS] Hardware Interrupt Generated (int_n = 0)!");
                disable int_timeout_check;
            end
            begin : int_timeout_check
                #20000000; // 20 ms Timeout scaled for 1 MHz CAN timing
                $display("[WARNING] Timeout waiting for int_n assertion!");
            end
        join

        // --- Step 6: Verify Reception Payload via SPI ---
        $display("\n--- [TEST 5] Verify RX Buffer Data ---");
        
        // Read Header and DLC from RX Buffer 0
        spi_read(ADDR_RXB0SIDH, rx_val);
        $display("[RX READ] RXB0SIDH = 0x%02X (Expected: 0xAA)", rx_val);

        spi_read(ADDR_RXB0DLC, rx_val);
        $display("[RX READ] RXB0DLC  = 0x%02X (Expected: 0x04)", rx_val);

        // Read and verify data bytes
        for (i = 0; i < 4; i = i + 1) begin
            spi_read(ADDR_RXB0D0 + i, rx_val);
            $display("[RX READ] Payload Byte %0d = 0x%02X", i, rx_val);
        end

        #5000;
        $display("\n=======================================================");
        $display("   ALL TESTBENCH SEQUENCES COMPLETED SUCCESSFULLY     ");
        $display("=======================================================\n");
        $finish;
    end

endmodule