`timescale 1ns/1ps

module tb_spi_can_e2e;

    // =========================================================================
    // 1. Clock & System Signals
    // =========================================================================
    reg clk;
    reg rst_n;

    // SPI Signals (Mode 0,0)
    reg sck;
    reg si;
    reg cs_n;
    wire so;

    // CAN Bus Signals
    wire tx_can;
    wire tx_en;
    wire rx_pin;
    wire int_n;

    // Global Test Counters
    integer pass_count = 0;
    integer fail_count = 0;

    // System Clock Generation (50 MHz System Clock)
    initial clk = 0;
    always #10 clk = ~clk;

    // Transceiver Loopback (Loop back TX to RX when TX enabled, default recessive)
    assign rx_pin = tx_en ? tx_can : 1'b1;

    // =========================================================================
    // 2. DUT Instantiation
    // =========================================================================
    can_top u_can_top (
        .clk     (clk),
        .rst_n   (rst_n),
        .sck     (sck),
        .si      (si),
        .cs_n    (cs_n),
        .so      (so),
        .rx_pin  (rx_pin),
        .tx_can  (tx_can),
        .tx_en   (tx_en),
        .int_n   (int_n)
    );

    // =========================================================================
    // 3. SPI Bit-Bang Tasks & OPCODES
    // =========================================================================
    localparam CMD_LOAD_TX0  = 8'h40; // Load TX Buffer 0
    localparam CMD_RTS_TX0   = 8'h81; // Request-To-Send TX Buffer 0

    task spi_write_byte(input [7:0] data);
        integer i;
        begin
            for (i = 7; i >= 0; i = i - 1) begin
                si = data[i];
                #100;
                sck = 1'b1;
                #100;
                sck = 1'b0;
            end
        end
    endtask

    // Helper Task to Send SPI Message Sequence
    task spi_send_can_frame(
        input [10:0] id,
        input        rtr,
        input [3:0]  dlc,
        input [63:0] data
    );
        integer b;
        begin
            // Step 1: CS Low -> Load TX Buffer
            cs_n = 1'b0;
            #100;
            spi_write_byte(CMD_LOAD_TX0);
            
            // Standard ID (11-bit) & RTR Packet Encoding
            spi_write_byte({id[10:3]});              // TXB0SIDH
            spi_write_byte({id[2:0], 2'b00, rtr, 2'b00}); // TXB0SIDL
            spi_write_byte({4'b0000, dlc});          // TXB0DLC
            
            // Send payload bytes as defined by DLC
            for (b = 0; b < dlc; b = b + 1) begin
                spi_write_byte(data[63 - (b*8) -: 8]);
            end
            
            #100;
            cs_n = 1'b1;
            #200;

            // Step 2: Request to Send (RTS) Command
            cs_n = 1'b0;
            cs_n = 1'b0;
            #100;
            spi_write_byte(CMD_RTS_TX0);
            #100;
            cs_n = 1'b1;
            #200;
        end
    endtask

    // =========================================================================
    // 4. CAN 2.0B CRC-15 Reference Function
    // =========================================================================
    function [14:0] calc_can_crc15(
        input [10:0] id,
        input        rtr,
        input [3:0]  dlc,
        input [63:0] data
    );
        reg [14:0] crc;
        reg        crc_nxt;
        integer    i, j;
        reg [127:0] bit_stream;
        integer    total_bits;
        begin
            crc = 15'h0000;
            
            // Construct bitstream: SOF(0) + ID[10:0] + RTR + IDE(0) + r0(0) + DLC[3:0] + Data
            bit_stream = {id[10:0], rtr, 1'b0, 1'b0, dlc[3:0], data};
            total_bits = 11 + 1 + 1 + 1 + 4 + (dlc * 8);

            for (i = 127; i > (128 - total_bits); i = i - 1) begin
                crc_nxt = bit_stream[i] ^ crc[14];
                crc = crc << 1;
                if (crc_nxt)
                    crc = crc ^ 15'h4599; // CAN 2.0B Polynomial
            end
            calc_can_crc15 = crc & 15'h7FFF;
        end
    endfunction

    // =========================================================================
    // 5. Signal Capture & Frame Verification
    // =========================================================================
    reg [255:0] captured_bits;
    integer     bit_idx;
    reg         capturing;

    // Field Decoding Variables
    reg [10:0] cap_id;
    reg        cap_rtr;
    reg        cap_ide;
    reg        cap_r0;
    reg [3:0]  cap_dlc;
    reg [63:0] cap_data;
    reg [14:0] cap_crc;

    // Internal bit_tick recovery from the protocol engine
    wire bit_tick = u_can_top.u_protocol_engine.f1.bit_tick;

    initial begin
        captured_bits = 0;
        bit_idx       = 0;
        capturing     = 0;
    end

    // Capture CAN bitstream synchronously at each bit_tick while transmitter is active
    always @(posedge clk) begin
        if (tx_en) begin
            capturing <= 1'b1;
            if (bit_tick) begin
                captured_bits[255 - bit_idx] <= tx_can;
                bit_idx <= bit_idx + 1;
            end
        end else if (capturing) begin
            capturing <= 1'b0;
        end
    end

    // =========================================================================
    // 6. Test Environment Execution & Self-Checking Asserts
    // =========================================================================
    reg [10:0] exp_id;
    reg        exp_rtr;
    reg [3:0]  exp_dlc;
    reg [63:0] exp_data;
    reg [14:0] exp_crc;

    initial begin
        // Reset Setup
        sck  = 1'b0;
        si   = 1'b0;
        cs_n = 1'b1;
        rst_n = 1'b0;
        #500;
        rst_n = 1'b1;
        #500;

        $display("=================================================");
        $display("   STARTING END-TO-END SPI -> CAN BUS TESTBENCH  ");
        $display("=================================================");

        // Test Stimulus Configuration
        exp_id   = 11'h5A5;      // Chosen to minimize stuff bits during validation
        exp_rtr  = 1'b0;
        exp_dlc  = 4'd2;
        exp_data = 64'hA55A_0000_0000_0000;
        exp_crc  = calc_can_crc15(exp_id, exp_rtr, exp_dlc, exp_data);

        // Step 1: Write Command over SPI Interface
        $display("[SPI] Writing CAN Message over SPI interface...");
        spi_send_can_frame(exp_id, exp_rtr, exp_dlc, exp_data);

        // Step 2: Wait for Transmission Completion on the CAN bus
        wait(capturing == 1'b1);
        wait(capturing == 1'b0);
        #1000;

        // Step 3: Parse Bitstream Fields from Captured Bus
        // captured_bits layout: [SOF(1)] [ID(11)] [RTR(1)] [IDE(1)] [r0(1)] [DLC(4)] [DATA(8*DLC)] [CRC(15)]
        cap_id   = captured_bits[254 -: 11];
        cap_rtr  = captured_bits[243];
        cap_ide  = captured_bits[242];
        cap_r0   = captured_bits[241];
        cap_dlc  = captured_bits[240 -: 4];
        cap_data = captured_bits[236 -: 16] << 48; // Scaled to MSB alignment
        cap_crc  = captured_bits[(236 - (exp_dlc * 8)) -: 15];

        // Step 4: Verification Asserts
        $display("\n-------------------------------------------------");
        $display("              FRAME DECODE ANALYSIS              ");
        $display("-------------------------------------------------");
        $display("Field    | Expected    | Captured    | Status");
        $display("-------------------------------------------------");

        // Assert Identifier
        if (cap_id === exp_id) begin
            $display("ID       | 0x%03X       | 0x%03X       | PASS", exp_id, cap_id);
            pass_count = pass_count + 1;
        end else begin
            $display("ID       | 0x%03X       | 0x%03X       | FAIL", exp_id, cap_id);
            fail_count = fail_count + 1;
        end

        // Assert RTR Flag
        if (cap_rtr === exp_rtr) begin
            $display("RTR      | %b           | %b           | PASS", exp_rtr, cap_rtr);
            pass_count = pass_count + 1;
        end else begin
            $display("RTR      | %b           | %b           | FAIL", exp_rtr, cap_rtr);
            fail_count = fail_count + 1;
        end

        // Assert DLC
        if (cap_dlc === exp_dlc) begin
            $display("DLC      | 0x%01X         | 0x%01X         | PASS", exp_dlc, cap_dlc);
            pass_count = pass_count + 1;
        end else begin
            $display("DLC      | 0x%01X         | 0x%01X         | FAIL", exp_dlc, cap_dlc);
            fail_count = fail_count + 1;
        end

        // Assert Data Payload
        if (cap_data[63:48] === exp_data[63:48]) begin
            $display("DATA     | 0x%04X     | 0x%04X     | PASS", exp_data[63:48], cap_data[63:48]);
            pass_count = pass_count + 1;
        end else begin
            $display("DATA     | 0x%04X     | 0x%04X     | FAIL", exp_data[63:48], cap_data[63:48]);
            fail_count = fail_count + 1;
        end

        // Assert CRC
        if (cap_crc === exp_crc) begin
            $display("CRC-15   | 0x%04X      | 0x%04X      | PASS", exp_crc, cap_crc);
            pass_count = pass_count + 1;
        end else begin
            $display("CRC-15   | 0x%04X      | 0x%04X      | FAIL", exp_crc, cap_crc);
            fail_count = fail_count + 1;
        end

        // Final Testbench Summary
        $display("-------------------------------------------------");
        if (fail_count == 0) begin
            $display(">> TEST RESULT: ALL CHECKS PASSED (%0d/5)", pass_count);
        end else begin
            $display(">> TEST RESULT: TEST FAILED (%0d Failure(s))", fail_count);
        end
        $display("=================================================\n");

        $finish;
    end

endmodule