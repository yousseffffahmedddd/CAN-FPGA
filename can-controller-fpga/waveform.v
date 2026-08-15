`timescale 1ns / 1ps

module tb_spi_can_e2e;

    // Parameters
    parameter CLK_PERIOD = 20; // 50 MHz system clock

    // Signals
    reg clk;
    reg rst_n;

    // SPI Master emulation signals
    reg sck;
    reg cs_n;
    reg mosi;
    wire miso;

    // CAN Bus physical pins & wrapper monitoring wires
    wire can_rx;
    wire can_tx;
    wire tx_en_wire;
    wire int_n_wire;

    // Testbench capture variables
    reg [255:0] captured_bits;
    reg [9:0]   bit_idx;
    reg         capturing;
    reg         tx_en_prev;

    // Decoded fields
    reg [10:0] cap_id;
    reg        cap_rtr;
    reg [3:0]  cap_dlc;
    reg [63:0] cap_data;
    reg [14:0] cap_crc;

    // Top-level instantiation matching actual can_top ports
    can_top u_can_top (
        .clk     (clk),
        .rst_n   (rst_n),
        .sck     (sck),
        .cs_n    (cs_n),
        .si      (mosi),
        .so      (miso),
        .rx_pin  (can_rx),
        .tx_can  (can_tx),
        .tx_en   (tx_en_wire),
        .int_n   (int_n_wire)
    );

    // Simulated CAN bus loopback / termination
    assign can_rx = can_tx; 

    // Clock generation
    initial begin
        clk = 0;
        forever #(CLK_PERIOD / 2.0) clk = ~clk;
    end

    // Waveform Dump Setup for ModelSim / Questa / GTKWave
    initial begin
        $dumpfile("can_e2e_waveform.vcd");
        $dumpvars(0, tb_spi_can_e2e);
    end

    // SPI Transaction Task
    task spi_write(input [7:0] addr, input [7:0] data);
        integer i;
        begin
            @(posedge clk);
            cs_n = 0;
            // Send Write Instruction (0x02)
            for (i = 7; i >= 0; i = i - 1) begin
                sck = 0;
                mosi = (8'h02 >> i) & 1;
                #(CLK_PERIOD);
                sck = 1;
                #(CLK_PERIOD);
            end
            // Send Address
            for (i = 7; i >= 0; i = i - 1) begin
                sck = 0;
                mosi = (addr >> i) & 1;
                #(CLK_PERIOD);
                sck = 1;
                #(CLK_PERIOD);
            end
            // Send Data
            for (i = 7; i >= 0; i = i - 1) begin
                sck = 0;
                mosi = (data >> i) & 1;
                #(CLK_PERIOD);
                sck = 1;
                #(CLK_PERIOD);
            end
            sck = 0;
            cs_n = 1;
            #(CLK_PERIOD * 2);
        end
    endtask

    // Send a single-byte SPI command such as RTS.
    task spi_command(input [7:0] opcode);
        integer i;
        begin
            @(posedge clk);
            cs_n = 0;
            for (i = 7; i >= 0; i = i - 1) begin
                sck = 0;
                mosi = opcode[i];
                #(CLK_PERIOD);
                sck = 1;
                #(CLK_PERIOD);
            end
            sck = 0;
            cs_n = 1;
            #(CLK_PERIOD * 2);
        end
    endtask

    // Frame capture monitor logic (Gated by bit_tick to prevent oversampling)
    wire tb_bit_tick   = u_can_top.u_protocol_engine.bit_tick;
    wire current_tx_en = tx_en_wire;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            captured_bits <= 256'd0;
            bit_idx       <= 10'd0;
            capturing     <= 1'b0;
            tx_en_prev    <= 1'b0;
        end else begin
            tx_en_prev <= current_tx_en;

            if (current_tx_en && !tx_en_prev) begin
                capturing     <= 1'b1;
                bit_idx       <= 10'd0;
                captured_bits <= 256'd0;
                $display("[DEBUG] TX Started at time %0t", $time);
            end

            // Capture exactly once per CAN bit using bit_tick
            if (current_tx_en && capturing && tb_bit_tick) begin
                if (bit_idx < 256) begin
                    captured_bits[255 - bit_idx] <= can_tx;
                    bit_idx <= bit_idx + 1'b1;
                end
            end

            if (!current_tx_en && tx_en_prev && capturing) begin
                capturing <= 1'b0;
                $display("[DEBUG] TX Ended at time %0t, total bits: %0d", $time, bit_idx);
            end
        end
    end

    // Main Test Stimulus & Analysis
    integer failures;
    initial begin
        
        failures = 0;

        // Initialize signals
        rst_n = 0;
        sck   = 0;
        cs_n  = 1;
        mosi  = 0;

        #(CLK_PERIOD * 10);
        rst_n = 1;
        #(CLK_PERIOD * 10);

        $display("=================================================");
        $display("   STARTING END-TO-END SPI -> CAN WAVEFORM TEST  ");
        $display("=================================================");

        // [INIT] Initialize CAN Bit Timing Registers (CNF1, CNF2, CNF3)
        $display("[INIT] Initializing CAN Controller Bit Timing...");
        spi_write(8'h2a, 8'h03); // CNF3
        spi_write(8'h29, 8'hb8); // CNF2
        spi_write(8'h28, 8'h05); // CNF1

        // [INIT] Enter Normal Mode via CANCTRL
        $display("[INIT] Writing CANCTRL = 0x00 to enter Normal Mode");
        spi_write(8'h0f, 8'h00);
        #(CLK_PERIOD * 20);

        // [TEST] Write Frame to TX Buffer via SPI
        $display("[TEST] CAN Frame Config: ID=0x5a5, RTR=0, DLC=2, Data=0xa55a");
        $display("[SPI] Writing CAN Message over SPI interface...");
        
        spi_write(8'h31, 8'hB4); // TXB0SIDH for standard ID 0x5A5
        spi_write(8'h32, 8'ha0); // TXB0SIDL
        spi_write(8'h35, 8'h02); // TXB0DLC (DLC = 2)
        spi_write(8'h36, 8'ha5); // TXB0D0
        spi_write(8'h37, 8'h5a); // TXB0D1

        $display("[TX_FRAME] TX buffer loaded, now sending RTS command...");
        spi_command(8'h81); // RTS TXB0 opcode
        $display("[TX_FRAME] RTS command sent");

        // Wait for transmission to complete
        #(CLK_PERIOD * 80000);

        // Extract fields from captured bitstream
        cap_id   = captured_bits[254 -: 11];
        cap_rtr  = captured_bits[243];
        cap_dlc  = captured_bits[240 -: 4];
        cap_data = captured_bits[236 -: 16]; // 2 bytes for DLC=2
        cap_crc  = captured_bits[220 -: 15];

        $display("-------------------------------------------------");
        $display("                FRAME DECODE ANALYSIS            ");
        $display("-------------------------------------------------");
        $display("Field    | Expected    | Captured    | Status");
        $display("-------------------------------------------------");
        $display("ID       | 0x5a5       | 0x%03h       | %s", cap_id, (cap_id == 11'h5a5) ? "PASS" : "FAIL");
        $display("RTR      | 0           | %0d           | %s", cap_rtr, (cap_rtr == 1'b0) ? "PASS" : "FAIL");
        $display("DLC      | 0x2         | 0x%01h           | %s", cap_dlc, (cap_dlc == 4'h2) ? "PASS" : "FAIL");
        $display("DATA     | 0xa55a      | 0x%04h      | %s", cap_data[15:0], (cap_data[15:0] == 16'ha55a) ? "PASS" : "FAIL");
        $display("CRC-15   | 0x0000      | 0x%04h      | %s", cap_crc, (cap_crc == 15'h0000) ? "PASS" : "FAIL");
        $display("-------------------------------------------------");

        if (cap_id != 11'h5a5) failures = failures + 1;
        if (cap_rtr != 1'b0)  failures = failures + 1;
        if (cap_dlc != 4'h2)  failures = failures + 1;
        if (cap_data[15:0] != 16'ha55a) failures = failures + 1;

        if (failures == 0)
            $display(">> TEST RESULT: TEST PASSED Successfully!");
        else
            $display(">> TEST RESULT: TEST FAILED (%0d Failure(s))", failures);
            
        $display("=================================================");
        
        $finish;
    end

endmodule