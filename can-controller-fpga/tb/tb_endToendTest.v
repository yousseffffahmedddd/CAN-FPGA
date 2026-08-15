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
    integer sim_timeout = 0;
    parameter TIMEOUT_CYCLES = 20000; // ~20ms at 1MHz

    // System Clock Generation (1 MHz System Clock = 1us period, 500ns half-period)
    initial clk = 0;
    always #500 clk = ~clk;

    // Timeout watchdog
    always @(posedge clk) begin
        sim_timeout <= sim_timeout + 1;
    end

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

    // Internal probes for debugging
    wire rts_pulse_dbg = u_can_top.u_spi_if.rts_pulse;
    wire [7:0] we_addr_dbg = u_can_top.u_spi_if.addr;
    wire we_dbg = u_can_top.u_spi_if.we;
    wire [7:0] we_data_dbg = u_can_top.u_spi_if.wdata;

    // Control / protocol internals
    wire [2:0] ctrl_mode_dbg = u_can_top.u_reg_bank.opmod;
    wire ctrl_txreq_dbg = u_can_top.u_reg_bank.txb0_txreq;
    wire [2:0] proto_state_dbg = u_can_top.u_protocol_engine.current_state;

    // --- FIX: bit-level tick probe, needed to gate bitstream capture ---
    // Without this, the capture block below samples every SYSTEM clock
    // instead of every CAN BIT, which is the root cause of the original
    // failure (captured_bits filled with oversampled SOF, ID/DLC/Data
    // all read back as zero).
    wire proto_bit_tick_dbg = u_can_top.u_protocol_engine.bit_tick;

    // =========================================================================
    // 3. SPI Bit-Bang Tasks & OPCODES
    // =========================================================================
    localparam CMD_LOAD_TX0  = 8'h40; // Load TX Buffer 0
    localparam CMD_RTS_TX0   = 8'h81; // Request-To-Send TX Buffer 0
    localparam CMD_WRITE     = 8'h02; // Write register
    localparam CMD_READ      = 8'h03; // Read register
    localparam CMD_RESET     = 8'hC0; // Soft reset

    // Register addresses
    localparam CANCTRL       = 8'h0F; // Control register
    localparam CANSTAT       = 8'h0E; // Status register
    localparam TXB0SIDH      = 8'h31; // TX Buffer 0 ID High
    localparam TXB0SIDL      = 8'h32; // TX Buffer 0 ID Low
    localparam TXB0DLC       = 8'h35; // TX Buffer 0 DLC
    localparam TXB0D0        = 8'h36; // TX Buffer 0 Data

    task spi_write_byte(input [7:0] data);
        integer i;
        begin
            sck = 1'b0;
            for (i = 7; i >= 0; i = i - 1) begin
                si = data[i];
                #500;              // MOSI setup before SCK rising edge
                sck = 1'b1;
                #2000;             // allow the 2-FF synchronizer to see the edge
                sck = 1'b0;
                #500;              // hold low before the next bit
            end
            si = 1'b0;
            #200;                 // keep CS low until the byte is fully received
        end
    endtask

    // Task to write a register via SPI
    task spi_write_register(input [7:0] address, input [7:0] data);
        begin
            $display("[SPI_WRITE] Addr=0x%02X, Data=0x%02X", address, data);
            cs_n = 1'b0;
            #200;
            spi_write_byte(CMD_WRITE);
            spi_write_byte(address);
            spi_write_byte(data);
            #200;
            cs_n = 1'b1;
            sck = 1'b0;
            #2000;
            $display("[SPI_WRITE] Complete");
        end
    endtask

    // Task to read a register via SPI
    task spi_read_register(input [7:0] address, output [7:0] data);
        integer i;
        reg bit_val;
        begin
            $display("[SPI_READ] Addr=0x%02X", address);
            data = 8'h00;
            cs_n = 1'b0;
            #100;
            spi_write_byte(CMD_READ);       // Read command (0x03)
            spi_write_byte(address);        // Register address

            // Read 8 bits - capture on SCK rising edge
            for (i = 7; i >= 0; i = i - 1) begin
                #100;
                sck = 1'b1;
                #50;  // Wait for output to settle
                bit_val = so;
                data[i] = bit_val;
                #50;
                sck = 1'b0;
            end

            #100;
            cs_n = 1'b1;
            #500;
            $display("[SPI_READ] Read value = 0x%02X", data);
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
            $display("[TX_FRAME] Loading TX buffer with ID=0x%03X, DLC=%0d", id, dlc);

            cs_n = 1'b0;
            #200;
            spi_write_byte(CMD_LOAD_TX0);
            spi_write_byte({id[10:3]});
spi_write_byte({id[2:0], rtr, 4'b0000}); // Matches the reg_bank decoding: ID[2:0] at [7:5], RTR at [4]           
 spi_write_byte({4'b0000, dlc});

            for (b = 0; b < dlc; b = b + 1) begin
                spi_write_byte(data[63 - (b*8) -: 8]);
            end

            #200;
            cs_n = 1'b1;
            sck = 1'b0;
            #2000;

            $display("[TX_FRAME] TX buffer loaded, now sending RTS command...");

            cs_n = 1'b0;
            #200;
            spi_write_byte(CMD_RTS_TX0);
            #200;
            cs_n = 1'b1;
            sck = 1'b0;
            #2000;

            $display("[TX_FRAME] RTS command sent");
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
    reg         tx_en_prev;
    integer     tx_en_start_time;
    integer     tx_en_end_time;

    initial begin
        captured_bits = 0;
        bit_idx       = 0;
        capturing     = 0;
        tx_en_prev    = 0;
        tx_en_start_time = 0;
        tx_en_end_time = 0;
    end

    // Capture CAN bitstream synchronously, ONE SAMPLE PER CAN BIT (gated by
    // proto_bit_tick_dbg), while the transmitter is active. Previously this
    // sampled every system clock edge, which is wrong: fsm_part2 only
    // updates tx_can once per bit_tick, so sampling on every system clock
    // just captured dozens/hundreds of repeats of whatever bit was currently
    // being held (mostly the SOF dominant bit), corrupting every downstream
    // field decode.
    always @(posedge clk) begin
        tx_en_prev <= tx_en;

        // Detect TX start
        if (tx_en && !tx_en_prev) begin
            capturing <= 1'b1;
            bit_idx <= 0;
            tx_en_start_time <= sim_timeout;
            $display("[DEBUG] TX Started at time %0t (cycle %0d)", $time, sim_timeout);
        end

        // Capture exactly once per real CAN bit, not every system clock
        if (tx_en && capturing && proto_bit_tick_dbg) begin
            captured_bits[255 - bit_idx] <= tx_can;
            bit_idx <= bit_idx + 1;
        end

        // Detect TX end
        if (!tx_en && tx_en_prev && capturing) begin
            capturing <= 1'b0;
            tx_en_end_time <= sim_timeout;
            $display("[DEBUG] TX Ended at time %0t (cycle %0d), total bits: %0d",
                     $time, sim_timeout, bit_idx);
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

    // Field Decoding Variables
    reg [10:0] cap_id;
    reg        cap_rtr;
    reg [3:0]  cap_dlc;
    reg [63:0] cap_data;
    reg [14:0] cap_crc;

    // Temporary variables for tasks
    reg [7:0] status_byte;

    initial begin
        // Reset Setup
        sck  = 1'b0;
        si   = 1'b0;
        cs_n = 1'b1;
        rst_n = 1'b0;
        sim_timeout = 0;
        #500;
        rst_n = 1'b1;
        #5000;

        $display("=================================================");
        $display("   STARTING END-TO-END SPI -> CAN BUS TESTBENCH  ");
        $display("=================================================");

        // =====================================================
        // INITIALIZATION: Configure Bit Timing & Enter Normal Mode
        // =====================================================
        $display("\n[INIT] Initializing CAN Controller Bit Timing...");
        spi_write_register(8'h2A, 8'h03); // CNF1: SJW=1, BRP=3
        spi_write_register(8'h29, 8'hB8); // CNF2: BTLMODE=1, PHSEG1=3, PRSEG=8
        spi_write_register(8'h28, 8'h05); // CNF3: PHSEG2=5

        $display("[INIT] Writing CANCTRL = 0x00 to enter Normal Mode");
        spi_write_register(CANCTRL, 8'h00);
        #10000;  // Wait for mode transition
        $display("[INIT] Waiting complete, proceeding to transmission test\n");

        // =====================================================
        // TEST: Transmit CAN Frame
        // =====================================================
        // Test Stimulus Configuration
        exp_id   = 11'h5A5;
        exp_rtr  = 1'b0;
        exp_dlc  = 4'd2;
        exp_data = 64'hA55A_0000_0000_0000;
        exp_crc  = calc_can_crc15(exp_id, exp_rtr, exp_dlc, exp_data);

        $display("[TEST] CAN Frame Config: ID=0x%03X, RTR=%b, DLC=%0d, Data=0x%016X",
                 exp_id, exp_rtr, exp_dlc, exp_data);

        // Step 1: Write Command over SPI Interface
        $display("[SPI] Writing CAN Message over SPI interface...");
        spi_send_can_frame(exp_id, exp_rtr, exp_dlc, exp_data);

        // DEBUG: Monitor internal signals
        #1000;
        $display("[DEBUG] After SPI Write:");
        $display("  tx_en = %b", tx_en);
        $display("  so = %b", so);
        $display("  int_n = %b", int_n);
        $display("[DEBUG] Internal Controller State:");
        $display("  RTS Pulse = %b", rts_pulse_dbg);
        $display("  Last SPI Addr = 0x%02X", we_addr_dbg);
        $display("  Last SPI WE = %b, Data = 0x%02X", we_dbg, we_data_dbg);
        $display("  Controller Mode (OPMOD) = 0x%02X", ctrl_mode_dbg);
        $display("  TX Request Flag = %b", ctrl_txreq_dbg);
        $display("  Protocol Engine State = %b", proto_state_dbg);

        // Step 2: Wait for Transmission Start (with timeout)
        $display("[WAIT] Waiting for TX to start (max %0d cycles)...", TIMEOUT_CYCLES);
        sim_timeout = 0;
        wait(tx_en == 1'b1);
        if (sim_timeout > TIMEOUT_CYCLES) begin
            $display("[ERROR] TIMEOUT: tx_en never asserted after %0d cycles", sim_timeout);
            fail_count = fail_count + 1;
            $finish;
        end
        $display("[OK] TX Started at cycle %0d", sim_timeout);

        // Step 3: Wait for Transmission End (with timeout)
        $display("[WAIT] Waiting for TX to complete...");
        sim_timeout = 0;
        wait(tx_en == 1'b0);
        if (sim_timeout > TIMEOUT_CYCLES) begin
            $display("[ERROR] TIMEOUT: tx_en never deasserted after %0d cycles", sim_timeout);
            fail_count = fail_count + 1;
            $finish;
        end
        $display("[OK] TX Ended at cycle %0d, captured %0d bits", sim_timeout, bit_idx);

        // Wait for capture to finalize
        #2000;

        // Step 4: Verify captured data
        if (bit_idx == 0) begin
            $display("[ERROR] No bits captured! Check tx_en signal.");
            fail_count = fail_count + 1;
            $finish;
        end

        // Step 5: Parse Bitstream Fields from Captured Bus
        cap_id   = captured_bits[254 -: 11];
        cap_rtr  = captured_bits[243];
        cap_dlc  = captured_bits[240 -: 4];
        cap_data = captured_bits[236 -: 16] << 48;
        cap_crc  = captured_bits[(236 - (exp_dlc * 8)) -: 15];

        $display("[DEBUG] Captured Frame - ID: 0x%03X, RTR: %b, DLC: 0x%01X, Data: 0x%016X",
                 cap_id, cap_rtr, cap_dlc, cap_data);

        // Step 6: Verification Asserts
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

// `timescale 1ns / 1ps

// module tb_spi_can_e2e;

//     // Parameters
//     parameter CLK_PERIOD = 20; // 50 MHz system clock

//     // Signals
//     reg clk;
//     reg rst_n;

//     // SPI Master emulation signals
//     reg sck;
//     reg cs_n;
//     reg mosi;
//     wire miso;

//     // CAN Bus physical pins & wrapper monitoring wires
//     wire can_rx;
//     wire can_tx;
//     wire tx_en_wire;
//     wire int_n_wire;

//     // Testbench capture variables
//     reg [255:0] captured_bits;
//     reg [9:0]   bit_idx;
//     reg         capturing;
//     reg         tx_en_prev;

//     // Decoded fields
//     reg [10:0] cap_id;
//     reg        cap_rtr;
//     reg [3:0]  cap_dlc;
//     reg [63:0] cap_data;
//     reg [14:0] cap_crc;

//     // Top-level instantiation matching actual can_top ports
//     can_top u_can_top (
//         .clk     (clk),
//         .rst_n   (rst_n),
//         .sck     (sck),
//         .cs_n    (cs_n),
//         .si      (mosi),
//         .so      (miso),
//         .rx_pin  (can_rx),
//         .tx_can  (can_tx),
//         .tx_en   (tx_en_wire),
//         .int_n   (int_n_wire)
//     );

//     // Simulated CAN bus loopback / termination
//     assign can_rx = can_tx; 

//     // Clock generation
//     initial begin
//         clk = 0;
//         forever #(CLK_PERIOD / 2.0) clk = ~clk;
//     end

//     // Waveform Dump Setup for ModelSim / Questa / GTKWave
//     initial begin
//         $dumpfile("can_e2e_waveform.vcd");
//         $dumpvars(0, tb_spi_can_e2e);
//     end

//     // SPI Transaction Task
//     task spi_write(input [7:0] addr, input [7:0] data);
//         integer i;
//         begin
//             @(posedge clk);
//             cs_n = 0;
//             // Send Write Instruction (0x02)
//             for (i = 7; i >= 0; i = i - 1) begin
//                 sck = 0;
//                 mosi = (8'h02 >> i) & 1;
//                 #(CLK_PERIOD);
//                 sck = 1;
//                 #(CLK_PERIOD);
//             end
//             // Send Address
//             for (i = 7; i >= 0; i = i - 1) begin
//                 sck = 0;
//                 mosi = (addr >> i) & 1;
//                 #(CLK_PERIOD);
//                 sck = 1;
//                 #(CLK_PERIOD);
//             end
//             // Send Data
//             for (i = 7; i >= 0; i = i - 1) begin
//                 sck = 0;
//                 mosi = (data >> i) & 1;
//                 #(CLK_PERIOD);
//                 sck = 1;
//                 #(CLK_PERIOD);
//             end
//             sck = 0;
//             cs_n = 1;
//             #(CLK_PERIOD * 2);
//         end
//     endtask

//     // Frame capture monitor logic (Gated by bit_tick to prevent oversampling)
//     wire tb_bit_tick   = u_can_top.u_protocol_engine.bit_tick;
//     wire current_tx_en = tx_en_wire;

//     always @(posedge clk or negedge rst_n) begin
//         if (!rst_n) begin
//             captured_bits <= 256'd0;
//             bit_idx       <= 10'd0;
//             capturing     <= 1'b0;
//             tx_en_prev    <= 1'b0;
//         end else begin
//             tx_en_prev <= current_tx_en;

//             if (current_tx_en && !tx_en_prev) begin
//                 capturing     <= 1'b1;
//                 bit_idx       <= 10'd0;
//                 captured_bits <= 256'd0;
//                 $display("[DEBUG] TX Started at time %0t", $time);
//             end

//             // Capture exactly once per CAN bit using bit_tick
//             if (current_tx_en && capturing && tb_bit_tick) begin
//                 if (bit_idx < 256) begin
//                     captured_bits[255 - bit_idx] <= can_tx;
//                     bit_idx <= bit_idx + 1'b1;
//                 end
//             end

//             if (!current_tx_en && tx_en_prev && capturing) begin
//                 capturing <= 1'b0;
//                 $display("[DEBUG] TX Ended at time %0t, total bits: %0d", $time, bit_idx);
//             end
//         end
//     end

//     // Main Test Stimulus & Analysis
//     integer failures;
//     initial begin
        
//         failures = 0;

//         // Initialize signals
//         rst_n = 0;
//         sck   = 0;
//         cs_n  = 1;
//         mosi  = 0;

//         #(CLK_PERIOD * 10);
//         rst_n = 1;
//         #(CLK_PERIOD * 10);

//         $display("=================================================");
//         $display("   STARTING END-TO-END SPI -> CAN WAVEFORM TEST  ");
//         $display("=================================================");

//         // [INIT] Initialize CAN Bit Timing Registers (CNF1, CNF2, CNF3)
//         $display("[INIT] Initializing CAN Controller Bit Timing...");
//         spi_write(8'h2a, 8'h03); // CNF3
//         spi_write(8'h29, 8'hb8); // CNF2
//         spi_write(8'h28, 8'h05); // CNF1

//         // [INIT] Enter Normal Mode via CANCTRL
//         $display("[INIT] Writing CANCTRL = 0x00 to enter Normal Mode");
//         spi_write(8'h0f, 8'h00);
//         #(CLK_PERIOD * 20);

//         // [TEST] Write Frame to TX Buffer via SPI
//         $display("[TEST] CAN Frame Config: ID=0x5a5, RTR=0, DLC=2, Data=0xa55a");
//         $display("[SPI] Writing CAN Message over SPI interface...");
        
//         spi_write(8'h31, 8'h5a); // TXB0SIDH
//         spi_write(8'h32, 8'ha0); // TXB0SIDL
//         spi_write(8'h35, 8'h02); // TXB0DLC (DLC = 2)
//         spi_write(8'h36, 8'ha5); // TXB0D0
//         spi_write(8'h37, 8'h5a); // TXB0D1

//         $display("[TX_FRAME] TX buffer loaded, now sending RTS command...");
//         spi_write(8'h00, 8'h01); // RTS command
//         $display("[TX_FRAME] RTS command sent");

//         // Wait for transmission to complete
//         #(CLK_PERIOD * 80000);

//         // Extract fields from captured bitstream
//         cap_id   = captured_bits[254 -: 11];
//         cap_rtr  = captured_bits[243];
//         cap_dlc  = captured_bits[240 -: 4];
//         cap_data = captured_bits[236 -: 16]; // 2 bytes for DLC=2
//         cap_crc  = captured_bits[220 -: 15];

//         $display("-------------------------------------------------");
//         $display("                FRAME DECODE ANALYSIS            ");
//         $display("-------------------------------------------------");
//         $display("Field    | Expected    | Captured    | Status");
//         $display("-------------------------------------------------");
//         $display("ID       | 0x5a5       | 0x%03h       | %s", cap_id, (cap_id == 11'h5a5) ? "PASS" : "FAIL");
//         $display("RTR      | 0           | %0d           | %s", cap_rtr, (cap_rtr == 1'b0) ? "PASS" : "FAIL");
//         $display("DLC      | 0x2         | 0x%01h           | %s", cap_dlc, (cap_dlc == 4'h2) ? "PASS" : "FAIL");
//         $display("DATA     | 0xa55a      | 0x%04h      | %s", cap_data[15:0], (cap_data[15:0] == 16'ha55a) ? "PASS" : "FAIL");
//         $display("CRC-15   | 0x0000      | 0x%04h      | %s", cap_crc, (cap_crc == 15'h0000) ? "PASS" : "FAIL");
//         $display("-------------------------------------------------");

//         if (cap_id != 11'h5a5) failures = failures + 1;
//         if (cap_rtr != 1'b0)  failures = failures + 1;
//         if (cap_dlc != 4'h2)  failures = failures + 1;
//         if (cap_data[15:0] != 16'ha55a) failures = failures + 1;

//         if (failures == 0)
//             $display(">> TEST RESULT: TEST PASSED Successfully!");
//         else
//             $display(">> TEST RESULT: TEST FAILED (%0d Failure(s))", failures);
            
//         $display("=================================================");
        
//         $finish;
//     end

// endmodule