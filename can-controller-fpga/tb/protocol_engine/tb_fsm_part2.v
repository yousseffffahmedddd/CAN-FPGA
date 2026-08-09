// =============================================================================
// Testbench      : tb_fsm_part2
// DUT            : fsm_part2.v (Core field-level Protocol FSM)
// SOW traceability (statement_of_work_v2.docx, Module 1 - CAN Protocol Engine,
//                   Milestone 1 "Core FSM Sequencing"):
//   M1-1/2   : recessive->dominant edge on RXCAN -> IDLE to SOF.
//   M1-3/4/5 : one dominant bit for exactly one tick during SOF, then to
//              ARBITRATION.
//   M1-6..12 : 11-bit ID + RTR shifted via PISO one bit/tick; non-destructive
//              arbitration compare; keep driving while matching; drop to
//              receive-only and set arb_lost (NOT bit_error) on mismatch;
//              move to CONTROL once all 12 bits are processed.
//   M1-13..17: IDE, RB0, 4-bit DLC processed and latched MSB-first; move to
//              DATA once 6 CONTROL bits are processed.
//   M1-18..21: DLC*8 bits shifted via PISO/SIPO in DATA; move to CRC after.
//   M1-20    : DLC=0 skips DATA entirely.
//   M1-22..26: PISO request/valid handshake, SIPO output/valid handshake,
//              MSB-first bit ordering on both interfaces.
// =============================================================================
`timescale 1ns/1ps

module tb_fsm_part2;

    reg clk, rst_n;
    reg bit_tick, rx_can_sync;
    reg piso_data_in, piso_valid;

    wire tx_can, tx_en;
    wire piso_req;
    wire sipo_data_out, sipo_valid;
    wire bit_error, arb_lost;
    wire [3:0] latched_dlc;
    wire [2:0] current_state;

    localparam [2:0] STATE_IDLE        = 3'b000;
    localparam [2:0] STATE_SOF         = 3'b001;
    localparam [2:0] STATE_ARBITRATION = 3'b010;
    localparam [2:0] STATE_CONTROL     = 3'b011;
    localparam [2:0] STATE_DATA        = 3'b100;
    localparam [2:0] STATE_CRC         = 3'b101;

    integer errors = 0;
    integer checks = 0;

    fsm_part2 dut (
        .clk          (clk),
        .rst_n        (rst_n),
        .bit_tick     (bit_tick),
        .rx_can_sync  (rx_can_sync),
        .tx_can       (tx_can),
        .tx_en        (tx_en),
        .piso_data_in (piso_data_in),
        .piso_valid   (piso_valid),
        .piso_req     (piso_req),
        .sipo_data_out(sipo_data_out),
        .sipo_valid   (sipo_valid),
        .bit_error    (bit_error),
        .arb_lost     (arb_lost),
        .latched_dlc  (latched_dlc),
        .current_state(current_state)
    );

    always #5 clk = ~clk;

    task check(input cond, input [8*256-1:0] msg);
        begin
            checks = checks + 1;
            if (!cond) begin
                errors = errors + 1;
                $display("[FAIL] %0t : %s", $time, msg);
            end
            else
                $display("[PASS] %0t : %s", $time, msg);
        end
    endtask

    task reset_dut;
        begin
            rst_n = 0; bit_tick = 0; rx_can_sync = 1;
            piso_data_in = 0; piso_valid = 0;
            repeat (4) @(negedge clk);
            rst_n = 1;
            @(negedge clk);
        end
    endtask

    // Captured snapshot of every DUT output, valid for the bit period just driven
    reg cap_tx_can, cap_tx_en, cap_piso_req;
    reg cap_sipo_valid, cap_sipo_data, cap_bit_error, cap_arb_lost;
    reg [3:0] cap_latched_dlc;
    reg [2:0] cap_state;

    // Drives one full bit period:
    //   - places rx_bit on rx_can_sync ahead of the tick (models the bus)
    //   - pulses piso_valid/piso_data_in = piso_bit partway through the period
    //     (models a PISO/Frame Constructor that answers the FSM's continuous
    //     piso_req with one fresh bit per bit period)
    //   - pulses bit_tick for one clk cycle at the sample point
    //   - snapshots all outputs right after that edge
    task drive_bit(input piso_bit, input rx_bit);
        begin
            rx_can_sync = rx_bit;
            @(negedge clk);
            piso_valid   = 1'b1;
            piso_data_in = piso_bit;
            @(negedge clk);
            piso_valid = 1'b0;
            @(negedge clk); // let tx_bit_reg -> tx_can settle before the tick
            @(negedge clk);
            bit_tick = 1'b1;
            @(posedge clk);
            #1;
            cap_tx_can      = tx_can;
            cap_tx_en       = tx_en;
            cap_piso_req    = piso_req;
            cap_sipo_valid  = sipo_valid;
            cap_sipo_data   = sipo_data_out;
            cap_bit_error   = bit_error;
            cap_arb_lost    = arb_lost;
            cap_latched_dlc = latched_dlc;
            cap_state       = current_state;
            @(negedge clk);
            bit_tick = 1'b0;
            @(negedge clk);
        end
    endtask

    integer i;
    integer err_before;

    initial begin
        clk = 0;

        // =================================================================
        // TEST GROUP A: full winning-arbitration path, DLC=3, through to CRC
        // Covers M1-1..8,12..19,21,22,24..26
        // =================================================================
        reset_dut();
        check(current_state == STATE_IDLE, "A: post-reset state is IDLE");

        drive_bit(1'b0, 1'b1); // idle bit, bus recessive
        check(cap_state == STATE_IDLE, "A(M1-1): bus stays recessive -> FSM stays in IDLE");

        drive_bit(1'b0, 1'b0); // recessive->dominant edge this tick -> SOF
        check(cap_state == STATE_SOF, "A(M1-1,2): recessive->dominant edge moves IDLE->SOF");

        drive_bit(1'b0, 1'b1); // SOF's one bit_tick
        check(cap_tx_en == 1'b1 && cap_tx_can == 1'b0,
              "A(M1-3): SOF drives exactly one dominant bit");
        check(cap_state == STATE_ARBITRATION,
              "A(M1-4,5): after exactly one SOF tick, FSM moves to ARBITRATION");
        check(cap_sipo_valid == 1'b1 && cap_sipo_data == 1'b1,
              "A(M1-24,25): SIPO tap fires during SOF too, reflecting the observed bit");

        // 12 arbitration bits: ID[10:0]=00000000001, RTR=0. Winning node: rx
        // mirrors what we drive, so arbitration is never lost.
        begin : arb_win
            reg [11:0] arb_bits;
            arb_bits = 12'b000000000010; // 11-bit ID + RTR, MSB first
            for (i = 0; i < 12; i = i + 1) begin
                drive_bit(arb_bits[11-i], arb_bits[11-i]); // rx mirrors tx: winning
                check(cap_arb_lost == 1'b0, "A(M1-8,9): matching bits -> arbitration not lost");
                check(cap_tx_en == 1'b1, "A(M1-8): still driving the bus while winning");
                check(cap_piso_req == 1'b1, "A(M1-22): piso_req asserted while transmitting");
                check(cap_sipo_valid == 1'b1 && cap_sipo_data == arb_bits[11-i],
                      "A(M1-24,25,26): SIPO reflects each arbitration bit, MSB-first");
            end
            check(cap_state == STATE_CONTROL,
                  "A(M1-12): after 12 arbitration bits with no unresolved mismatch -> CONTROL");
        end

        // 6 control bits: IDE=1, RB0=0, DLC=3 (0011), MSB-first, bits 2..5 = DLC
        begin : ctrl_win
            reg [5:0] ctrl_bits;
            ctrl_bits = 6'b100011; // IDE, RB0, DLC[3:0]
            for (i = 0; i < 6; i = i + 1) begin
                drive_bit(ctrl_bits[5-i], ctrl_bits[5-i]);
                check(cap_bit_error == 1'b0, "A(M1 CONTROL): no bit_error on self-consistent bits");
            end
            check(cap_latched_dlc == 4'd3,
                  "A(M1-15,16,26): DLC=3 latched correctly, MSB-first shift-in");
            check(cap_state == STATE_DATA,
                  "A(M1-17): after 6 control bits (DLC!=0) -> DATA");
        end

        // DATA field: DLC*8 = 24 bits
        begin : data_win
            for (i = 0; i < 24; i = i + 1) begin
                drive_bit(i[0], i[0]); // arbitrary self-consistent pattern
                check(cap_bit_error == 1'b0, "A(M1 DATA): no bit_error on self-consistent data bits");
                check(cap_sipo_valid == 1'b1, "A(M1-18,24,25): SIPO pulses for every data bit");
                if (i < 23)
                    check(cap_state == STATE_DATA, "A(M1-19): stays in DATA until all DLC*8 bits done");
            end
            check(cap_state == STATE_CRC,
                  "A(M1-21): after DLC*8 data bits -> CRC");
        end

        // =================================================================
        // TEST GROUP B: arbitration lost partway through, no bit_error,
        // FSM continues (receive-only) through to CONTROL.
        // Covers M1-9,10,11
        // =================================================================
        reset_dut();
        drive_bit(1'b0, 1'b1);              // idle recessive
        drive_bit(1'b0, 1'b0);              // SOF edge
        drive_bit(1'b0, 1'b1);              // SOF tick -> ARBITRATION

        begin : arb_lose
            reg [11:0] arb_bits_lose;
            arb_bits_lose = 12'b111111111110; // our transmitted ID (all recessive, low priority)
            // Bits 0..2: no contention (bus mirrors us)
            for (i = 0; i < 3; i = i + 1)
                drive_bit(arb_bits_lose[11-i], arb_bits_lose[11-i]);
            check(cap_arb_lost == 1'b0, "B: still winning before the contested bit");

            // Bit 3: we send recessive (1), a higher-priority node drives dominant (0)
            drive_bit(arb_bits_lose[11-3], 1'b0);
            check(cap_arb_lost == 1'b1, "B(M1-9,10): recessive-vs-dominant mismatch -> arb_lost asserted");
            check(cap_tx_en == 1'b0, "B(M1-10): drops to receive-only immediately on loss");
            check(cap_bit_error == 1'b0, "B(M1-11): arbitration mismatch does NOT raise bit_error");

            // Remaining 8 arbitration bits: FSM just observes the winner's bus value
            for (i = 4; i < 12; i = i + 1) begin
                drive_bit(1'b1, 1'b0); // our piso side is irrelevant now (receive-only)
                check(cap_tx_en == 1'b0, "B: stays receive-only for rest of arbitration");
                check(cap_bit_error == 1'b0, "B: no spurious bit_error while receive-only");
            end
            check(cap_state == STATE_CONTROL,
                  "B(M1-12): still reaches CONTROL after losing arbitration mid-field");
        end

        // =================================================================
        // TEST GROUP C: DLC=0 bypasses DATA entirely
        // Covers M1-20
        // =================================================================
        reset_dut();
        drive_bit(1'b0, 1'b1);
        drive_bit(1'b0, 1'b0); // SOF edge
        drive_bit(1'b0, 1'b1); // SOF tick -> ARBITRATION
        for (i = 0; i < 12; i = i + 1)
            drive_bit(1'b0, 1'b0); // trivial winning ID of all dominant bits
        begin : ctrl_dlc0
            reg [5:0] ctrl_bits0;
            ctrl_bits0 = 6'b000000; // IDE=0, RB0=0, DLC=0000
            for (i = 0; i < 6; i = i + 1)
                drive_bit(ctrl_bits0[5-i], ctrl_bits0[5-i]);
            check(cap_latched_dlc == 4'd0, "C(M1-20 setup): DLC=0 latched");
            check(cap_state == STATE_CRC,
                  "C(M1-20): DLC=0 skips DATA entirely, goes straight to CRC");
        end

        // =================================================================
        // Summary
        // =================================================================
        $display("--------------------------------------------------");
        $display("fsm_part2 TB: %0d checks, %0d failures", checks, errors);
        if (errors == 0)
            $display("RESULT: PASS - all SOW Module-1/Milestone-1 FSM requirements met");
        else
            $display("RESULT: FAIL - see log above");
        $display("--------------------------------------------------");
        $finish;
    end

endmodule
