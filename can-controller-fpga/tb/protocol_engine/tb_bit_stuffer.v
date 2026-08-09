// =============================================================================
// Testbench      : tb_bit_stuffer
// DUT            : bit_stuffer.v
// SOW traceability (statement_of_work_v2.docx, Module 1 - CAN Protocol Engine,
//                   Milestone 2 "CRC, Stuffing, ACK/EOF, Error Handling"):
//   Req M2-7  : Monitor outgoing bit stream for 5 consecutive identical bits.
//   Req M2-8  : Insert 1 opposite-polarity bit immediately after a run of 5.
//   Req M2-9  : Apply stuffing only between SOF and the CRC delimiter
//               (modeled here via stuffing_en).
//   Req M2-10 : Remove inserted stuff bits from the incoming bit stream (RX).
//   Req M2-11 : Flag a stuff error if 6 consecutive identical bits appear
//               where a stuff bit was expected.
// Each test below is tagged with the requirement(s) it verifies.
// =============================================================================
`timescale 1ns/1ps

module tb_bit_stuffer;

    reg clk, rst_n;
    reg bit_tick, rx_can_sync, stuffing_en;

    wire data_bit_tick, data_bit, stuff_error;
    wire insert_stuff, stuff_tx_bit;

    integer errors = 0;
    integer checks = 0;

    // Captured pulse-output snapshot, valid for the bit period just driven
    reg cap_data_bit_tick, cap_data_bit, cap_stuff_error;
    reg cap_insert_stuff, cap_stuff_tx_bit;

    bit_stuffer dut (
        .clk          (clk),
        .rst_n        (rst_n),
        .bit_tick     (bit_tick),
        .rx_can_sync  (rx_can_sync),
        .stuffing_en  (stuffing_en),
        .data_bit_tick(data_bit_tick),
        .data_bit     (data_bit),
        .stuff_error  (stuff_error),
        .insert_stuff (insert_stuff),
        .stuff_tx_bit (stuff_tx_bit)
    );

    // 10 ns clock period
    always #5 clk = ~clk;

    // Drives one bit period: places bit value on rx_can_sync, pulses bit_tick
    // for one clk cycle, then snapshots the DUT's pulse outputs right after
    // the posedge that samples the tick (before they decay back low on the
    // next posedge), so callers can check them immediately afterward.
    task drive_bit(input bit_val);
        begin
            rx_can_sync = bit_val;
            @(negedge clk);
            bit_tick = 1'b1;
            @(posedge clk);
            #1; // let non-blocking updates settle
            cap_data_bit_tick = data_bit_tick;
            cap_data_bit      = data_bit;
            cap_stuff_error   = stuff_error;
            cap_insert_stuff  = insert_stuff;
            cap_stuff_tx_bit  = stuff_tx_bit;
            @(negedge clk);
            bit_tick = 1'b0;
            @(negedge clk); // idle cycle between bits
        end
    endtask

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

    initial begin
        clk = 0; rst_n = 0; bit_tick = 0; rx_can_sync = 1; stuffing_en = 0;
        repeat (3) @(negedge clk);
        rst_n = 1;
        @(negedge clk);

        // -------------------------------------------------------------
        // Reset-state check
        // -------------------------------------------------------------
        check(stuff_error == 1'b0, "post-reset: stuff_error deasserted");
        check(insert_stuff == 1'b0, "post-reset: insert_stuff deasserted (stuffing_en=0)");

        // -------------------------------------------------------------
        // Test 1 (Req M2-9): with stuffing_en=0, straight passthrough,
        // no stuffing logic active, data_bit_tick mirrors bit_tick.
        // -------------------------------------------------------------
        stuffing_en = 0;
        drive_bit(1'b0);
        check(cap_data_bit_tick == 1'b1 && cap_data_bit == 1'b0,
              "M2-9: outside stuffing region, bit passes through untouched");

        // -------------------------------------------------------------
        // Test 2 (Req M2-7/M2-8): enable stuffing, send 5 identical
        // dominant bits, then the correct opposite-polarity stuff bit.
        // Expect: no data_bit_tick on the stuff bit, no stuff_error.
        // -------------------------------------------------------------
        stuffing_en = 1;
        drive_bit(1'b0); // run bit 1 (dominant) - restarts run count after Test1's 1'b0
        drive_bit(1'b0); // run bit 2
        drive_bit(1'b0); // run bit 3
        drive_bit(1'b0); // run bit 4
        drive_bit(1'b0); // run bit 5 -> expect_stuff should assert after this tick
        check(cap_insert_stuff == 1'b1, "M2-7: insert_stuff asserted after 5 identical bits (TX side)");
        check(cap_stuff_tx_bit == 1'b1, "M2-8: stuff_tx_bit is opposite polarity of the run (recessive)");

        drive_bit(1'b1); // correct stuff bit (opposite polarity)
        check(cap_stuff_error == 1'b0, "M2-8: correctly-polarized stuff bit -> no stuff_error");
        check(cap_data_bit_tick == 1'b0, "M2-10: stuff bit is filtered out, no data_bit_tick pulse");

        // Following data bit should tick normally again
        drive_bit(1'b1);
        check(cap_data_bit_tick == 1'b1 && cap_data_bit == 1'b1,
              "M2-10: normal data bit after stuff bit passes through");

        // -------------------------------------------------------------
        // Test 3 (Req M2-11): 5 identical bits followed by a 6th
        // identical bit (violation) instead of the mandatory stuff bit.
        // -------------------------------------------------------------
        drive_bit(1'b0);
        drive_bit(1'b0);
        drive_bit(1'b0);
        drive_bit(1'b0);
        drive_bit(1'b0); // 5th identical -> expect_stuff = 1
        check(cap_insert_stuff == 1'b1, "M2-11 setup: expect_stuff armed before violation bit");
        drive_bit(1'b0); // violation: 6th identical bit instead of opposite-polarity stuff bit
        check(cap_stuff_error == 1'b1, "M2-11: 6 consecutive identical bits -> stuff_error asserted");

        // -------------------------------------------------------------
        // Test 4 (Req M2-9): de-asserting stuffing_en mid-run clears
        // tracking so a new frame's run count starts clean.
        // -------------------------------------------------------------
        stuffing_en = 0;
        drive_bit(1'b0);
        stuffing_en = 1;
        drive_bit(1'b0); // 1st bit of a fresh run - must not be counted with prior history
        drive_bit(1'b0);
        drive_bit(1'b0);
        drive_bit(1'b0);
        check(cap_insert_stuff == 1'b0,
              "M2-9: stuffing_en toggling resets run tracking (only 4 bits counted so far)");

        // -------------------------------------------------------------
        // Summary
        // -------------------------------------------------------------
        $display("--------------------------------------------------");
        $display("bit_stuffer TB: %0d checks, %0d failures", checks, errors);
        if (errors == 0)
            $display("RESULT: PASS - all SOW Module-1/Milestone-2 stuffing/destuffing requirements met");
        else
            $display("RESULT: FAIL - see log above");
        $display("--------------------------------------------------");
        $finish;
    end

endmodule
