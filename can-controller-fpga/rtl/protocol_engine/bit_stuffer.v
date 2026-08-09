// =============================================================================
// Module      : bit_stuffer
// Description : Bit Stuffing (TX) / Bit Destuffing (RX) sub-block of the CAN
//                Protocol Engine. Maps to Figure 1-4: "StuffReg<5:0>" +
//                "Comparator" fed by "BusMon".
//
// Position in system:
//   RX/monitoring path : fsm_part1.v --(bit_tick, rx_can_sync = BusMon)-->
//                         bit_stuffer.v --(data_bit_tick, data_bit)-->
//                         fsm_part2.v (field bit counters) / crc_gen_check.v
//   TX path             : fsm_part2.v's TX bit-select mux must consult
//                         insert_stuff / stuff_tx_bit from this module and,
//                         when insert_stuff=1, drive stuff_tx_bit on tx_can
//                         for that bit_tick INSTEAD OF consuming the next
//                         field/PISO bit (and must not advance its own
//                         bit_cnt that tick).
//
// Key design point: CAN monitors its OWN transmitted bits on the same
// physical wire (BusMon), so a single run-length tracker driven off
// rx_can_sync serves BOTH directions -- no separate TX-only bit history is
// needed. This matches Figure 1-4's single shared StuffReg/Comparator block.
//
// Scope: active only while stuffing_en=1, which the Protocol FSM should
// assert for states SOF..CRC (per Module 1 SOW: "stuffing applies only
// between SOF and the CRC delimiter"). De-asserted for ACK/EOF/Intermission.
//
// Implementation note: functionally equivalent to the datasheet's
// StuffReg<5:0> shift-register + Comparator, realized here as a run-length
// counter (last_bit + same_count) instead of an explicit 6-bit shift
// register -- same behavior, fewer flip-flops, easier to verify.
// =============================================================================

module bit_stuffer (
    input  wire clk,             // System clock
    input  wire rst_n,           // Active-low asynchronous reset

    input  wire bit_tick,        // From fsm_part1: 1-cycle pulse per bit period
    input  wire rx_can_sync,     // From fsm_part1: observed bus bit (BusMon)
    input  wire stuffing_en,     // From Protocol FSM: 1 = active stuffing region

    // ---- RX / Destuffing interface ----
    output reg  data_bit_tick,   // Pulses only for real frame-data bits (stuff bits filtered out)
    output reg  data_bit,        // Destuffed bit value, valid when data_bit_tick=1
    output reg  stuff_error,     // 1-cycle pulse: stuffing violation (6 identical bits) detected

    // ---- TX / Stuffing-insertion interface ----
    output wire insert_stuff,    // 1 = TX mux must drive stuff_tx_bit this tick, not a field bit
    output wire stuff_tx_bit     // Value to drive when insert_stuff=1 (opposite of last observed bit)
);

    localparam integer RUN_LIMIT = 5; // stuff after 5 consecutive identical bits

    reg [2:0] same_count;   // consecutive identical-bit run length observed so far
    reg       last_bit;     // polarity of the last bit in that run
    reg       expect_stuff; // 1 = the NEXT bit_tick is the mandatory stuff bit

    // TX-side outputs are simple combinational reads of the same tracked state
    assign insert_stuff = stuffing_en && expect_stuff;
    assign stuff_tx_bit = ~last_bit;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            same_count    <= 3'd0;
            last_bit      <= 1'b1;   // recessive default (idle bus)
            expect_stuff  <= 1'b0;
            data_bit_tick <= 1'b0;
            data_bit      <= 1'b1;
            stuff_error   <= 1'b0;
        end
        else begin
            data_bit_tick <= 1'b0;   // defaults: 1-cycle pulses deassert unless re-driven below
            stuff_error   <= 1'b0;

            if (!stuffing_en) begin
                // Outside the stuffed region (IDLE, or ACK/EOF/Intermission once
                // fsm_part3 exists): pass bits straight through, reset tracking
                // so the next frame's SOF starts a clean run count.
                same_count   <= 3'd0;
                expect_stuff <= 1'b0;
                last_bit     <= 1'b1;
                if (bit_tick) begin
                    data_bit_tick <= 1'b1;
                    data_bit      <= rx_can_sync;
                end
            end
            else if (bit_tick) begin

                if (expect_stuff) begin
                    // ---- This bit IS the mandatory stuff bit: verify & discard ----
                    if (rx_can_sync == last_bit)
                        stuff_error <= 1'b1;      // violation: not the opposite polarity
                    last_bit     <= rx_can_sync;
                    same_count   <= 3'd1;         // run restarts, stuff bit counts as bit 1
                    expect_stuff <= 1'b0;
                    // data_bit_tick intentionally NOT asserted: this bit is not frame data
                end
                else begin
                    // ---- Normal data bit: pass through to CRC / field counters ----
                    data_bit_tick <= 1'b1;
                    data_bit      <= rx_can_sync;

                    if (rx_can_sync == last_bit) begin
                        if (same_count == RUN_LIMIT - 1)
                            expect_stuff <= 1'b1;  // 5th identical bit: next bit must be stuffed
                        same_count <= same_count + 1'b1;
                    end
                    else begin
                        same_count   <= 3'd1;      // run broken, start counting new polarity
                        expect_stuff <= 1'b0;
                    end
                    last_bit <= rx_can_sync;
                end
            end
        end
    end

endmodule