// =============================================================================
// Module      : fsm_part1
// Description : Bit Timing Logic sub-block of the CAN Protocol Engine.
//                Maps to Figure 1-4: "Bit Timing Logic" + "Sample<2:0>" +
//                "Majority Decision" + the "SAM" mux, producing the two
//                signals fsm_part2.v depends on: bit_tick and rx_can_sync.
//
// Position in system: RX_raw (physical pin, after transceiver) -> this module
//                      -> {bit_tick, rx_can_sync} -> fsm_part2.v / bit_stuffer.v
//                      / crc_gen_check.v
//
// Design notes:
//   - Configuration (CLKS_PER_TQ, TQ_PER_BIT, SAMPLE_TQ_IDX, SAM_MODE) is
//     synthesis-time hardcoded via parameters, consistent with the project's
//     "hardcoded configuration parameters" simplification (no CNF1-3 runtime
//     register writes in this design's scope).
//   - Implements HARD synchronization only (on the SOF edge, while bus is
//     idle), per CAN spec: "the bit timing... DPLL... provide the nominal
//     timing for transmitted data... hard sync only on SOF." Continuous
//     resynchronization (SJW-based soft sync mid-frame) is NOT implemented
//     here -- flagged as a known simplification for this course-scope design.
//   - Includes a 2-FF synchronizer on the raw RX pin (matches the project's
//     established 2-FF synchronizer convention at clock domain boundaries).
// =============================================================================

module fsm_part1 #(
    parameter integer CLKS_PER_TQ   = 25,   // System clocks per Time Quantum (derived from Fosc & BRP)
    parameter integer TQ_PER_BIT    = 8,    // Total TQ per bit: Sync_Seg+PropSeg+PS1+PS2 (CAN spec: 8-25)
    parameter integer SAMPLE_TQ_IDX = 6,    // 0-based TQ index of the sample point (~75% of bit time)
    parameter         SAM_MODE      = 1'b1  // 0 = single-sample, 1 = triple-sample majority vote
)(
    input  wire clk,             // System clock
    input  wire rst_n,           // Active-low asynchronous reset
    input  wire rx_pin,          // Raw asynchronous RX from CAN transceiver

    output reg  bit_tick,        // 1-cycle pulse at the sample point of every bit
    output reg  rx_can_sync,     // Synchronized, sampled RX bit (1=recessive, 0=dominant)

    // Debug / monitoring (optional, mirrors current_state style in fsm_part2)
    output wire [4:0] tq_index,  // Current Time Quantum index within the bit
    output wire        bus_idle  // 1 = bus has been recessive for >=1 full bit period
);

    localparam integer CLK_CNT_W = (CLKS_PER_TQ  <= 1) ? 1 : $clog2(CLKS_PER_TQ);
    localparam integer TQ_CNT_W  = (TQ_PER_BIT   <= 1) ? 1 : $clog2(TQ_PER_BIT);

    // =========================================================================
    // 2-FF synchronizer on the raw async RX pin (clock-domain-crossing safety)
    // =========================================================================
    reg rx_ff0, rx_ff1;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_ff0 <= 1'b1;   // bus idles recessive
            rx_ff1 <= 1'b1;
        end else begin
            rx_ff0 <= rx_pin;
            rx_ff1 <= rx_ff0;
        end
    end
    wire rx_safe = rx_ff1;    // metastability-safe raw RX bit

    // =========================================================================
    // Recessive->Dominant edge detector on rx_safe (checked every clk, not
    // just at TQ boundaries, so a hard sync can react as fast as possible)
    // =========================================================================
    reg  rx_safe_prev;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) rx_safe_prev <= 1'b1;
        else        rx_safe_prev <= rx_safe;
    end
    wire dominant_edge = (rx_safe_prev == 1'b1) && (rx_safe == 1'b0);

    // =========================================================================
    // Bus-idle tracker: gates hard sync so it only fires on a genuine SOF
    // edge (bus recessive for the whole previous bit), not mid-frame edges
    // =========================================================================
    reg bus_idle_r;
    assign bus_idle = bus_idle_r;

    wire hard_sync_event = bus_idle_r && dominant_edge;

    // =========================================================================
    // Time Quantum (TQ) pulse generator: divides clk by CLKS_PER_TQ
    // =========================================================================
    reg [CLK_CNT_W-1:0] clk_div_cnt;
    reg                 tq_pulse;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            clk_div_cnt <= {CLK_CNT_W{1'b0}};
            tq_pulse    <= 1'b0;
        end
        else if (hard_sync_event) begin
            // Restart the TQ phase immediately at the SOF edge
            clk_div_cnt <= {CLK_CNT_W{1'b0}};
            tq_pulse    <= 1'b0;
        end
        else if (clk_div_cnt == CLKS_PER_TQ - 1) begin
            clk_div_cnt <= {CLK_CNT_W{1'b0}};
            tq_pulse    <= 1'b1;   // 1-cycle pulse marking a TQ boundary
        end
        else begin
            clk_div_cnt <= clk_div_cnt + 1'b1;
            tq_pulse    <= 1'b0;
        end
    end

    // =========================================================================
    // Bit-relative TQ index counter (0 .. TQ_PER_BIT-1), reset by hard sync
    // =========================================================================
    reg [TQ_CNT_W-1:0] tq_idx_r;
    assign tq_index = tq_idx_r;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            tq_idx_r <= {TQ_CNT_W{1'b0}};
        else if (hard_sync_event)
            tq_idx_r <= {TQ_CNT_W{1'b0}};
        else if (tq_pulse) begin
            if (tq_idx_r == TQ_PER_BIT - 1)
                tq_idx_r <= {TQ_CNT_W{1'b0}};
            else
                tq_idx_r <= tq_idx_r + 1'b1;
        end
    end

    // =========================================================================
    // Sample<2:0>: shift register holding the last 3 samples of rx_safe,
    // captured once per TQ pulse
    // =========================================================================
    reg [2:0] sample_shift; // sample_shift[0] = most recent sample

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            sample_shift <= 3'b111; // recessive default
        else if (tq_pulse)
            sample_shift <= {sample_shift[1:0], rx_safe};
    end

    // =========================================================================
    // Majority Decision block (combinational 2-of-3 vote)
    // =========================================================================
    wire majority_bit = (sample_shift[0] & sample_shift[1]) |
                         (sample_shift[1] & sample_shift[2]) |
                         (sample_shift[0] & sample_shift[2]);

    // =========================================================================
    // SAM mux: selects single-sample vs. triple-sample majority result
    // =========================================================================
    wire sam_selected_bit = SAM_MODE ? majority_bit : sample_shift[0];

    // =========================================================================
    // Output stage: at the sample-point TQ, latch the synchronized bit and
    // pulse bit_tick for exactly 1 clk cycle
    // =========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            bit_tick    <= 1'b0;
            rx_can_sync <= 1'b1; // recessive default
        end
        else begin
            bit_tick <= 1'b0; // default: deassert (1-cycle pulse)

            if (tq_pulse && (tq_idx_r == SAMPLE_TQ_IDX)) begin
                rx_can_sync <= sam_selected_bit;
                bit_tick    <= 1'b1;
            end
        end
    end

    // =========================================================================
    // Bus-idle tracking: updated once per bit, using the just-latched
    // rx_can_sync value from the previous bit_tick
    // =========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            bus_idle_r <= 1'b1; // assume idle at reset
        else if (tq_pulse && (tq_idx_r == SAMPLE_TQ_IDX))
            bus_idle_r <= sam_selected_bit; // 1=recessive->still idle, 0=dominant->frame active
    end

endmodule