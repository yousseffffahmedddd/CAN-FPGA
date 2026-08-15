`include "../common/can_defs.vh"

// =============================================================================
// Module      : fsm_part1
// Description : Bit Timing Logic sub-block of the CAN Protocol Engine.
//               Maps to Figure 1-4: "Bit Timing Logic" + "Sample<2:0>" +
//               "Majority Decision" + the "SAM" mux, producing the two
//               signals fsm_part2.v depends on: bit_tick and rx_can_sync.
//
// Position in system: RX_raw (physical pin, after transceiver) -> this module
//                     -> {bit_tick, rx_can_sync} -> fsm_part2.v / bit_stuffer.v
//                     / crc_gen_check.v
// =============================================================================

module fsm_part1 #(
    parameter integer CLKS_PER_TQ   = `CAN_CLKS_PER_TQ,   // System clocks per Time Quantum
    parameter integer TQ_PER_BIT    = `CAN_TQ_PER_BIT,    // Total TQ per bit
    parameter integer SAMPLE_TQ_IDX = `CAN_SAMPLE_TQ_IDX, // 0-based TQ index of the sample point
    parameter         SAM_MODE      = `CAN_SAM_MODE       // 0 = single-sample, 1 = triple-sample majority vote
)(
    input  wire clk,           // System clock
    input  wire rst_n,         // Active-low asynchronous reset
    input  wire rx_pin,          // Raw asynchronous RX from CAN transceiver

    output reg  bit_tick,        // 1-cycle pulse at the sample point of every bit
    output reg  rx_can_sync,     // Synchronized, sampled RX bit (1=recessive, 0=dominant)

    // Debug / monitoring
    output wire [4:0] tq_index,  // Current Time Quantum index within the bit
    output wire       bus_idle   // 1 = bus has been recessive for >=1 full bit period
);

    localparam integer CLK_CNT_W = (CLKS_PER_TQ  <= 1) ? 1 : $clog2(CLKS_PER_TQ);
    localparam integer TQ_CNT_W  = (TQ_PER_BIT   <= 1) ? 1 : $clog2(TQ_PER_BIT);

    // =========================================================================
    // 2-FF synchronizer on the raw async RX pin
    // =========================================================================
    wire rx_safe;
    rxcan_sync rx_sync_inst (
        .clk        (clk),
        .rst_n      (rst_n),
        .rx_pin     (rx_pin),
        .rx_can_sync(rx_safe)
    );

    // =========================================================================
    // Recessive->Dominant edge detector on rx_safe
    // =========================================================================
    reg  rx_safe_prev;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) rx_safe_prev <= `CAN_RECESSIVE;
        else        rx_safe_prev <= rx_safe;
    end
    wire dominant_edge = (rx_safe_prev == 1'b1) && (rx_safe == 1'b0);

    // =========================================================================
    // Time Quantum (TQ) pulse generator
    // =========================================================================
    wire tq_pulse;
    wire [TQ_CNT_W-1:0] tq_idx_int;
    wire hard_sync_event_internal;
    bit_clk_gen #(
        .CLKS_PER_TQ(CLKS_PER_TQ),
        .TQ_PER_BIT (TQ_PER_BIT)
    ) bit_clk_inst (
        .clk        (clk),
        .rst_n      (rst_n),
        .hard_sync  (hard_sync_event_internal),
        .tq_pulse   (tq_pulse),
        .tq_idx     (tq_idx_int)
    );

    assign tq_index = tq_idx_int;

    // =========================================================================
    // Sample<2:0>: shift register holding the last 3 samples of rx_safe
    // =========================================================================
    wire [2:0] sample_shift;
    shift_reg #(.WIDTH(3)) sample_shift_reg (
        .clk         (clk),
        .rst_n       (rst_n),
        .load        (1'b0),
        .shift_en    (tq_pulse),
        .serial_in   (rx_safe),
        .parallel_in (3'b111),
        .serial_out  (),
        .parallel_out(sample_shift)
    );

    // =========================================================================
    // Majority Decision block (combinational 2-of-3 vote)
    // =========================================================================
    wire majority_bit = (sample_shift[0] & sample_shift[1]) |
                        (sample_shift[1] & sample_shift[2]) |
                        (sample_shift[0] & sample_shift[2]);

    // =========================================================================
    // SAM mux: selects single-sample vs. triple-sample majority result
    // =========================================================================
    wire sam_selected_bit = (SAM_MODE) ? majority_bit : sample_shift[0];

    // =========================================================================
    // Bus-idle tracker: gates hard sync so it only fires on a genuine SOF edge
    // =========================================================================
    reg bus_idle_r;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            bus_idle_r <= `CAN_RECESSIVE;
        else if (tq_pulse && (tq_idx_int == SAMPLE_TQ_IDX))
            bus_idle_r <= sam_selected_bit; // 1=recessive->still idle, 0=dominant->frame active
    end
    assign bus_idle = bus_idle_r;

    wire hard_sync_event = bus_idle_r && dominant_edge;
    assign hard_sync_event_internal = hard_sync_event;

    // =========================================================================
    // Output stage: at the sample-point TQ, latch the synchronized bit and
    // pulse bit_tick for exactly 1 clk cycle
    // =========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            bit_tick    <= 1'b0;
            rx_can_sync <= `CAN_RECESSIVE; // recessive default
        end
        else begin
            bit_tick <= 1'b0; // default: deassert (1-cycle pulse)

            if (tq_pulse && (tq_idx_int == SAMPLE_TQ_IDX)) begin
                rx_can_sync <= sam_selected_bit;
                bit_tick    <= 1'b1;
            end
        end
    end

endmodule

// =============================================================================
// Module      : bit_clk_gen
// Description : Baud Rate Generator producing TQ pulses and indexes.
// =============================================================================
module bit_clk_gen #(
    parameter integer CLKS_PER_TQ = 1,
    parameter integer TQ_PER_BIT  = 8
)(
    input  wire clk,
    input  wire rst_n,
    input  wire hard_sync,

    output reg  tq_pulse,
    output reg  [($clog2(TQ_PER_BIT) > 0 ? $clog2(TQ_PER_BIT) : 1)-1:0] tq_idx
);

    localparam integer CLK_CNT_W = (CLKS_PER_TQ <= 1) ? 1 : $clog2(CLKS_PER_TQ);
    localparam integer TQ_CNT_W  = (TQ_PER_BIT <= 1) ? 1 : $clog2(TQ_PER_BIT);

    reg [CLK_CNT_W-1:0] clk_div_cnt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            clk_div_cnt <= {CLK_CNT_W{1'b0}};
            tq_pulse    <= 1'b0;
            tq_idx      <= {TQ_CNT_W{1'b0}};
        end
        else if (hard_sync) begin
            clk_div_cnt <= {CLK_CNT_W{1'b0}};
            tq_pulse    <= 1'b1;
            tq_idx      <= {TQ_CNT_W{1'b0}};
        end
        else if (CLKS_PER_TQ == 1) begin
            clk_div_cnt <= {CLK_CNT_W{1'b0}};
            tq_pulse    <= 1'b1;
            if (tq_idx == TQ_PER_BIT - 1)
                tq_idx <= {TQ_CNT_W{1'b0}};
            else
                tq_idx <= tq_idx + 1'b1;
        end
        else if (clk_div_cnt == CLKS_PER_TQ - 1) begin
            clk_div_cnt <= {CLK_CNT_W{1'b0}};
            tq_pulse    <= 1'b1;
            if (tq_idx == TQ_PER_BIT - 1)
                tq_idx <= {TQ_CNT_W{1'b0}};
            else
                tq_idx <= tq_idx + 1'b1;
        end
        else begin
            clk_div_cnt <= clk_div_cnt + 1'b1;
            tq_pulse    <= 1'b0;
        end
    end
endmodule