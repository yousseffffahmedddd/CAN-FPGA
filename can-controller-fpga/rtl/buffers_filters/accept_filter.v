// =============================================================================
// accept_filter.v -- CAN acceptance filter, matching the MCP2515 datasheet's
// filter/mask architecture (Section 4.5, Figure 4-2/4-5):
//
//   RXB0 (higher priority): 1 mask (RXM0) + 2 filters (RXF0, RXF1)
//   RXB1 (lower priority):  1 mask (RXM1) + 4 filters (RXF2..RXF5)
//
// A message is checked against RXB0's filters first. If it matches, it goes
// to RXB0 ONLY (never both) -- Section 4.5.2's note that "only one filter
// match occurs". If RXB0 already holds an unread message and BUKT
// (rollover) is enabled, an RXB0-matching message rolls into RXB1 instead
// of being lost (Section 4.2.1) -- bypassing RXB1's own filters, exactly as
// the datasheet describes.
//
// SCOPE: standard 11-bit IDs only, matching the rest of this codebase
// (spi_if.v's byte map only implements SIDH/D0 addressing -- no EIDx
// registers exist anywhere in the current design, so extended 29-bit IDs
// are out of scope project-wide, not just here). Data byte filtering
// (Section 4.5.1, applying filter bits to the first two data bytes on
// standard frames) is also not implemented -- it's a minor DeviceNet-style
// convenience feature, not core to filter/mask compliance, and there are
// no spare EID filter bits in an 11-bit-only design to reuse for it.
//
// RXM[1:0] = 11 equivalent: tie rxb0_accept_all / rxb1_accept_all high to
// bypass that buffer's mask/filters entirely (Section 4.2.2).
//
// RTR (remote transmission request) is not itself filtered on -- filters
// only ever compare against the ID field (Section 4.5) -- but it rides
// along with id/data/dlc as part of the message and must be forwarded
// to whichever buffer accepts the frame, same as dlc.
//
// Purely combinational -- no internal state, so no clk/reset ports.
// =============================================================================

module accept_filter (
    // From protocol engine / CAN receiver front end
    input  wire        frame_valid,   // 1-cycle pulse: message received with no CRC/stuff/form/ack errors
    input  wire [10:0] rx_id_in,
    input  wire [63:0] rx_data_in,
    input  wire [3:0]  rx_dlc_in,
    input  wire        rx_rtr_in,     // 1 = remote frame (no data bytes), 0 = data frame

    // RXB0 configuration (from reg_bank: RXM0, RXF0, RXF1, RXB0CTRL.RXM[1:0], RXB0CTRL.BUKT)
    input  wire [10:0] rxm0_mask,
    input  wire [10:0] rxf0_id,
    input  wire [10:0] rxf1_id,
    input  wire        rxb0_accept_all, // RXM0[1:0] == 11: ignore mask/filters, accept anything
    input  wire        bukt,            // rollover enable: RXB0 full + match -> spill into RXB1

    // RXB1 configuration (from reg_bank: RXM1, RXF2..RXF5, RXB1CTRL.RXM[1:0])
    input  wire [10:0] rxm1_mask,
    input  wire [10:0] rxf2_id,
    input  wire [10:0] rxf3_id,
    input  wire [10:0] rxf4_id,
    input  wire [10:0] rxf5_id,
    input  wire        rxb1_accept_all, // RXM1[1:0] == 11

    // Buffer occupancy, so rollover knows whether RXB0 can accept
    input  wire        rxb0_full,       // from rx_buffer0.full

    // To rx_buffer0 (RXB0) and rx_buffer1 (RXB1)
    output wire        accept_rxb0,     // drive rx_buffer0.write_enable
    output wire        accept_rxb1,     // drive rx_buffer1.write_enable
    output wire [10:0] rx_id_out,
    output wire [63:0] rx_data_out,
    output wire [3:0]  rx_dlc_out,
    output wire        rx_rtr_out,      // forwarded unchanged to whichever buffer accepts

    // Filter-hit encoding for RXB0CTRL.FILHIT0 / RXB1CTRL.FILHIT[2:0]
    // (Section 4.5.3). Only meaningful when the corresponding accept_* is
    // asserted; ascending filter number wins on multiple matches (4.5.4).
    output reg         filhit0,         // RXB0: 0 = RXF0 matched, 1 = RXF1 matched
    output reg  [2:0]  filhit1          // RXB1: 0..3 = RXF2..RXF5, 4/5 = rollover from RXF0/RXF1
);

    // ---- RXB0 filter/mask compare -------------------------------------
    wire match_f0 = (rx_id_in & rxm0_mask) == (rxf0_id & rxm0_mask);
    wire match_f1 = (rx_id_in & rxm0_mask) == (rxf1_id & rxm0_mask);
    wire rxb0_match = rxb0_accept_all || match_f0 || match_f1;

    // ---- RXB1 filter/mask compare -------------------------------------
    wire match_f2 = (rx_id_in & rxm1_mask) == (rxf2_id & rxm1_mask);
    wire match_f3 = (rx_id_in & rxm1_mask) == (rxf3_id & rxm1_mask);
    wire match_f4 = (rx_id_in & rxm1_mask) == (rxf4_id & rxm1_mask);
    wire match_f5 = (rx_id_in & rxm1_mask) == (rxf5_id & rxm1_mask);
    wire rxb1_match = rxb1_accept_all || match_f2 || match_f3 || match_f4 || match_f5;

    // ---- Acceptance decisions -------------------------------------
    // RXB0 has priority: a message matching RXB0's filters goes to RXB0
    // only, never to RXB1 in the same cycle (Section 4.5.2).
    wire rxb0_hit  = frame_valid && rxb0_match;
    wire rollover  = rxb0_hit && rxb0_full && bukt;

    assign accept_rxb0 = rxb0_hit && !rxb0_full;
    assign accept_rxb1 = frame_valid && ((rxb1_match && !rxb0_match) || rollover);

    assign rx_id_out   = rx_id_in;
    assign rx_data_out = rx_data_in;
    assign rx_dlc_out  = rx_dlc_in;
    assign rx_rtr_out  = rx_rtr_in;

    // ---- FILHIT encoding (ascending filter number wins, 4.5.4) --------
    always @(*) begin
        filhit0 = match_f0 ? 1'b0 : 1'b1; // only meaningful when accept_rxb0 is asserted

        if (rollover)
            filhit1 = match_f0 ? 3'd4 : 3'd5;      // RXF0/RXF1 rolled over into RXB1
        else if (match_f2)
            filhit1 = 3'd0;
        else if (match_f3)
            filhit1 = 3'd1;
        else if (match_f4)
            filhit1 = 3'd2;
        else
            filhit1 = 3'd3;                        // match_f5, or rxb1_accept_all default
    end

endmodule