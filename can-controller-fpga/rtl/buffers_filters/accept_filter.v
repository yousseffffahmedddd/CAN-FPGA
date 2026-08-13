// =============================================================================
// accept_filter.v -- single acceptance filter + single acceptance mask for
// Module 2 (Buffers, Filters, & Masks).
//
// SCOPE (per Module 2 spec):
//   - Exactly ONE filter register and ONE mask register.
//   - Direct bitwise compare: a received ID matches only if every bit the
//     mask marks "care about" is identical between the incoming ID and the
//     filter register. Masked-out bits are ignored, same idea as the
//     MCP2515's RXMn/RXFn pair but scaled to a single instance instead of
//     2 masks / 6 filters across two buffers.
//   - Drives a single rx_buffer's write_enable. No RXB0/RXB1 split, no
//     buffer priority, no BUKT rollover, no FILHIT encoding -- all
//     explicitly out of scope per this module's Limitations.
//
// Standard 11-bit IDs only, matching the rest of this codebase (no EIDx
// addressing exists anywhere in this design, so extended 29-bit IDs are
// out of scope project-wide).
//
// To accept every incoming ID regardless of filter_id, tie mask to all
// zeros (no bits are "cared about", so the compare is vacuously true).
//
// RTR is not itself filtered on -- the filter only ever compares against
// the ID field -- but it rides along with id/data/dlc as part of the
// message and must be forwarded to the buffer on acceptance.
//
// Purely combinational -- no internal state, so no clk/reset ports.
// =============================================================================

module accept_filter (
    // From protocol engine / CAN receiver front end
    input  wire        frame_valid,  // 1-cycle pulse: message received with no CRC/stuff/form/ack errors
    input  wire [10:0] rx_id_in,
    input  wire [63:0] rx_data_in,
    input  wire [3:0]  rx_dlc_in,
    input  wire        rx_rtr_in,    // 1 = remote frame (no data bytes), 0 = data frame

    // Filter/mask configuration (from reg_bank)
    input  wire [10:0] filter_id,    // acceptance filter register
    input  wire [10:0] filter_mask,  // acceptance mask register: 1 = compare this bit, 0 = don't-care

    // To rx_buffer
    output wire        accept,       // drive rx_buffer.write_enable
    output wire [10:0] rx_id_out,
    output wire [63:0] rx_data_out,
    output wire [3:0]  rx_dlc_out,
    output wire        rx_rtr_out    // forwarded unchanged on acceptance
);

    // ---- Filter/mask compare -------------------------------------------
    // Only bits set in filter_mask are compared; masked-out bits always
    // match regardless of their value.
    wire id_match = (rx_id_in & filter_mask) == (filter_id & filter_mask);

    assign accept = frame_valid && id_match;

    assign rx_id_out   = rx_id_in;
    assign rx_data_out = rx_data_in;
    assign rx_dlc_out  = rx_dlc_in;
    assign rx_rtr_out  = rx_rtr_in;

endmodule
