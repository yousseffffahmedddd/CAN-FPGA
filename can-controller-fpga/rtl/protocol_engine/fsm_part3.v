`include "../common/can_defs.vh"

// =============================================================================
// Module      : fsm_part3
// Description : CRC delimiter / ACK / EOF / Intermission handling for the
//               Protocol Engine. Interfaces to `crc_gen_check` and to the
//               bit-stuffer control. Produces error/success pulses consumed by
//               `error_mgmt` and outputs TX-control overrides during the
//               post-data phases (CRC..Intermission).
//
// Notes: This is a pragmatic, self-contained implementation that expects the
// higher-level FSM (fsm_part2) to drive `current_state` and to provide the
// transmit enable sample (`tx_en_sample`) at SOF so this module can decide
// whether this node was the transmitter for the frame.
// =============================================================================

module fsm_part3 (
    input  wire        clk,
    input  wire        rst_n,

    // Timing / bus monitoring
    input  wire        bit_tick,        // 1-cycle sample pulse per bit
    input  wire        data_bit_tick,   // from `bit_stuffer`: pulses only for destuffed data bits
    input  wire        data_bit,        // destuffed logical data bit (valid when data_bit_tick=1)
    input  wire        rx_can_sync,     // observed bus bit at sample point (1=recessive)

    // Hook into fsm_part2: sample of state and whether we were driving at SOF
    input  wire [2:0]  current_state,   // wire from fsm_part2.current_state
    input  wire        tx_en_sample,    // sampled `tx_en` at SOF entry (1 = we were driving -> transmitter)

    // Inputs from bit_stuffer (for reporting)
    input  wire        stuff_error_in,

    // Outputs to other blocks
    output reg         stuffing_en,     // asserted during SOF..CRC inclusive

    // CRC engine interface (drives crc_gen_check)
    output reg         crc_init,        // 1-cycle pulse at frame start (SOF)
    output reg         crc_latch,       // 1-cycle pulse at CRC entry to latch crc_out
    output reg         crc_field_rx,    // when 1, crc_gen_check treats incoming bits as RX-CRC bits
    output reg         crc_bit_strobe,  // pulses once per (destuffed) bit to step the CRC engine

    // TX override outputs for post-data phases (CRC..Intermission)
    output reg         tx_en_out,       // when high, this module drives the bus
    output reg         tx_can_out,      // driven bit when tx_en_out=1 (1=recessive)

    // Error / status pulses (1-cycle)
    output reg         crc_err_pulse,   // CRC mismatch detected (from CRC checker)
    output reg         stuff_err_pulse, // forwarded from bit_stuffer
    output reg         ack_err_pulse,   // no ACK observed while TX
    output reg         tx_success_pulse,// ACK observed while TX
    output reg         rx_success_pulse,// received frame OK (CRC match)
    output reg         recessive11_pulse// 11 consecutive recessive bits observed
);

    // Local parameters: post-CRC subsequence lengths (in bit periods)
    localparam integer CRC_BITS       = 15;
    localparam integer CRC_DELIM_BITS = 1;
    localparam integer ACK_SLOT_BITS  = 1;
    localparam integer ACK_DELIM_BITS = 1;
    localparam integer EOF_BITS       = 7;
    localparam integer INTER_BITS     = 3;

    // FSM for this module: idle/accumulating -> crc_rx -> post_crc sequence
    reg prev_state_is_crc;
    // State tag aligned with bit_stuffer's one-clock-delayed data_bit_tick.
    // It records whether the physical bit sampled on the preceding bit_tick
    // belonged to the CRC field.
    reg sampled_bit_was_crc;
    reg node_is_tx; // latched at SOF entry

    // CRC shifting for TX
    reg [14:0] crc_shift_reg;

    // CRC bit counter (counts 0..14 while receiving/transmitting CRC bits)
    reg [4:0] crc_cnt;

    // Post-CRC subsequence counter (driven by bit_tick)
    reg [4:0] post_cnt;
    reg in_post_crc_seq;
    reg ack_seen;
    reg crc_latch_d;

    // Recessive-run counter (for recessive11 pulse)
    reg [4:0] recessive_run;

    // instantiate CRC checker (local instance) so we get crc_error and crc_out
    wire [14:0] crc_out_w;
    wire crc_error_w;

    crc_gen_check crc_inst (
        .clk        (clk),
        .rst_n      (rst_n),
        .crc_init   (crc_init),
        .bit_strobe (crc_bit_strobe),
        .logical_bit(data_bit),
        .crc_field_rx(crc_field_rx),
        .crc_latch  (crc_latch),
        .crc_out    (crc_out_w),
        .crc_error  (crc_error_w)
    );

    // detect edge into CRC state from the supplied `current_state`
    wire is_state_crc = (current_state == `CAN_STATE_CRC);

    // Main sequential datapath
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            stuffing_en      <= 1'b0;
            crc_init         <= 1'b0;
            crc_latch        <= 1'b0;
            crc_field_rx     <= 1'b0;
            crc_bit_strobe   <= 1'b0;
            tx_en_out        <= 1'b0;
            tx_can_out       <= `CAN_RECESSIVE;
            crc_err_pulse    <= 1'b0;
            stuff_err_pulse  <= 1'b0;
            ack_err_pulse    <= 1'b0;
            tx_success_pulse <= 1'b0;
            rx_success_pulse <= 1'b0;
            recessive11_pulse<= 1'b0;
            prev_state_is_crc<= 1'b0;
            sampled_bit_was_crc <= 1'b0;
            node_is_tx       <= 1'b0;
            crc_shift_reg    <= 15'h0;
            crc_cnt          <= 5'd0;
            post_cnt         <= 5'd0;
            in_post_crc_seq  <= 1'b0;
            ack_seen          <= 1'b0;
            crc_latch_d        <= 1'b0;
            recessive_run    <= 5'd0;
        end
        else begin
            // default deassert single-cycle pulses
            crc_init         <= 1'b0;
            crc_latch        <= 1'b0;
            crc_bit_strobe   <= 1'b0;
            crc_err_pulse    <= 1'b0;
            stuff_err_pulse  <= 1'b0;
            ack_err_pulse    <= 1'b0;
            tx_success_pulse <= 1'b0;
            rx_success_pulse <= 1'b0;
            recessive11_pulse<= 1'b0;

            // stuffing region: SOF..CRC inclusive per project note
            stuffing_en <= (current_state == `CAN_STATE_SOF) ||
                           (current_state == `CAN_STATE_ARBITRATION) ||
                           (current_state == `CAN_STATE_CONTROL) ||
                           (current_state == `CAN_STATE_DATA) ||
                           (current_state == `CAN_STATE_CRC);

            // latch node_is_tx on SOF entry (sample provided externally)
            if (!prev_state_is_crc && (current_state == `CAN_STATE_SOF)) begin
                node_is_tx <= tx_en_sample;
                // initialize CRC engine at SOF
                crc_init <= 1'b1;
            end

            // bit_stuffer registers data_bit_tick/data_bit, so those signals
            // describe the physical bit sampled on the PREVIOUS bit_tick.
            // Remember the state that owned that physical bit so the DATA->CRC
            // boundary cannot misclassify the final DATA bit as CRC bit 0.
            if (bit_tick)
                sampled_bit_was_crc <= is_state_crc;

            // Accumulate all destuffed logical bits that were sampled before the
            // CRC field.  In particular, when current_state has just become CRC,
            // sampled_bit_was_crc is still 0 for the delayed final DATA bit.
            if (data_bit_tick && !sampled_bit_was_crc) begin
                crc_bit_strobe <= 1'b1;

                // The delayed final pre-CRC bit is the correct moment to latch.
                // crc_gen_check sees crc_bit_strobe and crc_latch together on
                // the following clock, so crc_accum_next includes this final bit.
                if (is_state_crc) begin
                    crc_latch       <= 1'b1;
                    crc_field_rx    <= 1'b0;
                    crc_cnt         <= 5'd0;
                    in_post_crc_seq <= 1'b0;
                    tx_en_out       <= node_is_tx;
                    if (!node_is_tx)
                        tx_can_out <= `CAN_RECESSIVE;
                end
            end

            // Entry into CRC only prepares the handoff.  Do NOT latch here:
            // the bit_stuffer still owes us the delayed final DATA bit.
            if (!prev_state_is_crc && is_state_crc) begin
                crc_cnt         <= 5'd0;
                in_post_crc_seq <= 1'b0;
                crc_field_rx    <= 1'b0;
                tx_en_out       <= node_is_tx;
                if (!node_is_tx)
                    tx_can_out <= `CAN_RECESSIVE;
            end

            // crc_out updates one clock after crc_latch is observed by the CRC
            // engine.  crc_latch_d therefore gives crc_out time to settle before
            // we preload the 15-bit TX shifter.  At the same point it is safe to
            // switch the checker into CRC-field capture mode.
            crc_latch_d <= crc_latch;
            if (crc_latch_d) begin
                crc_field_rx <= 1'b1;
                if (node_is_tx) begin
                    crc_shift_reg <= crc_out_w;
                    tx_can_out    <= crc_out_w[14];
                end
            end

            // Process only bits that were physically sampled while the field FSM
            // was in CRC.  This also naturally accepts the delayed 15th CRC bit
            // after fsm_part2 has moved on to ACK.
            if (data_bit_tick && sampled_bit_was_crc) begin
                crc_bit_strobe <= 1'b1;
                crc_field_rx   <= 1'b1;

                if (node_is_tx) begin
                    // The current MSB was already present on the bus for this
                    // sampled CRC bit.  Shift now and drive the NEXT CRC bit
                    // for the following bit period.
                    tx_can_out    <= crc_shift_reg[13];
                    crc_shift_reg <= {crc_shift_reg[13:0], 1'b0};
                end

                if (crc_cnt == CRC_BITS - 1) begin
                    // Completed last CRC bit -> check CRC error (crc_error_w)
                    if (crc_error_w)
                        crc_err_pulse <= 1'b1;
                    // move into post-CRC sequence (CRC_DELIM .. INTER)
                    in_post_crc_seq <= 1'b1;
                    post_cnt <= 5'd0;
                    ack_seen <= 1'b0;
                    // release bus for ACK slot (transmitter must stop driving)
                    tx_en_out <= 1'b0;
                    // stop CRC RX field flag (we will not feed crc bits any more)
                    crc_field_rx <= 1'b0;
                    crc_cnt <= 5'd0;
                end
                else begin
                    crc_cnt <= crc_cnt + 5'd1;
                end
            end

            // Never leave the post-data TX override latched high if the core
            // FSM has already left CRC before the internal CRC counter finishes.
            if (prev_state_is_crc && !is_state_crc && !in_post_crc_seq) begin
                tx_en_out    <= 1'b0;
                crc_field_rx <= 1'b0;
            end

            // Latch a dominant ACK anywhere during the ACK-slot window.
            // rx_can_sync is itself sampled/pipelined, so using a level latch
            // here is safer than relying on one exact system-clock edge.
            if (in_post_crc_seq && node_is_tx &&
                (post_cnt == 5'd1) &&
                (rx_can_sync == `CAN_DOMINANT)) begin
                ack_seen <= 1'b1;
            end

            // Post-CRC subsequence driven by `bit_tick` (not destuffed-only)
            if (in_post_crc_seq && bit_tick) begin
                post_cnt <= post_cnt + 5'd1;

                // Sequence mapping by cumulative post_cnt value
                // 0 : CRC_DELIM
                // 1 : ACK_SLOT
                // 2 : ACK_DELIM
                // 3..9 : EOF (7 bits)
                // 10..12 : INTERMISSION (3 bits)

                // ACK timing is aligned to the registered rx_can_sync sample.
                // post_cnt 0 completes the CRC delimiter and prepares the ACK
                // slot; post_cnt 1 is the physical ACK slot; at post_cnt 2 the
                // registered sample from that ACK slot is available to inspect.
                if (post_cnt == 5'd0) begin
                    if (!node_is_tx && !crc_error_w) begin
                        tx_en_out  <= 1'b1;
                        tx_can_out <= `CAN_DOMINANT;
                    end
                end

                if (post_cnt == 5'd1) begin
                    if (!node_is_tx) begin
                        // Release immediately after the ACK slot so the ACK
                        // delimiter remains recessive.
                        tx_en_out  <= 1'b0;
                        tx_can_out <= `CAN_RECESSIVE;
                    end
                end

                if (post_cnt == 5'd2 && node_is_tx) begin
                    // Accept either the latched ACK-slot observation or the
                    // currently visible synchronized dominant level.
                    if (ack_seen || (rx_can_sync == `CAN_DOMINANT))
                        tx_success_pulse <= 1'b1;
                    else
                        ack_err_pulse <= 1'b1;
                end

                // End of Intermission -> conclude frame and optionally assert rx_success
                if (post_cnt == (CRC_DELIM_BITS + ACK_SLOT_BITS + ACK_DELIM_BITS + EOF_BITS + INTER_BITS - 1)) begin
                    in_post_crc_seq <= 1'b0;
                    // if we were receiver and CRC matched, indicate RX success
                    if (!node_is_tx && !crc_error_w)
                        rx_success_pulse <= 1'b1;
                end
            end

            // Forward stuffing error pulses from bit_stuffer
            if (stuff_error_in)
                stuff_err_pulse <= 1'b1;

            // Track 11 consecutive recessive bits on the bus (uses bit_tick sampling)
            if (bit_tick) begin
                if (rx_can_sync == `CAN_RECESSIVE) begin
                    if (recessive_run == 5'd31)
                        recessive_run <= recessive_run; // saturate (shouldn't happen)
                    else
                        recessive_run <= recessive_run + 5'd1;
                end
                else begin
                    recessive_run <= 5'd0;
                end
                if (recessive_run >= 5'd11)
                    recessive11_pulse <= 1'b1;
            end

            prev_state_is_crc <= is_state_crc;
        end
    end

endmodule
