`include "../common/can_defs.vh"

// =============================================================================
// Module : protocol_engine
// Top-level glue for the Protocol Engine: instantiates and wires
// - fsm_part1 (bit timing / sampling)
// - bit_stuffer (stuff/destuff)
// - fsm_part2 (field-level FSM)
// - fsm_part3 (CRC/ACK/EOF/Intermission)
// - error_mgmt (TEC/REC and fault-confinement)
//
// This module exposes the minimal external interface for the protocol engine:
//   - physical RX/TX pins to the transceiver
//   - PISO (input) and SIPO (output) handshake lines for frame data
//   - TEC/REC and error status outputs
// =============================================================================

module protocol_engine (
    input  wire        clk,
    input  wire        rst_n,

    // Physical bus
    input  wire        rx_pin,      // raw async RX from transceiver
    output wire        tx_can,      // value to drive on bus (1=recessive)
    output wire        tx_en,       // 1 = actively drive bus, 0 = Hi-Z / receive-only

    // PISO (Frame constructor) input
    input  wire        piso_data_in,
    input  wire        piso_valid,
    output wire        piso_req,

    // SIPO (Frame deconstructor) output
    output wire        sipo_data_out,
    output wire        sipo_valid,

    // Status / error outputs
    output wire [7:0]  tec,
    output wire [7:0]  rec,
    output wire        err_active,
    output wire        err_passive,
    output wire        bus_off,
    output wire        ewarn,

    // Optional visibility outputs
    output wire [4:0]  tq_index,
    output wire        bus_idle,
    output wire [2:0]  current_state,
    output wire [3:0]  latched_dlc
);

    // Internal signals
    wire        bit_tick;
    wire        rx_can_sync;

    // bit_stuffer signals
    wire        data_bit_tick;
    wire        data_bit;
    wire        insert_stuff;
    wire        stuff_tx_bit;
    wire        stuff_error_w;

    // fsm_part2 signals
    wire        tx_can_fsm2;
    wire        tx_en_fsm2;
    wire        bit_error_w;
    wire        arb_lost_w;
    wire [3:0]  latched_dlc_w;
    wire [2:0]  current_state_w;

    // fsm_part3 signals
    wire        stuffing_en_w;
    wire        f3_crc_err_pulse;
    wire        f3_stuff_err_pulse;
    wire        f3_ack_err_pulse;
    wire        f3_tx_success_pulse;
    wire        f3_rx_success_pulse;
    wire        f3_recessive11_pulse;
    wire        f3_tx_en_out;
    wire        f3_tx_can_out;

    // tx_en sampled at SOF for node_is_tx determination
    reg         prev_state_sof;
    reg         tx_en_sample_reg;

    // -------------------------------------------------------------------------
    // Instantiate fsm_part1 (bit timing / sample generator)
    // -------------------------------------------------------------------------
    fsm_part1 f1 (
        .clk        (clk),
        .rst_n      (rst_n),
        .rx_pin     (rx_pin),
        .bit_tick   (bit_tick),
        .rx_can_sync(rx_can_sync),
        .tq_index   (tq_index),
        .bus_idle   (bus_idle)
    );

    // -------------------------------------------------------------------------
    // Instantiate bit_stuffer
    // -------------------------------------------------------------------------
    bit_stuffer bs (
        .clk          (clk),
        .rst_n        (rst_n),
        .bit_tick     (bit_tick),
        .rx_can_sync  (rx_can_sync),
        .stuffing_en  (stuffing_en_w),
        .data_bit_tick(data_bit_tick),
        .data_bit     (data_bit),
        .stuff_error  (stuff_error_w),
        .insert_stuff (insert_stuff),
        .stuff_tx_bit (stuff_tx_bit)
    );

    // -------------------------------------------------------------------------
    // Instantiate fsm_part2 (core field-level FSM)
    // -------------------------------------------------------------------------
    fsm_part2 f2 (
        .clk          (clk),
        .rst_n        (rst_n),
        .bit_tick     (bit_tick),
        .rx_can_sync  (rx_can_sync),
        .tx_can       (tx_can_fsm2),
        .tx_en        (tx_en_fsm2),
        .piso_data_in (piso_data_in),
        .piso_valid   (piso_valid),
        .piso_req     (piso_req),
        .sipo_data_out(sipo_data_out),
        .sipo_valid   (sipo_valid),
        .bit_error    (bit_error_w),
        .arb_lost     (arb_lost_w),
        .latched_dlc  (latched_dlc_w),
        .current_state(current_state_w)
    );

    assign current_state = current_state_w;
    assign latched_dlc  = latched_dlc_w;

    // -------------------------------------------------------------------------
    // Instantiate fsm_part3 (CRC/ACK/EOF/Intermission handling)
    // -------------------------------------------------------------------------
    fsm_part3 f3 (
        .clk             (clk),
        .rst_n           (rst_n),
        .bit_tick        (bit_tick),
        .data_bit_tick   (data_bit_tick),
        .data_bit        (data_bit),
        .rx_can_sync     (rx_can_sync),
        .current_state   (current_state_w),
        .tx_en_sample    (tx_en_sample_reg),
        .stuff_error_in  (stuff_error_w),

        .stuffing_en     (stuffing_en_w),
        .crc_init        (),
        .crc_latch       (),
        .crc_field_rx    (),
        .crc_bit_strobe  (),
        .tx_en_out       (f3_tx_en_out),
        .tx_can_out      (f3_tx_can_out),
        .crc_err_pulse   (f3_crc_err_pulse),
        .stuff_err_pulse (f3_stuff_err_pulse),
        .ack_err_pulse   (f3_ack_err_pulse),
        .tx_success_pulse(f3_tx_success_pulse),
        .rx_success_pulse(f3_rx_success_pulse),
        .recessive11_pulse(f3_recessive11_pulse)
    );

    // -------------------------------------------------------------------------
    // Instantiate error_mgmt
    // -------------------------------------------------------------------------
    error_mgmt em (
        .clk                      (clk),
        .rst_n                    (rst_n),
        .crc_err_pulse            (f3_crc_err_pulse),
        .stuff_err_pulse          (f3_stuff_err_pulse),
        .form_err_pulse           (1'b0),
        .bit_err_pulse            (bit_error_w),
        .ack_err_pulse            (f3_ack_err_pulse),
        .err_flag_bit_err_pulse   (1'b0),
        .node_is_tx               (tx_en_sample_reg),
        .tx_success_pulse         (f3_tx_success_pulse),
        .rx_success_pulse         (f3_rx_success_pulse),
        .recessive11_pulse        (f3_recessive11_pulse),
        .tec                      (tec),
        .rec                      (rec),
        .err_active               (err_active),
        .err_passive              (err_passive),
        .bus_off                  (bus_off),
        .ewarn                    (ewarn)
    );

    // -------------------------------------------------------------------------
    // TX arbitration between fsm_part2, bit_stuffer (inserted stuff bits),
    // and fsm_part3 (ACK drive / CRC TX override). Priority rules:
    //  - If fsm_part3 requests to drive (`f3_tx_en_out`), it has top priority
    //    (ACK slot, etc.). Otherwise
    //  - If `insert_stuff` is asserted and `tx_en_fsm2` is active, the
    //    stuffed bit (`stuff_tx_bit`) is driven instead of fsm_part2.tx_can.
    //  - Otherwise use fsm_part2's outputs.
    // -------------------------------------------------------------------------
    wire tx_en_from_fsm2_or_f3 = f3_tx_en_out ? 1'b1 : tx_en_fsm2;
    wire tx_can_from_fsm2_or_f3;
    assign tx_can_from_fsm2_or_f3 = f3_tx_en_out ? f3_tx_can_out :
                                    ((insert_stuff && tx_en_fsm2) ? stuff_tx_bit : tx_can_fsm2);

    assign tx_en = tx_en_from_fsm2_or_f3;
    assign tx_can = tx_can_from_fsm2_or_f3;

    // -------------------------------------------------------------------------
    // Sample tx_en at the SOF entry to determine whether this node was
    // the transmitter for the upcoming frame (node_is_tx). We detect SOF by
    // observing `current_state_w` rising into `CAN_STATE_SOF`.
    // -------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            prev_state_sof <= 1'b0;
            tx_en_sample_reg <= 1'b0;
        end else begin
            if ((current_state_w == `CAN_STATE_SOF) && !prev_state_sof) begin
                tx_en_sample_reg <= tx_en_fsm2;
            end
            prev_state_sof <= (current_state_w == `CAN_STATE_SOF);
        end
    end

endmodule
