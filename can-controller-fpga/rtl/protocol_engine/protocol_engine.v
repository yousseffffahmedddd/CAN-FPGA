`include "../common/can_defs.vh"

// =============================================================================
// Module : protocol_engine
// -----------------------------------------------------------------------------
// Top-level glue for the CAN Protocol Engine: instantiates and wires:
//   - fsm_part1 (bit timing / sampling)
//   - bit_stuffer (stuffing / destuffing)
//   - fsm_part2 (field-level FSM & arbitration)
//   - fsm_part3 (CRC / ACK / EOF / Intermission)
//   - error_mgmt (TEC/REC & fault confinement)
// =============================================================================

module protocol_engine (
    input  wire        clk,
    input  wire        rst_n,

    // Physical CAN bus pins
    input  wire        rx_pin,
    output wire        tx_can,
    output wire        tx_en,

    // TX Buffer Interface
    input  wire [10:0] txb_id,
    input  wire [63:0] txb_data,
    input  wire [3:0]  txb_dlc,
    input  wire        txb_rtr,
    input  wire        txb_txreq,

    // RX Buffer Interface
    output wire [10:0] rxb_id,
    output wire [63:0] rxb_data,
    output wire [3:0]  rxb_dlc,
    output wire        rxb_rtr,
    output wire        accept_rxb0,

    // Protocol Status & Pulses
    output wire        tx_done_pulse,
    output wire        rx_done_pulse,
    output wire        msg_err,
    output wire [7:0]  tec,
    output wire [7:0]  rec,
    output wire [1:0]  err_state,
    output wire        err_active,
    output wire        err_passive,
    output wire        bus_off,
    output wire        ewarn,
    output wire        bus_idle,

    // Visibility / Monitoring Ports
    output wire [2:0]  current_state,
    output wire [3:0]  latched_dlc,
    output wire [4:0]  tq_index,

    // Filtering inputs
    input  wire [10:0] rxm0_mask,
    input  wire [10:0] rxf0_id
);

    // -------------------------------------------------------------------------
    // Internal Signals
    // -------------------------------------------------------------------------
    wire        bit_tick;
    wire        rx_can_sync;

    // Bit stuffer signals
    wire        data_bit_tick;
    wire        data_bit;
    wire        insert_stuff;
    wire        stuff_tx_bit;
    wire        stuff_error_w;

    // FSM Part 2 signals
    wire        tx_can_fsm2;
    wire        tx_en_fsm2;
    wire        bit_error_w;
    wire        arb_lost_w;
    wire [3:0]  latched_dlc_w;
    wire [2:0]  current_state_w;
    wire        fsm2_bit_tick;

    // FSM Part 3 signals
    wire        stuffing_en_w;
    wire        f3_crc_err_pulse;
    wire        f3_stuff_err_pulse;
    wire        f3_ack_err_pulse;
    wire        f3_tx_success_pulse;
    wire        f3_rx_success_pulse;
    wire        f3_recessive11_pulse;
    wire        f3_tx_en_out;
    wire        f3_tx_can_out;

    // PISO / SIPO adapter signals
    wire        piso_data_in;
    wire        piso_valid;
    wire        piso_req;
    wire        sipo_data_out;
    wire        sipo_valid;

    // Internal sampling registers
    reg         prev_state_sof;
    reg         tx_en_sample_reg;

    // -------------------------------------------------------------------------
    // 1) Bit Timing Generator (fsm_part1)
    // -------------------------------------------------------------------------
    fsm_part1 f1 (
        .clk         (clk),
        .rst_n       (rst_n),
        .rx_pin      (rx_pin),
        .bit_tick    (bit_tick),
        .rx_can_sync (rx_can_sync),
        .tq_index    (tq_index),
        .bus_idle    (bus_idle)
    );

    // -------------------------------------------------------------------------
    // 2) Bit Stuffer / Destuffer (bit_stuffer)
    // -------------------------------------------------------------------------
    bit_stuffer bs (
        .clk           (clk),
        .rst_n         (rst_n),
        .bit_tick      (bit_tick),
        .rx_can_sync   (rx_can_sync),
        .stuffing_en   (stuffing_en_w),
        .data_bit_tick (data_bit_tick),
        .data_bit      (data_bit),
        .stuff_error   (stuff_error_w),
        .insert_stuff  (insert_stuff),
        .stuff_tx_bit  (stuff_tx_bit)
    );

// -------------------------------------------------------------------------
    // 3) Frame Serializer / Deserializer Adapter Logic
    // -------------------------------------------------------------------------
    // 82 logical bits after SOF: ID + RTR + IDE + r0 + DLC + DATA.
    // The old 108-bit register zero-extended this 82-bit concatenation and
    // injected 26 false dominant bits ahead of the real identifier.
    reg [81:0] tx_frame_shift;
    reg [81:0] rx_frame_shift;

    // // Construct simple standard frame shift register (ID + RTR + IDE + DLC + DATA)
    // always @(posedge clk or negedge rst_n) begin
    //     if (!rst_n) begin
    //         tx_frame_shift <= 82'd0;
    //     end else if (txb_txreq && (current_state_w == `CAN_STATE_IDLE)) begin 
    //         // Pack fields: 11-bit ID + 1-bit RTR + 1-bit IDE (0) + 4-bit DLC + 64-bit Data
    //         tx_frame_shift <= {txb_id, txb_rtr, 1'b0, txb_dlc, txb_data};
    //     end else if (bit_tick && piso_req) begin
    //         tx_frame_shift <= {tx_frame_shift[106:0], 1'b1}; // Shift left synchronized with bit_tick
    //     end
    // end
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx_frame_shift <= 82'd0;
        end else if (txb_txreq && (current_state_w == `CAN_STATE_IDLE)) begin
            // Exact MSB-first standard-frame fields following SOF.
            tx_frame_shift <= {txb_id, txb_rtr, 1'b0, 1'b0, txb_dlc, txb_data};
        end else if (fsm2_bit_tick && piso_req) begin
            // Do not consume a logical field bit while a stuff bit is inserted.
            tx_frame_shift <= {tx_frame_shift[80:0], 1'b1}; 
        end
    end

    assign piso_data_in = tx_frame_shift[81];
    assign piso_valid   = txb_txreq;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_frame_shift <= 82'd0;
        end else if (sipo_valid) begin
            rx_frame_shift <= {rx_frame_shift[80:0], sipo_data_out};
        end
    end

    // Map deserialized frame fields to RX buffer outputs
    assign rxb_id   = rx_frame_shift[81:71];
    assign rxb_rtr  = rx_frame_shift[70];
    assign rxb_dlc  = rx_frame_shift[67:64];
    assign rxb_data = rx_frame_shift[63:0];

    // -------------------------------------------------------------------------
    // 4) Core Field State Machine (fsm_part2)
    // // -------------------------------------------------------------------------
  // -------------------------------------------------------------------------
    // 4) Core Field State Machine (fsm_part2)
    // -------------------------------------------------------------------------
    // Stuff bits occupy physical time but are not logical field bits.
    // Freeze fsm_part2 and the serializer while the stuffing mux owns the bus.
    assign fsm2_bit_tick = bit_tick && !(insert_stuff && (tx_en_fsm2 || f3_tx_en_out));

    fsm_part2 f2 (
        .clk           (clk),
        .rst_n         (rst_n),
        .bit_tick      (fsm2_bit_tick),
        .rx_can_sync   (rx_can_sync),
        .txb_txreq     (txb_txreq),          // <--- Connect transmission request
        .dlc           (txb_dlc),            // <--- Connect DLC for frame length handling
        .tx_can        (tx_can_fsm2),
        .tx_en         (tx_en_fsm2),
        .piso_data_in  (piso_data_in),
        .piso_valid    (piso_valid),
        .piso_req      (piso_req),
        .sipo_data_out (sipo_data_out),
        .sipo_valid    (sipo_valid),
        .bit_error     (bit_error_w),
        .arb_lost      (arb_lost_w),
        .latched_dlc   (latched_dlc_w),
        .current_state (current_state_w)
    );
    // -------------------------------------------------------------------------
    // 4) Core Field State Machine (fsm_part2)
    // -------------------------------------------------------------------------
    // fsm_part2 f2 (
    //     .clk           (clk),
    //     .rst_n         (rst_n),
    //     .bit_tick      (bit_tick),
    //     .rx_can_sync   (rx_can_sync),
    //     .txb_txreq     (txb_txreq),           // <--- Added transmission request wire
    //     .dlc           (txb_dlc),             // <--- Passed for 44+8n frame length handling
    //     .tx_can        (tx_can_fsm2),
    //     .tx_en         (tx_en_fsm2),
    //     .piso_data_in  (piso_data_in),
    //     .piso_valid    (piso_valid),
    //     .piso_req      (piso_req),
    //     .sipo_data_out (sipo_data_out),
    //     .sipo_valid    (sipo_valid),
    //     .bit_error     (bit_error_w),
    //     .arb_lost      (arb_lost_w),
    //     .latched_dlc   (latched_dlc_w),
    //     .current_state (current_state_w)
    // );

    assign current_state = current_state_w;
    assign latched_dlc   = latched_dlc_w;

    // -------------------------------------------------------------------------
    // 5) CRC / ACK / EOF / Intermission Engine (fsm_part3)
    // -------------------------------------------------------------------------
    fsm_part3 f3 (
        .clk               (clk),
        .rst_n             (rst_n),
        .bit_tick          (bit_tick),
        .data_bit_tick     (data_bit_tick),
        .data_bit          (data_bit),
        .rx_can_sync       (rx_can_sync),
        .current_state     (current_state_w),
        .tx_en_sample      (tx_en_sample_reg),
        .stuff_error_in    (stuff_error_w),
        .stuffing_en       (stuffing_en_w),
        .crc_init          (),
        .crc_latch         (),
        .crc_field_rx      (),
        .crc_bit_strobe    (),
        .tx_en_out         (f3_tx_en_out),
        .tx_can_out        (f3_tx_can_out),
        .crc_err_pulse     (f3_crc_err_pulse),
        .stuff_err_pulse   (f3_stuff_err_pulse),
        .ack_err_pulse     (f3_ack_err_pulse),
        .tx_success_pulse  (f3_tx_success_pulse),
        .rx_success_pulse  (f3_rx_success_pulse),
        .recessive11_pulse (f3_recessive11_pulse)
    );

    // -------------------------------------------------------------------------
    // 6) Fault Confinement & Error Management (error_mgmt)
    // -------------------------------------------------------------------------
    error_mgmt em (
        .clk                    (clk),
        .rst_n                  (rst_n),
        .crc_err_pulse          (f3_crc_err_pulse),
        .stuff_err_pulse        (f3_stuff_err_pulse),
        .form_err_pulse         (1'b0),
        .bit_err_pulse          (bit_error_w),
        .ack_err_pulse          (f3_ack_err_pulse),
        .err_flag_bit_err_pulse (1'b0),
        .node_is_tx             (tx_en_sample_reg),
        .tx_success_pulse       (f3_tx_success_pulse),
        .rx_success_pulse       (f3_rx_success_pulse),
        .recessive11_pulse      (f3_recessive11_pulse),
        .tec                    (tec),
        .rec                    (rec),
        .err_active             (err_active),
        .err_passive            (err_passive),
        .bus_off                (bus_off),
        .ewarn                  (ewarn)
    );

    // Map fault states to a 2-bit error state status
    assign err_state = bus_off ? 2'b10 : (err_passive ? 2'b01 : 2'b00);
    assign msg_err   = f3_crc_err_pulse | f3_stuff_err_pulse | f3_ack_err_pulse | bit_error_w;

    // Pass-through signal to commit received packets
    assign accept_rxb0 = f3_rx_success_pulse;

    // -------------------------------------------------------------------------
    // 7) Output Transmission Drive Routing
    // -------------------------------------------------------------------------
    // Keep TX enable continuous across the fsm_part2 -> fsm_part3 CRC
    // handoff.  fsm_part2 releases tx_en as it enters CRC, while fsm_part3
    // takes one system clock to observe that state and assert its override.
    // The old mux therefore produced a one-clock low glitch in the middle of
    // every transmitted frame.
    wire tx_crc_phase = (current_state_w == `CAN_STATE_CRC) && txb_txreq;
    wire tx_en_mux    = tx_crc_phase || f3_tx_en_out || tx_en_fsm2;
    wire tx_can_base  = (tx_crc_phase || f3_tx_en_out) ? f3_tx_can_out
                                                       : tx_can_fsm2;
    wire tx_can_mux   = (insert_stuff && tx_en_mux) ? stuff_tx_bit : tx_can_base;

    assign tx_en          = tx_en_mux;
    assign tx_can         = tx_can_mux;
    assign tx_done_pulse  = f3_tx_success_pulse;
    assign rx_done_pulse  = f3_rx_success_pulse;

    // Sample tx_en on entering SOF state
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            prev_state_sof   <= 1'b0;
            tx_en_sample_reg <= 1'b0;
        end else begin
            if ((current_state_w == `CAN_STATE_SOF) && !prev_state_sof) begin
                tx_en_sample_reg <= tx_en_fsm2;
            end
            prev_state_sof <= (current_state_w == `CAN_STATE_SOF);
        end
    end

endmodule