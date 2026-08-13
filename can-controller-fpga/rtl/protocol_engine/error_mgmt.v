`include "../common/can_defs.vh"

// =============================================================================
// Module      : error_mgmt
// Description : Fault-confinement state machine for the CAN Protocol Engine.
//               Tracks TEC/REC, active/passive/bus-off state, and warning bits.
// =============================================================================

module error_mgmt (
    input  wire       clk,
    input  wire       rst_n,

    input  wire       crc_err_pulse,
    input  wire       stuff_err_pulse,
    input  wire       form_err_pulse,
    input  wire       bit_err_pulse,
    input  wire       ack_err_pulse,
    input  wire       err_flag_bit_err_pulse,

    input  wire       node_is_tx,
    input  wire       tx_success_pulse,
    input  wire       rx_success_pulse,
    input  wire       recessive11_pulse,

    output reg  [7:0] tec,
    output reg  [7:0] rec,
    output wire       err_active,
    output wire       err_passive,
    output wire       bus_off,
    output wire       ewarn
);

    localparam [7:0] TEC_TX_ERR            = 8'd8;
    localparam [7:0] REC_RX_ERR            = 8'd1;
    localparam [7:0] WARN_LIMIT            = 8'd96;
    localparam [7:0] PASSIVE_LIMIT         = 8'd128;
    localparam [7:0] BUSOFF_LIMIT          = 8'd255;
    localparam [7:0] BUSOFF_RECOVERY_LIMIT = 8'd128;

    reg        bus_off_q;
    reg [7:0]  recovery_count;
    reg [7:0]  tec_next;
    reg [7:0]  rec_next;
    reg        crc_err_prev;
    reg        stuff_err_prev;
    reg        form_err_prev;
    reg        bit_err_prev;
    reg        ack_err_prev;
    reg        flag_err_prev;

    wire       crc_err_event = crc_err_pulse && !crc_err_prev;
    wire       stuff_err_event = stuff_err_pulse && !stuff_err_prev;
    wire       form_err_event = form_err_pulse && !form_err_prev;
    wire       bit_err_event = bit_err_pulse && !bit_err_prev;
    wire       ack_err_event = ack_err_pulse && !ack_err_prev;
    wire       flag_err_event = err_flag_bit_err_pulse && !flag_err_prev;

    // Level latch: count a continuous error-level only once until it clears
    reg        err_level_latched;
    wire       err_level = crc_err_pulse || stuff_err_pulse || form_err_pulse || bit_err_pulse;
    wire       tx_err_event = err_level && !err_level_latched && node_is_tx;
    wire       rx_err_event = err_level && !err_level_latched && !node_is_tx;

    assign err_active = !bus_off_q && (tec < PASSIVE_LIMIT) && (rec < PASSIVE_LIMIT);
    assign err_passive = !bus_off_q && ((tec >= PASSIVE_LIMIT) || (rec >= PASSIVE_LIMIT));
    assign bus_off = bus_off_q;
    assign ewarn = !bus_off_q && ((tec >= WARN_LIMIT) || (rec >= WARN_LIMIT));

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tec <= 8'd0;
            rec <= 8'd0;
            bus_off_q <= 1'b0;
            recovery_count <= 8'd0;
            crc_err_prev <= 1'b0;
            stuff_err_prev <= 1'b0;
            form_err_prev <= 1'b0;
            bit_err_prev <= 1'b0;
            ack_err_prev <= 1'b0;
            flag_err_prev <= 1'b0;
            err_level_latched <= 1'b0;
        end
        else begin
            tec_next = tec;
            rec_next = rec;

            if (!bus_off_q) begin
                if (tx_err_event) begin
                    $display("TX_ERR_EVENT @%0t tec=%0d crc=%b prev=%b node=%b", $time, tec, crc_err_pulse, crc_err_prev, node_is_tx);
                    if (tec < 8'd248)
                        tec_next = tec + 8'd8;
                    else
                        tec_next = 8'd255;
                    if (tec_next == 8'd255)
                        bus_off_q <= 1'b1;
                end
                else if (rx_err_event) begin
                    if (rec < 8'd255)
                        rec_next = rec + 8'd1;
                    else
                        rec_next = 8'd255;
                end

                if (ack_err_event && node_is_tx && (tec < PASSIVE_LIMIT) && (rec < PASSIVE_LIMIT)) begin
                    if (tec < 8'd248)
                        tec_next = tec + 8'd8;
                    else
                        tec_next = 8'd255;
                    if (tec_next == 8'd255)
                        bus_off_q <= 1'b1;
                end

                if (flag_err_event && node_is_tx) begin
                    if (tec < 8'd248)
                        tec_next = tec + 8'd8;
                    else
                        tec_next = 8'd255;
                    if (tec_next == 8'd255)
                        bus_off_q <= 1'b1;
                end

                if (tx_success_pulse && (tec > 8'd0))
                    tec_next = tec - 8'd1;

                if (rx_success_pulse && (rec > 8'd0))
                    rec_next = rec - 8'd1;
            end

            if (bus_off_q && recessive11_pulse) begin
                recovery_count <= recovery_count + 8'd1;
                if (recovery_count >= (BUSOFF_RECOVERY_LIMIT - 8'd1)) begin
                    bus_off_q <= 1'b0;
                    recovery_count <= 8'd0;
                    tec_next = 8'd0;
                    rec_next = 8'd0;
                end
            end

            tec <= tec_next;
            rec <= rec_next;

            crc_err_prev <= crc_err_pulse;
            stuff_err_prev <= stuff_err_pulse;
            form_err_prev <= form_err_pulse;
            bit_err_prev <= bit_err_pulse;
            ack_err_prev <= ack_err_pulse;
            flag_err_prev <= err_flag_bit_err_pulse;
            // latch level until error-level clears
            err_level_latched <= err_level ? 1'b1 : 1'b0;
        end
    end

endmodule
