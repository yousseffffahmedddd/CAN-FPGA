`include "../common/can_defs.vh"

// =============================================================================
// can_top.v
// -----------------------------------------------------------------------------
// SOW-aligned integration for the current codebase.
//
// Scope enforced here:
//   - one TX buffer
//   - one RX buffer
//   - one acceptance filter
//   - one acceptance mask
//   - Normal Mode only
//   - no configuration-mode or extended-ID support
//   - no BIT MODIFY instruction path
//
// The external host interface is the SPI path already implemented by spi_if.v.
// The protocol bus interface is the protocol_engine.v wrapper.
//
// This top-level wrapper intentionally avoids inventing undocumented packet-level
// RX validation handshakes that are not present in the current RTL set.
// =============================================================================

module can_top (
    input  wire        clk,
    input  wire        rst_n,

    // SPI host interface
    input  wire        sck,
    input  wire        si,
    input  wire        cs_n,
    output wire        so,

    // Physical CAN bus
    input  wire        rx_pin,
    output wire        tx_can,
    output wire        tx_en,

    // Interrupt line
    output wire        int_n
);

    // -------------------------------------------------------------------------
    // SPI / Register-bus signals
    // -------------------------------------------------------------------------
    wire [7:0] spi_addr;
    wire [7:0] spi_wdata;
    wire [7:0] spi_rdata;
    wire       spi_we;
    wire       spi_reset_pulse;
    wire       spi_rts_pulse;
    wire       spi_rxbuf_done;
    wire [7:0] status_byte;
    wire [7:0] rxstatus_byte;

    wire [7:0] spi_addr_cdc;
    wire [7:0] spi_wdata_cdc;
    wire       spi_we_cdc;
    wire       spi_reset_cdc;
    wire       spi_rts_cdc;
    wire       spi_rxbuf_done_cdc;

    wire       tx_done_core_pulse;
    wire       rx_done_core_pulse;
    wire       int_req_core;
    wire       int_n_sync;
    wire [10:0] zero_11;
    wire        zero_1;

    // -------------------------------------------------------------------------
    // Register-bank outputs used by the SOW scope controller
    // -------------------------------------------------------------------------
    wire [2:0] opmod;
    wire       config_mode;
    wire       normal_mode;
    wire [1:0] err_state;
    wire [7:0] tec;
    wire [7:0] rec;
    wire [2:0] tx_done_vec;
    wire       msg_err;
    wire [2:0] txreq_clr;

    // Single-buffer view only; the original reg_bank.v still exposes the larger
    // MCP2515-style bus, but the SOW top-level scope intentionally narrows it to
    // one TX buffer, one RX buffer, one filter, and one mask.
    wire [32:0] txb_id;
    wire [191:0] txb_data;
    wire [11:0] txb_dlc;
    wire [2:0] txb_rtr;
    wire [2:0] txb_wr;
    wire [2:0] txb_txreq;
    wire [2:0] txb_ready;

    wire [21:0] rxb_id;
    wire [127:0] rxb_data;
    wire [7:0] rxb_dlc;
    wire [1:0] rxb_rtr;
    wire [1:0] rxb_full;
    wire [1:0] rxb_cpu_read;
    wire [1:0] rx_ovr;

    wire [10:0] rxf0_id;
    wire [10:0] rxm0_mask;
    wire        accept_rxb0;

    // -------------------------------------------------------------------------
    // Protocol engine interface
    // -------------------------------------------------------------------------
    wire [7:0]  pe_tec;
    wire [7:0]  pe_rec;
    wire        pe_bus_idle;
    wire [2:0]  pe_current_state;
    wire [3:0]  pe_dlc;
    wire        pe_tx_can;
    wire        pe_tx_en;
    wire        pe_sipo_valid;
    wire        pe_piso_req;
    wire        pe_tx_done;
    wire        pe_rx_done;
    reg         piso_data_in_r;
    reg         piso_valid_r;
    reg         tx_piso_active;
    reg [127:0] tx_piso_frame;
    reg [7:0]   tx_piso_bit_count;
    wire        pe_sipo_data;
    reg [78:0] rx_shift_reg;
    reg [6:0]  rx_bit_cnt;
    reg        rx_frame_valid_r;
    wire        unused_pe_err_active;
    wire        unused_pe_err_passive;
    wire        unused_pe_bus_off;
    wire        unused_pe_ewarn;
    wire [4:0]  unused_pe_tq_index;
    wire [10:0] unused_tx_buf_id_lo;
    wire [63:0] unused_tx_buf_data_lo;
    wire [3:0]  unused_tx_buf_dlc_lo;
    wire        unused_tx_buf_rtr_lo;
    wire [10:0] unused_rx_buf_id;
    wire [63:0] unused_rx_buf_data;
    wire [3:0]  unused_rx_buf_dlc;
    wire        unused_rx_buf_rtr;

    // -------------------------------------------------------------------------
    // RX receive-path mapping bridged from the protocol engine SIPO output.
    // These signals are assembled from the actual serial receive stream and are
    // allowed to become active once the protocol engine signals a valid RX
    // completion pulse.
    // -------------------------------------------------------------------------
    reg [10:0] rx_frame_id;
    reg [63:0] rx_frame_data;
    reg [3:0]  rx_frame_dlc;
    reg        rx_frame_valid;
    reg        rx_frame_rtr;

    // -------------------------------------------------------------------------
    // 1) SPI interface to host
    // -------------------------------------------------------------------------
    spi_if u_spi_if (
        .clk          (clk),
        .rst_n        (rst_n),
        .sck          (sck),
        .si           (si),
        .so           (so),
        .cs_n         (cs_n),
        .addr         (spi_addr),
        .wdata        (spi_wdata),
        .rdata        (spi_rdata),
        .we           (spi_we),
        .reset_pulse  (spi_reset_pulse),
        .rts_pulse    (spi_rts_pulse),
        .rxbuf_done   (spi_rxbuf_done),
        .status_byte  (status_byte),
        .rxstatus_byte(rxstatus_byte)
    );

    // -------------------------------------------------------------------------
    // 2) CDC handshake for SPI register access into the 1 MHz CAN core domain
    // -------------------------------------------------------------------------
    cdc_handshake #(.DATA_WIDTH(8)) u_addr_cdc (
        .src_clk      (clk),
        .src_rst_n    (rst_n),
        .src_data     (spi_addr),
        .src_valid    (spi_we),
        .src_busy     (),
        .dst_clk      (clk),
        .dst_rst_n    (rst_n),
        .dst_data     (spi_addr_cdc),
        .valid_pulse  (),
        .dst_ack      ()
    );

    cdc_handshake #(.DATA_WIDTH(8)) u_wdata_cdc (
        .src_clk      (clk),
        .src_rst_n    (rst_n),
        .src_data     (spi_wdata),
        .src_valid    (spi_we),
        .src_busy     (),
        .dst_clk      (clk),
        .dst_rst_n    (rst_n),
        .dst_data     (spi_wdata_cdc),
        .valid_pulse  (),
        .dst_ack      ()
    );

    sync_2ff_edge u_we_cdc (
        .clk       (clk),
        .rst_n     (rst_n),
        .d         (spi_we),
        .sync_out  (spi_we_cdc),
        .posedge_pulse(),
        .negedge_pulse()
    );

    sync_2ff_edge u_reset_cdc (
        .clk       (clk),
        .rst_n     (rst_n),
        .d         (spi_reset_pulse),
        .sync_out  (spi_reset_cdc),
        .posedge_pulse(),
        .negedge_pulse()
    );

    sync_2ff_edge u_rts_cdc (
        .clk       (clk),
        .rst_n     (rst_n),
        .d         (spi_rts_pulse),
        .sync_out  (spi_rts_cdc),
        .posedge_pulse(),
        .negedge_pulse()
    );

    sync_2ff_edge u_rxbuf_done_cdc (
        .clk       (clk),
        .rst_n     (rst_n),
        .d         (spi_rxbuf_done),
        .sync_out  (spi_rxbuf_done_cdc),
        .posedge_pulse(),
        .negedge_pulse()
    );

    assign zero_11 = 11'd0;
    assign zero_1  = 1'b0;

    reg_bank u_reg_bank (
        .clk           (clk),
        .rst_n         (rst_n),
        .addr          (spi_addr_cdc),
        .wdata         (spi_wdata_cdc),
        .we            (spi_we_cdc),
        .rdata         (spi_rdata),
        .bitmod_mask   (8'h00),
        .bitmod_we     (1'b0),
        .reset_pulse   (spi_reset_cdc),
        .rts_pulse     ({2'b00, spi_rts_cdc}),
        .rxbuf_done    (spi_rxbuf_done_cdc),
        .rxbuf_sel     (1'b0),
        .status_byte   (status_byte),
        .rxstatus_byte (rxstatus_byte),
        .int_n         (int_req_core),
        .bus_idle      (pe_bus_idle),
        .tec           (tec),
        .rec           (rec),
        .err_state     (err_state),
        .tx_done       (tx_done_vec),
        .msg_err       (msg_err),
        .opmod         (opmod),
        .config_mode   (config_mode),
        .normal_mode   (normal_mode),
        .txreq_clr     (txreq_clr),
        .txb_id        (txb_id),
        .txb_data      (txb_data),
        .txb_dlc       (txb_dlc),
        .txb_rtr       (txb_rtr),
        .txb_wr        (txb_wr),
        .txb_txreq     (txb_txreq),
        .txb_ready     (txb_ready),
        .rxb_id        (rxb_id),
        .rxb_data      (rxb_data),
        .rxb_dlc       (rxb_dlc),
        .rxb_rtr       (rxb_rtr),
        .rxb_full      (rxb_full),
        .rx_ovr        (rx_ovr),
        .rxb_cpu_read  (rxb_cpu_read),
        .rxm0_mask     (rxm0_mask),
        .rxf0_id       (rxf0_id),
        .rxf1_id       (zero_11),
        .rxb0_accept_all(zero_1),
        .bukt          (zero_1),
        .rxm1_mask     (zero_11),
        .rxf2_id       (zero_11),
        .rxf3_id       (zero_11),
        .rxf4_id       (zero_11),
        .rxf5_id       (zero_11),
        .rxb1_accept_all(zero_1),
        .filhit0       (1'b0),
        .filhit1       (3'b000),
        .accept_rxb0   (accept_rxb0),
        .accept_rxb1   (1'b0)
    );

    // -------------------------------------------------------------------------
    // 3) Protocol engine and bus output
    // -------------------------------------------------------------------------
    protocol_engine u_protocol_engine (
        .clk          (clk),
        .rst_n        (rst_n),
        .rx_pin       (rx_pin),
        .tx_can       (pe_tx_can),
        .tx_en        (pe_tx_en),
        .piso_data_in (piso_data_in_r),
        .piso_valid   (piso_valid_r),
        .piso_req     (pe_piso_req),
        .sipo_data_out(pe_sipo_data),
        .sipo_valid   (pe_sipo_valid),
        .tec          (pe_tec),
        .rec          (pe_rec),
        .err_active   (unused_pe_err_active),
        .err_passive  (unused_pe_err_passive),
        .bus_off      (unused_pe_bus_off),
        .ewarn        (unused_pe_ewarn),
        .tx_done_pulse(pe_tx_done),
        .rx_done_pulse(pe_rx_done),
        .tq_index     (unused_pe_tq_index),
        .bus_idle     (pe_bus_idle),
        .current_state(pe_current_state),
        .latched_dlc  (pe_dlc)
    );

    sync_2ff_edge u_int_cdc (
        .clk       (clk),
        .rst_n     (rst_n),
        .d         (int_req_core),
        .sync_out  (int_n_sync),
        .posedge_pulse (),
        .negedge_pulse ()
    );

    sync_2ff_edge u_tx_done_cdc (
        .clk          (clk),
        .rst_n        (rst_n),
        .d            (pe_tx_done),
        .sync_out     (),
        .posedge_pulse(tx_done_core_pulse),
        .negedge_pulse()
    );

    sync_2ff_edge u_rx_done_cdc (
        .clk          (clk),
        .rst_n        (rst_n),
        .d            (pe_rx_done),
        .sync_out     (),
        .posedge_pulse(rx_done_core_pulse),
        .negedge_pulse()
    );

    assign tx_can = pe_tx_can;
    assign tx_en  = pe_tx_en;
    assign int_n  = int_n_sync;

    // -------------------------------------------------------------------------
    // 4) Single TX buffer in the SOW architecture
    // -------------------------------------------------------------------------
    wire [10:0] tx_buf_id   = txb_id[10:0];
    wire [63:0] tx_buf_data = txb_data[63:0];
    wire [3:0]  tx_buf_dlc  = txb_dlc[3:0];
    wire        tx_buf_rtr  = txb_rtr[0];

    tx_buffer u_tx_buffer (
        .clk         (clk),
        .reset       (~rst_n),
        .write_enable(txb_wr[0]),
        .tx_id       (tx_buf_id),
        .tx_data     (tx_buf_data),
        .tx_dlc      (tx_buf_dlc),
        .tx_rtr      (tx_buf_rtr),
        .txreq       (txb_txreq[0]),
        .tx_done     (pe_tx_done),
        .id          (unused_tx_buf_id_lo),
        .data        (unused_tx_buf_data_lo),
        .dlc         (unused_tx_buf_dlc_lo),
        .rtr         (unused_tx_buf_rtr_lo),
        .ready       (txb_ready[0])
    );

    // -------------------------------------------------------------------------
    // TX -> PISO bridge for the SOW controller.
    // The protocol engine expects a bit-by-bit transmit stream. The SOW can
    // support a minimal but realistic standard-ID frame layout here:
    //   SOF + ID[10:0] + RTR + IDE + r0 + DLC[3:0] + DATA[63:0]
    // CRC/ACK/EOF are not implemented in this top-level wrapper because the
    // protocol engine's own FSM stages do not yet expose a complete hardware
    // CRC/ACK path at the top edge.
    // -------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx_piso_active    <= 1'b0;
            tx_piso_frame     <= 128'b0;
            tx_piso_bit_count <= 8'd0;
            piso_data_in_r    <= 1'b0;
            piso_valid_r      <= 1'b0;
            rx_shift_reg      <= 79'b0;
            rx_bit_cnt        <= 7'd0;
            rx_frame_valid_r  <= 1'b0;
            rx_frame_id       <= 11'b0;
            rx_frame_data     <= 64'b0;
            rx_frame_dlc      <= 4'b0;
            rx_frame_valid    <= 1'b0;
            rx_frame_rtr      <= 1'b0;
        end else begin
            piso_valid_r <= 1'b0;
            rx_frame_valid_r <= 1'b0;

            if (!tx_piso_active && txb_txreq[0]) begin
                tx_piso_active    <= 1'b1;
                tx_piso_bit_count <= 8'd0;
                tx_piso_frame     <= {
                    tx_buf_data[63:0],
                    tx_buf_dlc[3:0],
                    1'b0,
                    1'b0,
                    tx_buf_rtr,
                    tx_buf_id[10:0],
                    1'b0
                };
            end else if (tx_piso_active && pe_piso_req) begin
                if (tx_piso_bit_count < 8'd83) begin
                    piso_data_in_r <= tx_piso_frame[127 - tx_piso_bit_count];
                    piso_valid_r   <= 1'b1;
                    tx_piso_bit_count <= tx_piso_bit_count + 8'd1;
                end else begin
                    tx_piso_active <= 1'b0;
                    piso_data_in_r <= 1'b0;
                    piso_valid_r   <= 1'b0;
                end
            end

            if (pe_sipo_valid) begin
                rx_shift_reg <= {rx_shift_reg[77:0], pe_sipo_data};
                rx_bit_cnt   <= rx_bit_cnt + 7'd1;
            end

            if (pe_rx_done) begin
                rx_frame_valid_r <= 1'b1;
                rx_frame_id      <= rx_shift_reg[78:68];
                rx_frame_dlc     <= rx_shift_reg[67:64];
                rx_frame_data    <= rx_shift_reg[63:0];
                rx_frame_rtr     <= 1'b0;
                rx_bit_cnt       <= 7'd0;
            end

            rx_frame_valid <= rx_frame_valid_r;
        end
    end

    // -------------------------------------------------------------------------
    // 5) Single RX buffer + acceptance filter + acceptance mask per SOW
    // -------------------------------------------------------------------------
    accept_filter u_accept_filter (
        .frame_valid (rx_frame_valid),
        .rx_id_in    (rx_frame_id),
        .rx_data_in  (rx_frame_data),
        .rx_dlc_in   (rx_frame_dlc),
        .rx_rtr_in   (rx_frame_rtr),
        .filter_id   (rxf0_id),
        .filter_mask (rxm0_mask),
        .accept      (accept_rxb0),
        .rx_id_out   (rxb_id[10:0]),
        .rx_data_out (rxb_data[63:0]),
        .rx_dlc_out  (rxb_dlc[3:0]),
        .rx_rtr_out  (rxb_rtr[0])
    );

    rx_buffer u_rx_buffer (
        .clk         (clk),
        .reset       (~rst_n),
        .write_enable(accept_rxb0),
        .rx_id       (rxb_id[10:0]),
        .rx_data     (rxb_data[63:0]),
        .rx_dlc      (rxb_dlc[3:0]),
        .rx_rtr      (rxb_rtr[0]),
        .cpu_read    (rxb_cpu_read[0]),
        .full        (rxb_full[0]),
        .id          (unused_rx_buf_id),
        .data        (unused_rx_buf_data),
        .dlc         (unused_rx_buf_dlc),
        .rtr         (unused_rx_buf_rtr)
    );

    // -------------------------------------------------------------------------
    // 6) SOW state and default values
    // -------------------------------------------------------------------------
    // No configuration-mode support, no extended-ID support, and no BIT MODIFY
    // instruction path are implemented in this top-level wrapper. The controller
    // is treated as a fixed Normal Mode device, which matches the SOW scope.
    assign tx_done_vec = {2'b00, tx_done_core_pulse};
    assign msg_err     = 1'b0;
    assign pe_tec      = tec;
    assign pe_rec      = rec;

endmodule
