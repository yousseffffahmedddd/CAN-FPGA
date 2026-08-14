`include "../common/can_defs.vh"

// =============================================================================
// Module : can_top
// -----------------------------------------------------------------------------
// SOW-Aligned Integrated Top-Level Module.
//
// Scope enforced:
//   - 1 TX Buffer, 1 RX Buffer
//   - 1 Acceptance Filter (RXF0), 1 Acceptance Mask (RXM0)
//   - Fixed Normal Operating Mode
//   - Full SPI Host CDC & Interrupt Line Management
//   - Direct Protocol Engine Interface Integration
// =============================================================================

module can_top (
    input  wire        clk,
    input  wire        rst_n,

    // SPI host interface
    input  wire        sck,
    input  wire        si,
    input  wire        cs_n,
    output wire        so,

    // Physical CAN bus pins
    input  wire        rx_pin,
    output wire        tx_can,
    output wire        tx_en,

    // Interrupt line
    output wire        int_n
);

    // -------------------------------------------------------------------------
    // SPI & CDC Internal Bus Declarations
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

    // -------------------------------------------------------------------------
    // Buffer, Control, & Status Wires
    // -------------------------------------------------------------------------
    wire [10:0] txb0_id;
    wire [63:0] txb0_data;
    wire [3:0]  txb0_dlc;
    wire        txb0_rtr;
    wire        txb0_txreq;

    wire [10:0] rxb0_id;
    wire [63:0] rxb0_data;
    wire [3:0]  rxb0_dlc;
    wire        rxb0_rtr;
    wire        rxb0_full;
    wire        rx0_ovr;

    wire [10:0] rxm0_mask;
    wire [10:0] rxf0_id;

    wire        pe_tx_done;
    wire        pe_rx_done;
    wire        pe_msg_err;
    wire        pe_bus_idle;
    wire [7:0]  tec;
    wire [7:0]  rec;
    wire [1:0]  err_state;

    wire        accept_rxb0;
    wire [10:0] filt_id_out;
    wire [63:0] filt_data_out;
    wire [3:0]  filt_dlc_out;
    wire        filt_rtr_out;

    wire        int_req_core;
    wire        int_n_sync;

    // Unused buffer status wires
    wire [10:0] unused_tx_buf_id;
    wire [63:0] unused_tx_buf_data;
    wire [3:0]  unused_tx_buf_dlc;
    wire        unused_tx_buf_rtr;
    wire        unused_tx_buf_ready;

    wire [10:0] unused_rx_buf_id;
    wire [63:0] unused_rx_buf_data;
    wire [3:0]  unused_rx_buf_dlc;
    wire        unused_rx_buf_rtr;

    // -------------------------------------------------------------------------
    // 1) SPI Host Interface
    // -------------------------------------------------------------------------
    spi_if u_spi_if (
        .clk           (clk),
        .rst_n         (rst_n),
        .sck           (sck),
        .si            (si),
        .so            (so),
        .cs_n          (cs_n),
        .addr          (spi_addr),
        .wdata         (spi_wdata),
        .rdata         (spi_rdata),
        .we            (spi_we),
        .reset_pulse   (spi_reset_pulse),
        .rts_pulse     (spi_rts_pulse),
        .rxbuf_done    (spi_rxbuf_done),
        .status_byte   (status_byte),
        .rxstatus_byte (rxstatus_byte)
    );

    // -------------------------------------------------------------------------
    // 2) CDC Synchronization
    // -------------------------------------------------------------------------
    cdc_handshake #(.DATA_WIDTH(8)) u_addr_cdc (
        .src_clk     (clk),
        .src_rst_n   (rst_n),
        .src_data    (spi_addr),
        .src_valid   (spi_we),
        .src_busy    (),
        .dst_clk     (clk),
        .dst_rst_n   (rst_n),
        .dst_data    (spi_addr_cdc),
        .valid_pulse (),
        .dst_ack     ()
    );

    cdc_handshake #(.DATA_WIDTH(8)) u_wdata_cdc (
        .src_clk     (clk),
        .src_rst_n   (rst_n),
        .src_data    (spi_wdata),
        .src_valid   (spi_we),
        .src_busy    (),
        .dst_clk     (clk),
        .dst_rst_n   (rst_n),
        .dst_data    (spi_wdata_cdc),
        .valid_pulse (),
        .dst_ack     ()
    );

    sync_2ff_edge u_we_cdc (
        .clk            (clk),
        .rst_n          (rst_n),
        .d              (spi_we),
        .sync_out       (spi_we_cdc),
        .posedge_pulse  (),
        .negedge_pulse  ()
    );

    sync_2ff_edge u_reset_cdc (
        .clk            (clk),
        .rst_n          (rst_n),
        .d              (spi_reset_pulse),
        .sync_out       (spi_reset_cdc),
        .posedge_pulse  (),
        .negedge_pulse  ()
    );

    sync_2ff_edge u_rts_cdc (
        .clk            (clk),
        .rst_n          (rst_n),
        .d              (spi_rts_pulse),
        .sync_out       (spi_rts_cdc),
        .posedge_pulse  (),
        .negedge_pulse  ()
    );

    sync_2ff_edge u_rxbuf_done_cdc (
        .clk            (clk),
        .rst_n          (rst_n),
        .d              (spi_rxbuf_done),
        .sync_out       (spi_rxbuf_done_cdc),
        .posedge_pulse  (),
        .negedge_pulse  ()
    );

    // -------------------------------------------------------------------------
    // 3) Trimmed SOW Control Register Bank
    // -------------------------------------------------------------------------
    reg_bank u_reg_bank (
        .clk           (clk),
        .rst_n         (rst_n),
        .addr          (spi_addr_cdc),
        .wdata         (spi_wdata_cdc),
        .we            (spi_we_cdc),
        .rdata         (spi_rdata),
        .reset_pulse   (spi_reset_cdc),
        .rts_pulse     (spi_rts_cdc),
        .rxbuf_done    (spi_rxbuf_done_cdc),
        .status_byte   (status_byte),
        .rxstatus_byte (rxstatus_byte),
        .int_n         (int_req_core),
        .bus_idle      (pe_bus_idle),
        .tec           (tec),
        .rec           (rec),
        .err_state     (err_state),
        .tx_done_pulse (pe_tx_done),
        .rx_done_pulse (pe_rx_done),
        .msg_err       (pe_msg_err),
        .opmod         (),
        .config_mode   (),
        .normal_mode   (),
        .txb0_id       (txb0_id),
        .txb0_data     (txb0_data),
        .txb0_dlc      (txb0_dlc),
        .txb0_rtr      (txb0_rtr),
        .txb0_txreq    (txb0_txreq),
        .rxb0_id       (rxb0_id),
        .rxb0_data     (rxb0_data),
        .rxb0_dlc      (rxb0_dlc),
        .rxb0_rtr      (rxb0_rtr),
        .rxb0_full     (rxb0_full),
        .rx0_ovr       (rx0_ovr),
        .rxm0_mask     (rxm0_mask),
        .rxf0_id       (rxf0_id)
    );

    // -------------------------------------------------------------------------
    // 4) Single Hardware TX Buffer
    // -------------------------------------------------------------------------
    tx_buffer u_tx_buffer (
        .clk          (clk),
        .reset        (~rst_n),
        .write_enable (spi_we_cdc && (spi_addr_cdc >= 8'h30) && (spi_addr_cdc <= 8'h3D)),
        .tx_id        (txb0_id),
        .tx_data      (txb0_data),
        .tx_dlc       (txb0_dlc),
        .tx_rtr       (txb0_rtr),
        .txreq        (txb0_txreq),
        .tx_done      (pe_tx_done),
        .id           (unused_tx_buf_id),
        .data         (unused_tx_buf_data),
        .dlc          (unused_tx_buf_dlc),
        .rtr          (unused_tx_buf_rtr),
        .ready        (unused_tx_buf_ready)
    );

    // -------------------------------------------------------------------------
    // 5) Top Protocol Engine Instance
    // -------------------------------------------------------------------------
    protocol_engine u_protocol_engine (
        .clk            (clk),
        .rst_n          (rst_n),
        .rx_pin         (rx_pin),
        .tx_can         (tx_can),
        .tx_en          (tx_en),
        .txb_id         (txb0_id),
        .txb_data       (txb0_data),
        .txb_dlc        (txb0_dlc),
        .txb_rtr        (txb0_rtr),
        .txb_txreq      (txb0_txreq),
        .rxb_id         (rxb0_id),
        .rxb_data       (rxb0_data),
        .rxb_dlc        (rxb0_dlc),
        .rxb_rtr        (rxb0_rtr),
        .accept_rxb0    (accept_rxb0),
        .tx_done_pulse  (pe_tx_done),
        .rx_done_pulse  (pe_rx_done),
        .msg_err        (pe_msg_err),
        .tec            (tec),
        .rec            (rec),
        .err_state      (err_state),
        .err_active     (),
        .err_passive    (),
        .bus_off        (),
        .ewarn          (),
        .bus_idle       (pe_bus_idle),
        .current_state  (),
        .latched_dlc    (),
        .tq_index       (),
        .rxm0_mask      (rxm0_mask),
        .rxf0_id        (rxf0_id)
    );

    // -------------------------------------------------------------------------
    // 6) Interrupt Line Synchronizer
    // -------------------------------------------------------------------------
    sync_2ff_edge u_int_cdc (
        .clk            (clk),
        .rst_n          (rst_n),
        .d              (int_req_core),
        .sync_out       (int_n_sync),
        .posedge_pulse  (),
        .negedge_pulse  ()
    );

    assign int_n = int_n_sync;

    // -------------------------------------------------------------------------
    // 7) Acceptance Filter & Hardware RX Buffer
    // -------------------------------------------------------------------------
    accept_filter u_accept_filter (
        .frame_valid (pe_rx_done),
        .rx_id_in    (rxb0_id),
        .rx_data_in  (rxb0_data),
        .rx_dlc_in   (rxb0_dlc),
        .rx_rtr_in   (rxb0_rtr),
        .filter_id   (rxf0_id),
        .filter_mask (rxm0_mask),
        .accept      (accept_rxb0),
        .rx_id_out   (filt_id_out),
        .rx_data_out (filt_data_out),
        .rx_dlc_out  (filt_dlc_out),
        .rx_rtr_out  (filt_rtr_out)
    );

    rx_buffer u_rx_buffer (
        .clk          (clk),
        .reset        (~rst_n),
        .write_enable (accept_rxb0),
        .rx_id        (filt_id_out),
        .rx_data      (filt_data_out),
        .rx_dlc       (filt_dlc_out),
        .rx_rtr       (filt_rtr_out),
        .cpu_read     (spi_rxbuf_done_cdc),
        .full         (rxb0_full),
        .id           (unused_rx_buf_id),
        .data         (unused_rx_buf_data),
        .dlc          (unused_rx_buf_dlc),
        .rtr          (unused_rx_buf_rtr)
    );

endmodule