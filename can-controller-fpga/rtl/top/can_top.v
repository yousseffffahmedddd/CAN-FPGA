`include "../common/can_defs.vh"

// =============================================================================
// Module : can_top
// -----------------------------------------------------------------------------
// Integrated reduced CAN controller:
//   - one TX buffer
//   - one RX buffer
//   - one acceptance filter and mask
//   - fixed Normal mode
// =============================================================================

module can_top (
    input  wire        clk,
    input  wire        rst_n,

    // SPI host interface
    input  wire        sck,
    input  wire        si,
    output wire        so,
    input  wire        cs_n,

    // Physical CAN bus pins
    input  wire        rx_pin,
    output wire        tx_can,
    output wire        tx_en,

    // Active-low interrupt line
    output wire        int_n
);

    // -------------------------------------------------------------------------
    // SPI register bus and command pulses
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

    // -------------------------------------------------------------------------
    // Register Bank <-> Control Logic
    // -------------------------------------------------------------------------
    wire [7:0] canctrl;
    wire [7:0] bfpctrl;
    wire [7:0] caninte;
    wire [7:0] canintf;

    wire       control_reset;
    wire       txreq_set;
    wire       txreq_clear;
    wire       set_rx0if;
    wire       clear_rx0if;
    wire       set_tx0if;
    wire       clear_tx0if;
    wire       set_merrf;
    wire       clear_merrf;
    wire       set_errif;
    wire       tx_start;
    wire [7:0] control_tec;
    wire [7:0] control_rec;
    wire       control_int_n;

    // Software RESET resets the internal controller blocks as well as the
    // register bank, while the SPI interface itself stays alive to receive it.
    wire core_rst_n = ~control_reset;

    // tx_buffer.v and rx_buffer.v use synchronous active-high reset inputs,
    // whereas rst_n/control_reset can change asynchronously to clk.  Hold a
    // registered reset through a clock edge so those two buffers can never miss
    // reset when rst_n is released on (or very near) a rising clock edge.
    reg buffer_reset_hold;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            buffer_reset_hold <= 1'b1;
        else if (spi_reset_pulse)
            buffer_reset_hold <= 1'b1;
        else
            buffer_reset_hold <= 1'b0;
    end

    // -------------------------------------------------------------------------
    // Register-bank TX staging and hardcoded filter/mask values
    // -------------------------------------------------------------------------
    wire [10:0] txb0_id;
    wire [63:0] txb0_data;
    wire [3:0]  txb0_dlc;
    wire        txb0_rtr;
    wire        txb0_txreq;
    wire        txb0_wr;

    wire [10:0] rxm0_mask;
    wire [10:0] rxf0_id;

    // -------------------------------------------------------------------------
    // Physical TX buffer outputs used by the Protocol Engine
    // -------------------------------------------------------------------------
    wire [10:0] txbuf_id;
    wire [63:0] txbuf_data;
    wire [3:0]  txbuf_dlc;
    wire        txbuf_rtr;
    wire        txbuf_ready;

    // -------------------------------------------------------------------------
    // Protocol Engine receive/status outputs
    // -------------------------------------------------------------------------
    wire [10:0] pe_rxb_id;
    wire [63:0] pe_rxb_data;
    wire [3:0]  pe_rxb_dlc;
    wire        pe_rxb_rtr;

    wire        pe_tx_done;
    wire        pe_rx_done;
    wire        pe_msg_err;
    wire [7:0]  pe_tec;
    wire [7:0]  pe_rec;
    wire        pe_err_passive;
    wire        pe_bus_off;

    // -------------------------------------------------------------------------
    // Acceptance filter and physical RX buffer
    // -------------------------------------------------------------------------
    wire        filter_accept;
    wire [10:0] filt_id_out;
    wire [63:0] filt_data_out;
    wire [3:0]  filt_dlc_out;
    wire        filt_rtr_out;

    wire [10:0] rxbuf_id;
    wire [63:0] rxbuf_data;
    wire [3:0]  rxbuf_dlc;
    wire        rxbuf_rtr;
    wire        rxbuf_full;

    // A receive interrupt is generated only when the accepted frame can
    // actually be committed into the single RX buffer.
    wire rx_store_event = filter_accept && !rxbuf_full;

    // The engine request is blocked during Bus-Off. tx_start launches a new
    // transfer, while txbuf_ready keeps the physical message pending for the
    // Protocol Engine's automatic retry path after an aborted transmission.
    // The visible TXREQ register bit may still clear on abort as required by
    // Module 4 without discarding the buffered frame.
    wire engine_txreq = (tx_start || txbuf_ready) && !pe_bus_off;

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
    // 2) Control Logic Interconnect
    // -------------------------------------------------------------------------
    control_logic u_control_logic (
        .clk             (clk),
        .rst_n           (rst_n),
        .rts_pulse       (spi_rts_pulse),
        .reset_pulse     (spi_reset_pulse),
        .rxbuf_done      (spi_rxbuf_done),
        .canctrl         (canctrl),
        .bfpctrl         (bfpctrl),
        .caninte         (caninte),
        .canintf         (canintf),
        .txreq           (txb0_txreq),
        .tx_done         (pe_tx_done),
        .tx_abort        (pe_msg_err),
        .rx_success      (rx_store_event),
        .eng_tec         (pe_tec),
        .eng_rec         (pe_rec),
        .eng_err_passive (pe_err_passive),
        .eng_bus_off     (pe_bus_off),
        .rx_full         (rxbuf_full),
        .rx_rtr          (rxbuf_rtr),
        .rx_ide          (1'b0),
        .rx_filter_hit   (4'h0),
        .control_reset   (control_reset),
        .txreq_set       (txreq_set),
        .txreq_clear     (txreq_clear),
        .set_rx0if       (set_rx0if),
        .clear_rx0if     (clear_rx0if),
        .set_tx0if       (set_tx0if),
        .clear_tx0if     (clear_tx0if),
        .set_merrf       (set_merrf),
        .clear_merrf     (clear_merrf),
        .set_errif       (set_errif),
        .tx_start        (tx_start),
        .status_byte     (status_byte),
        .rxstatus_byte   (rxstatus_byte),
        .tec             (control_tec),
        .rec             (control_rec),
        .eflg            (),
        .int_pin         (control_int_n)
    );

    assign int_n = control_int_n;

    // -------------------------------------------------------------------------
    // 3) Control & Interrupt Register Bank
    // -------------------------------------------------------------------------
    reg_bank u_reg_bank (
        .clk           (clk),
        .rst_n         (rst_n),
        .addr          (spi_addr),
        .wdata         (spi_wdata),
        .we            (spi_we),
        .rdata         (spi_rdata),
        .control_reset (control_reset),
        .txreq_set     (txreq_set),
        .txreq_clear   (txreq_clear),
        .set_rx0if     (set_rx0if),
        .clear_rx0if   (clear_rx0if),
        .set_tx0if     (set_tx0if),
        .clear_tx0if   (clear_tx0if),
        .set_merrf     (set_merrf),
        .clear_merrf   (clear_merrf),
        .set_errif     (set_errif),
        .tec_in        (control_tec),
        .rec_in        (control_rec),
        .canctrl       (canctrl),
        .bfpctrl       (bfpctrl),
        .caninte       (caninte),
        .canintf       (canintf),
        .opmod         (),
        .config_mode   (),
        .normal_mode   (),
        .txb0_id       (txb0_id),
        .txb0_data     (txb0_data),
        .txb0_dlc      (txb0_dlc),
        .txb0_rtr      (txb0_rtr),
        .txb0_txreq    (txb0_txreq),
        .txb0_wr       (txb0_wr),
        .rxb0_id       (rxbuf_id),
        .rxb0_data     (rxbuf_data),
        .rxb0_dlc      (rxbuf_dlc),
        .rxb0_rtr      (rxbuf_rtr),
        .rxm0_mask     (rxm0_mask),
        .rxf0_id       (rxf0_id)
    );

    // -------------------------------------------------------------------------
    // 4) Single TX Buffer
    // -------------------------------------------------------------------------
    tx_buffer u_tx_buffer (
        .clk          (clk),
        .reset        (buffer_reset_hold),
        .write_enable (txb0_wr),
        .tx_id        (txb0_id),
        .tx_data      (txb0_data),
        .tx_dlc       (txb0_dlc),
        .tx_rtr       (txb0_rtr),
        .txreq        (tx_start),
        .tx_done      (pe_tx_done),
        .id           (txbuf_id),
        .data         (txbuf_data),
        .dlc          (txbuf_dlc),
        .rtr          (txbuf_rtr),
        .ready        (txbuf_ready)
    );

    // -------------------------------------------------------------------------
    // 5) Protocol Engine
    // -------------------------------------------------------------------------
    protocol_engine u_protocol_engine (
        .clk            (clk),
        .rst_n          (core_rst_n),
        .rx_pin         (rx_pin),
        .tx_can         (tx_can),
        .tx_en          (tx_en),
        .txb_id         (txbuf_id),
        .txb_data       (txbuf_data),
        .txb_dlc        (txbuf_dlc),
        .txb_rtr        (txbuf_rtr),
        .txb_txreq      (engine_txreq),
        .rxb_id         (pe_rxb_id),
        .rxb_data       (pe_rxb_data),
        .rxb_dlc        (pe_rxb_dlc),
        .rxb_rtr        (pe_rxb_rtr),
        .accept_rxb0    (),
        .tx_done_pulse  (pe_tx_done),
        .rx_done_pulse  (pe_rx_done),
        .msg_err        (pe_msg_err),
        .tec            (pe_tec),
        .rec            (pe_rec),
        .err_state      (),
        .err_active     (),
        .err_passive    (pe_err_passive),
        .bus_off        (pe_bus_off),
        .ewarn          (),
        .bus_idle       (),
        .current_state  (),
        .latched_dlc    (),
        .tq_index       (),
        .rxm0_mask      (rxm0_mask),
        .rxf0_id        (rxf0_id)
    );

    // -------------------------------------------------------------------------
    // 6) Single Acceptance Filter
    // -------------------------------------------------------------------------
    accept_filter u_accept_filter (
        .frame_valid (pe_rx_done),
        .rx_id_in    (pe_rxb_id),
        .rx_data_in  (pe_rxb_data),
        .rx_dlc_in   (pe_rxb_dlc),
        .rx_rtr_in   (pe_rxb_rtr),
        .filter_id   (rxf0_id),
        .filter_mask (rxm0_mask),
        .accept      (filter_accept),
        .rx_id_out   (filt_id_out),
        .rx_data_out (filt_data_out),
        .rx_dlc_out  (filt_dlc_out),
        .rx_rtr_out  (filt_rtr_out)
    );

    // -------------------------------------------------------------------------
    // 7) Single RX Buffer
    // -------------------------------------------------------------------------
    rx_buffer u_rx_buffer (
        .clk          (clk),
        .reset        (buffer_reset_hold),
        .write_enable (filter_accept),
        .rx_id        (filt_id_out),
        .rx_data      (filt_data_out),
        .rx_dlc       (filt_dlc_out),
        .rx_rtr       (filt_rtr_out),
        .cpu_read     (spi_rxbuf_done),
        .full         (rxbuf_full),
        .id           (rxbuf_id),
        .data         (rxbuf_data),
        .dlc          (rxbuf_dlc),
        .rtr          (rxbuf_rtr)
    );

endmodule
