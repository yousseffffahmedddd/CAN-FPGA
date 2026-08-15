`include "../common/can_defs.vh"

// =============================================================================
// Module : control_logic
// -----------------------------------------------------------------------------
// Module 4 control/interconnect logic for the reduced CAN controller.
// Register storage is kept in reg_bank; this module generates the hardware
// control events, status bytes, error-state indications, and interrupt output.
// =============================================================================

module control_logic (
    input  wire        clk,
    input  wire        rst_n,

    // SPI / host command events
    input  wire        rts_pulse,
    input  wire        reset_pulse,
    input  wire        rxbuf_done,

    // Current register values from Register Bank
    input  wire [7:0]  canctrl,
    input  wire [7:0]  bfpctrl,
    input  wire [7:0]  caninte,
    input  wire [7:0]  canintf,
    input  wire        txreq,

    // Protocol Engine status
    input  wire        tx_done,
    input  wire        tx_abort,
    input  wire        rx_success,
    input  wire [7:0]  eng_tec,
    input  wire [7:0]  eng_rec,
    input  wire        eng_err_passive,
    input  wire        eng_bus_off,

    // Receive Buffer status
    input  wire        rx_full,
    input  wire        rx_rtr,
    input  wire        rx_ide,
    input  wire [3:0]  rx_filter_hit,

    // Hardware event strobes to Register Bank
    output wire        control_reset,
    output wire        txreq_set,
    output wire        txreq_clear,
    output wire        set_rx0if,
    output wire        clear_rx0if,
    output wire        set_tx0if,
    output wire        clear_tx0if,
    output wire        set_merrf,
    output wire        clear_merrf,
    output wire        set_errif,

    // One-cycle transmission request to the Protocol Engine
    output wire        tx_start,

    // Quick-status outputs to SPI
    output wire [7:0]  status_byte,
    output wire [7:0]  rxstatus_byte,

    // Error-counter / error-state mirrors
    output wire [7:0]  tec,
    output wire [7:0]  rec,
    output wire [7:0]  eflg,

    // Physical interrupt output, active low
    output wire        int_pin
);

    localparam integer RX0IF = 0;
    localparam integer TX0IF = 2;

    // CANCTRL remains locked to Normal mode in reg_bank and BFPCTRL has no
    // physical pin function in this reduced design. They remain interface
    // inputs for the Module 4 / Module 5 boundary.

    // -------------------------------------------------------------------------
    // Reset and error-state transition monitoring
    // -------------------------------------------------------------------------
    assign control_reset = (~rst_n) | reset_pulse;
    wire control_active = rst_n && !reset_pulse;

    // Module 4 also checks the passive threshold directly from TEC/REC while
    // accepting the Protocol Engine state output as the authoritative source.
    wire error_passive_status =
        !eng_bus_off &&
        (eng_err_passive || (eng_tec >= 8'd128) || (eng_rec >= 8'd128));

    reg err_passive_d;
    reg bus_off_d;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            err_passive_d <= 1'b0;
            bus_off_d     <= 1'b0;
        end else if (reset_pulse) begin
            err_passive_d <= 1'b0;
            bus_off_d     <= 1'b0;
        end else begin
            err_passive_d <= error_passive_status;
            bus_off_d     <= eng_bus_off;
        end
    end

    wire error_state_entry =
        (error_passive_status && !err_passive_d) ||
        (eng_bus_off          && !bus_off_d);

    // -------------------------------------------------------------------------
    // TXREQ management
    // -------------------------------------------------------------------------
    // RTS is rejected while Bus-Off. The SOW requires TXREQ to clear after
    // either successful completion or an abort/error indication.
    assign txreq_set   = control_active && rts_pulse && !eng_bus_off;
    assign tx_start    = control_active && rts_pulse && !eng_bus_off;
    assign txreq_clear = control_active && (tx_done || tx_abort);

    // -------------------------------------------------------------------------
    // Interrupt-flag hardware events
    // -------------------------------------------------------------------------
    assign set_tx0if    = control_active && tx_done;
    assign clear_tx0if  = 1'b0;

    assign set_rx0if    = control_active && rx_success;
    assign clear_rx0if  = control_active && rxbuf_done;

    assign set_merrf    = control_active && tx_abort;
    assign clear_merrf  = 1'b0;

    // ERRIF is raised for a protocol error and when the controller newly
    // enters Error-Passive or Bus-Off.
    assign set_errif = control_active && (tx_abort || error_state_entry);

    // -------------------------------------------------------------------------
    // Error counter and state visibility
    // -------------------------------------------------------------------------
    assign tec = eng_tec;
    assign rec = eng_rec;

    // Reduced EFLG visibility used by this project:
    // bit 7 = Bus-Off, bit 6 = Error-Passive.
    assign eflg = {
        eng_bus_off,
        error_passive_status,
        6'b000000
    };

    // -------------------------------------------------------------------------
    // SPI READ STATUS
    // -------------------------------------------------------------------------
    // Single-buffer subset of the MCP2515 READ STATUS layout:
    // {TX2IF, TX2REQ, TX1IF, TX1REQ, TX0IF, TX0REQ, RX1IF, RX0IF}.
    // Unsupported TX1/TX2/RX1 fields are tied low.
    assign status_byte = {
        4'b0000,
        canintf[TX0IF],
        txreq,
        1'b0,
        canintf[RX0IF]
    };

    // -------------------------------------------------------------------------
    // SPI RX STATUS
    // -------------------------------------------------------------------------
    // Single-RX-buffer subset: bits [7:6] identify a pending RXB0 message as
    // 01, bit 5 is unused, bit 4 is IDE, bit 3 is RTR/SRR, and bits [2:0]
    // report the filter-hit code. Extended frames are excluded, so rx_ide is
    // tied low by can_top; with one filter, can_top supplies filter-hit 000.
    assign rxstatus_byte = {
        1'b0,
        rx_full,
        1'b0,
        rx_ide,
        rx_rtr,
        rx_filter_hit[2:0]
    };

    // -------------------------------------------------------------------------
    // Active-low interrupt routing
    // -------------------------------------------------------------------------
    assign int_pin = control_reset ? 1'b1 : ~|(canintf & caninte);

endmodule
