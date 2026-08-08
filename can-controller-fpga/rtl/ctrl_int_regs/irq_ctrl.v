// =============================================================================
// irq_ctrl.v -- Interrupt controller (Module 5, SOW 8.4.5 / 8.4.6).
//
// Owns CANINTF (the flag register), the INT pin, and the CANSTAT.ICOD priority
// encode. CANINTE lives in reg_bank.v as a plain read/write register and is
// passed in here purely as a mask -- this module never writes it.
//
// CANINTF is the one register in the map that is neither read-only nor plain
// read/write. Per datasheet Register 10-3 each bit is:
//   - SET by hardware only. A host write of 1 to a clear bit does nothing.
//   - CLEARED by the host writing 0 (via WRITE or BIT MODIFY).
//   - additionally, RXnIF is cleared by hardware on SPI READ RX BUFFER
//     (Section 12.4).
// When a hardware set and a host clear land on the same bit in the same cycle,
// the SET wins. Resolving it the other way would silently drop an interrupt
// whenever the host happened to be acknowledging a different flag in the same
// byte -- a real lost-interrupt bug, not a theoretical one, because the host
// clears CANINTF one whole byte at a time.
// =============================================================================

module irq_ctrl (
    input  wire       clk,
    input  wire       rst_n,          // active-low async reset
    input  wire       sync_reset,     // SPI RESET instruction

    // CANINTE, owned by reg_bank.v -- mask only, never written here.
    input  wire [7:0] caninte,

    // Host access to CANINTF, routed through from reg_bank.v's write path.
    // canintf_wdata is the value the host wants the register to hold; for a
    // BIT MODIFY it is already merged with the mask by reg_bank.v, so this
    // module sees a plain "target value" either way.
    input  wire       canintf_we,
    input  wire [7:0] canintf_wdata,

    // Hardware set sources, one-cycle pulses.
    input  wire       set_rx0if,      // rx_buffer0 stored a valid message
    input  wire       set_rx1if,      // rx_buffer1 stored a valid message
    input  wire [2:0] set_txif,       // TXB0/1/2 transmission complete
    input  wire       set_errif,      // an EFLG bit went 0 -> 1
    input  wire       set_merrf,      // message error from the protocol engine
    input  wire       set_wakif,      // bus wake-up (tied low -- out of scope)

    // Automatic RXnIF clear on SPI READ RX BUFFER (Section 12.4). spi_if.v
    // pulses rxbuf_done on CS rise and holds rxbuf_sel to say which buffer.
    input  wire       rxbuf_done,
    input  wire       rxbuf_sel,      // 0 = RXB0, 1 = RXB1

    output reg  [7:0] canintf,
    output wire       int_n,          // active-low INT pin
    output reg  [2:0] icod            // -> CANSTAT.ICOD[2:0]
);

    // Datasheet Register 10-3 (CANINTF) / 10-2 (CANINTE) bit positions.
    localparam RX0IF = 0, RX1IF = 1, TX0IF = 2, TX1IF = 3,
               TX2IF = 4, ERRIF = 5, WAKIF = 6, MERRF = 7;

    // ---- Hardware set vector -------------------------------------------
    wire [7:0] hw_set;
    assign hw_set[RX0IF] = set_rx0if;
    assign hw_set[RX1IF] = set_rx1if;
    assign hw_set[TX0IF] = set_txif[0];
    assign hw_set[TX1IF] = set_txif[1];
    assign hw_set[TX2IF] = set_txif[2];
    assign hw_set[ERRIF] = set_errif;
    assign hw_set[WAKIF] = set_wakif;
    assign hw_set[MERRF] = set_merrf;

    // ---- Hardware clear vector (READ RX BUFFER only) --------------------
    wire [7:0] hw_clr;
    assign hw_clr[RX0IF] = rxbuf_done && !rxbuf_sel;
    assign hw_clr[RX1IF] = rxbuf_done &&  rxbuf_sel;
    assign hw_clr[7:2]   = 6'b000000;

    // ---- Host clear vector ----------------------------------------------
    // Only bits that are currently SET and that the host is writing 0 to.
    // Deriving it this way is what makes "writing 1 cannot set a flag"
    // (SOW 8.4.5.34) fall out for free -- a 1 in canintf_wdata never appears
    // in host_clr, and there is no host_set term anywhere in this module.
    wire [7:0] host_clr = canintf_we ? (canintf & ~canintf_wdata) : 8'h00;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            canintf <= 8'h00;
        else if (sync_reset)
            canintf <= 8'h00;
        else
            // OR-ing hw_set in last is the whole set-wins-over-clear rule.
            canintf <= (canintf & ~(host_clr | hw_clr)) | hw_set;
    end

    // ---- INT pin ---------------------------------------------------------
    // Level-driven, not pulsed: stays asserted until every ENABLED flag has
    // been cleared (SOW 8.4.6.39). A flag that is set but not enabled does
    // not hold the pin.
    wire [7:0] pending = canintf & caninte;
    assign int_n = (pending == 8'h00);

    // ---- CANSTAT.ICOD ----------------------------------------------------
    // Fixed priority, datasheet Register 10-2 (CANSTAT) ICOD table. MERRF has
    // no ICOD encoding of its own in that table, so it is deliberately absent
    // here even though it can assert INT.
    always @(*) begin
        if      (pending[ERRIF]) icod = 3'b001;   // Error
        else if (pending[WAKIF]) icod = 3'b010;   // Wake-up
        else if (pending[TX0IF]) icod = 3'b011;   // TXB0
        else if (pending[TX1IF]) icod = 3'b100;   // TXB1
        else if (pending[TX2IF]) icod = 3'b101;   // TXB2
        else if (pending[RX0IF]) icod = 3'b110;   // RXB0
        else if (pending[RX1IF]) icod = 3'b111;   // RXB1
        else                     icod = 3'b000;   // no interrupt
    end

endmodule
