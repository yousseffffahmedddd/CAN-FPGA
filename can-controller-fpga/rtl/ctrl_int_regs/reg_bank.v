`include "../common/can_defs.vh"

// =============================================================================
// Module : reg_bank
// -----------------------------------------------------------------------------
// SOW-Verified Control Register Bank
// Scope Enforced:
//   - 1 TX Buffer  (TXB0)
//   - 1 RX Buffer  (RXB0)
//   - 1 Acceptance Filter (RXF0)
//   - 1 Acceptance Mask   (RXM0)
//   - Normal Mode operation (Config mode bypassable)
// =============================================================================

module reg_bank (
    input  wire        clk,
    input  wire        rst_n,

    // Host SPI CDC interface
    input  wire [7:0]  addr,
    input  wire [7:0]  wdata,
    input  wire        we,
    output reg  [7:0]  rdata,
    input  wire        reset_pulse,
    input  wire        rts_pulse,      // Trigger TXREQ for TXB0
    input  wire        rxbuf_done,     // Clear RX0IF when host reads RX buffer

    // SPI Quick-Status Bytes
    output wire [7:0]  status_byte,
    output wire [7:0]  rxstatus_byte,

    // Hardware Interrupt Output
    output wire        int_n,

    // Protocol Engine Status Signals
    input  wire        bus_idle,
    input  wire [7:0]  tec,
    input  wire [7:0]  rec,
    input  wire [1:0]  err_state,
    input  wire        tx_done_pulse,  // TX done from protocol engine
    input  wire        rx_done_pulse,  // RX done from protocol engine
    input  wire        msg_err,

    // Operation Modes
    output wire [2:0]  opmod,
    output wire        config_mode,
    output wire        normal_mode,

    // TX Buffer 0 Interface
    output reg  [10:0] txb0_id,
    output reg  [63:0] txb0_data,
    output reg  [3:0]  txb0_dlc,
    output reg         txb0_rtr,
    output reg         txb0_txreq,

    // RX Buffer 0 Interface
    input  wire [10:0] rxb0_id,
    input  wire [63:0] rxb0_data,
    input  wire [3:0]  rxb0_dlc,
    input  wire        rxb0_rtr,
    output reg         rxb0_full,
    output reg         rx0_ovr,

    // Filter & Mask Configuration
    output reg  [10:0] rxm0_mask,
    output reg  [10:0] rxf0_id
);

    // -------------------------------------------------------------------------
    // Register Address Map (MCP2515 Compatible Subset)
    // -------------------------------------------------------------------------
    localparam ADDR_CANCTRL  = 8'h0F;
    localparam ADDR_CANSTAT  = 8'h0E;
    localparam ADDR_CANINTF  = 8'h2C;
    localparam ADDR_CANINTE  = 8'h2B;
    localparam ADDR_TEC      = 8'h1C;
    localparam ADDR_REC      = 8'h1D;

    // Filter / Mask Registers
    localparam ADDR_RXM0SIDH = 8'h20;
    localparam ADDR_RXM0SIDL = 8'h21;
    localparam ADDR_RXF0SIDH = 8'h00;
    localparam ADDR_RXF0SIDL = 8'h01;

    // TX Buffer 0 Registers
    localparam ADDR_TXB0CTRL = 8'h30;
    localparam ADDR_TXB0SIDH = 8'h31;
    localparam ADDR_TXB0SIDL = 8'h32;
    localparam ADDR_TXB0DLC  = 8'h35;
    localparam ADDR_TXB0D0   = 8'h36;

    // RX Buffer 0 Registers
    localparam ADDR_RXB0CTRL = 8'h60;
    localparam ADDR_RXB0SIDH = 8'h61;
    localparam ADDR_RXB0SIDL = 8'h62;
    localparam ADDR_RXB0DLC  = 8'h65;
    localparam ADDR_RXB0D0   = 8'h66;

    // -------------------------------------------------------------------------
    // Internal Registers
    // -------------------------------------------------------------------------
    reg [2:0] reqmod;
    reg [7:0] caninte;
    reg [7:0] canintf;

    assign opmod       = 3'b000; // Fixed Normal Mode per SOW
    assign config_mode = 1'b0;
    assign normal_mode = 1'b1;

    // Quick Status Bytes for Fast SPI Reads
    assign status_byte   = {1'b0, txb0_txreq, 3'b000, canintf[1], canintf[0]};
    assign rxstatus_byte = {rxb0_full, 3'b000, rxb0_rtr, 3'b000};

    // Interrupt Request Output (Active Low)
    assign int_n = ~|(caninte & canintf);

    // -------------------------------------------------------------------------
    // Register Write Logic
    // -------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            reqmod    <= 3'b000;
            caninte   <= 8'h00;
            txb0_id   <= 11'd0;
            txb0_data <= 64'd0;
            txb0_dlc  <= 4'd0;
            txb0_rtr  <= 1'b0;
            txb0_txreq<= 1'b0;
            rxm0_mask <= 11'h7FF; // Default: Match all 11 bits
            rxf0_id   <= 11'd0;
        end else if (reset_pulse) begin
            txb0_txreq<= 1'b0;
            caninte   <= 8'h00;
        end else begin
            // RTS pulse trigger from SPI
            if (rts_pulse) begin
                txb0_txreq <= 1'b1;
            end

            // Clear TXREQ on transmission success
            if (tx_done_pulse) begin
                txb0_txreq <= 1'b0;
            end

            // Register Writes via SPI
            if (we) begin
                case (addr)
                    ADDR_CANCTRL: reqmod <= wdata[7:5];
                    ADDR_CANINTE: caninte <= wdata;
                    ADDR_TXB0CTRL: txb0_txreq <= wdata[3];

                    // Standard ID Formats: SIDH = ID[10:3], SIDL[7:5] = ID[2:0]
                    ADDR_TXB0SIDH: txb0_id[10:3] <= wdata;
                    ADDR_TXB0SIDL: begin
                        txb0_id[2:0] <= wdata[7:5];
                        txb0_rtr     <= wdata[4];
                    end
                    ADDR_TXB0DLC:  txb0_dlc <= wdata[3:0];

                    // Data Payload Bytes (Byte 0 shown mapped for illustration)
                    ADDR_TXB0D0:   txb0_data[7:0] <= wdata;

                    // Mask & Filter ID Mapping
                    ADDR_RXM0SIDH: rxm0_mask[10:3] <= wdata;
                    ADDR_RXM0SIDL: rxm0_mask[2:0]  <= wdata[7:5];
                    ADDR_RXF0SIDH: rxf0_id[10:3]   <= wdata;
                    ADDR_RXF0SIDL: rxf0_id[2:0]    <= wdata[7:5];
                    default: ;
                endcase
            end
        end
    end

    // -------------------------------------------------------------------------
    // Interrupt & Buffer Flag Management
    // -------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            canintf   <= 8'h00;
            rxb0_full <= 1'b0;
            rx0_ovr   <= 1'b0;
        end else begin
            // Interrupt Flag 0 (RX0IF) & Buffer Full Logic
            if (rx_done_pulse) begin
                if (rxb0_full) begin
                    rx0_ovr <= 1'b1; // Overrun if unread
                end else begin
                    rxb0_full  <= 1'b1;
                    canintf[0] <= 1'b1; // RX0IF
                end
            end else if (rxbuf_done) begin
                rxb0_full  <= 1'b0;
                canintf[0] <= 1'b0;
            end

            // Interrupt Flag 2 (TX0IF)
            if (tx_done_pulse) begin
                canintf[2] <= 1'b1; // TX0IF
            end

            // Explicit Flag Clears via Host Writes to CANINTF
            if (we && (addr == ADDR_CANINTF)) begin
                canintf <= wdata;
            end
        end
    end

    // -------------------------------------------------------------------------
    // Register Read Multiplexer
    // -------------------------------------------------------------------------
    always @(*) begin
        case (addr)
            ADDR_CANCTRL:  rdata = {reqmod, 1'b0, 4'b0000};
            ADDR_CANSTAT:  rdata = {opmod, 1'b0, 4'b0000};
            ADDR_CANINTF:  rdata = canintf;
            ADDR_CANINTE:  rdata = caninte;
            ADDR_TEC:      rdata = tec;
            ADDR_REC:      rdata = rec;

            ADDR_TXB0CTRL: rdata = {4'b0000, txb0_txreq, 3'b000};
            ADDR_TXB0SIDH: rdata = txb0_id[10:3];
            ADDR_TXB0SIDL: rdata = {txb0_id[2:0], txb0_rtr, 4'b0000};
            ADDR_TXB0DLC:  rdata = {4'b0000, txb0_dlc};

            ADDR_RXB0CTRL: rdata = {1'b0, 2'b00, 1'b0, rxb0_rtr, 3'b000};
            ADDR_RXB0SIDH: rdata = rxb0_id[10:3];
            ADDR_RXB0SIDL: rdata = {rxb0_id[2:0], rxb0_rtr, 4'b0000};
            ADDR_RXB0DLC:  rdata = {4'b0000, rxb0_dlc};
            ADDR_RXB0D0:   rdata = rxb0_data[7:0];

            ADDR_RXM0SIDH: rdata = rxm0_mask[10:3];
            ADDR_RXM0SIDL: rdata = {rxm0_mask[2:0], 5'b00000};
            ADDR_RXF0SIDH: rdata = rxf0_id[10:3];
            ADDR_RXF0SIDL: rdata = {rxf0_id[2:0], 5'b00000};
            default:       rdata = 8'h00;
        endcase
    end

endmodule
