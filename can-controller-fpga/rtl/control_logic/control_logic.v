module control_logic (
    input wire clk,
    input wire rst_n,

    // SPI Interface Interconnect
    input wire [7:0] spi_addr,
    input wire [7:0] spi_wdata,
    input wire       spi_we,
    output reg [7:0] spi_rdata,
    input wire       rts_pulse,      // Direct hardware pulse from SPI RTS instruction
    input wire       rx_read_pulse,  // Clears RX buffer flag on READ RX BUFFER

    // Protocol Engine Interconnect
    input wire       tx_done,        // Transmission successful pulse
    input wire       tx_aborted,     // Transmission aborted pulse
    input wire       error_detected, // Error pulse from protocol engine
    input wire [2:0] error_type,     // Error code descriptor
    input wire       rx_complete,    // Message received successfully

    // Buffer Interconnect
    output reg       tx_req,         // TX Request flag to Protocol Engine / TX Buffer
    output reg       tx_buffer_we,   // Write enable to TX buffer
    input wire       tx_buffer_rdy,  // TX buffer ready/valid flag
    output reg       rx_buffer_clear,// Clear RX buffer flag after host read

    // Physical Interrupt Output
    output wire      int_n           // Active-low physical interrupt output pin
);

    // -------------------------------------------------------------------------
    // Internal Registers & State Mapping
    // -------------------------------------------------------------------------
    reg [7:0] canctrl;
    reg [7:0] canstat;
    reg [7:0] caninte;
    reg [7:0] canintf;
    reg [7:0] eflg;
    reg [7:0] tec; // Transmit Error Counter
    reg [7:0] rec; // Receive Error Counter

    // Error States
    localparam ST_ERROR_ACTIVE  = 2'b00;
    localparam ST_ERROR_PASSIVE = 2'b01;
    localparam ST_BUS_OFF       = 2'b10;
    
    reg [1:0] node_state;
    reg [7:0] bus_off_recovery_count; // Tracks 128 occurrences of 11 recessive bits
    
    // -------------------------------------------------------------------------
    // Register Read/Write & Initialization
    // -------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            canctrl   <= 8'h00; // Normal Mode default (REQOP = 000)
            caninte   <= 8'h00;
            canintf   <= 8'h00;
            eflg      <= 8'h00;
            tec       <= 8'h00;
            rec       <= 8'h00;
            node_state <= ST_ERROR_ACTIVE;
            bus_off_recovery_count <= 8'd0;
            tx_buffer_we <= 1'b0;
            rx_buffer_clear <= 1'b0;
        end else begin
            tx_buffer_we    <= 1'b0;
            rx_buffer_clear <= 1'b0;

            // SPI Register Write Routing
            if (spi_we) begin
                case (spi_addr)
                    8'h2B: caninte <= spi_wdata;
                    8'h2C: canintf <= canintf & spi_wdata; // Write 0 to clear flags
                    8'h2D: eflg    <= eflg & spi_wdata;
                    default: ;
                endcase
            end

            // RX Buffer Read Clearance
            if (rx_read_pulse) begin
                canintf[0]      <= 1'b0; // Clear RX0IF
                rx_buffer_clear <= 1'b1;
            end

            // -----------------------------------------------------------------
            // TXREQ State Machine Management
            // -----------------------------------------------------------------
            if (rts_pulse && (node_state != ST_BUS_OFF)) begin
                tx_req <= 1'b1;
            end else if (tx_done || tx_aborted) begin
                tx_req <= 1'b0;
            end

            // -----------------------------------------------------------------
            // Error Management Logic (TEC/REC & State Transitions)
            // -----------------------------------------------------------------
            if (error_detected) begin
                canintf[7] <= 1'b1; // MERRF (Message Error Flag)
                // Increment TEC/REC based on protocol rules (simplified summary)
                if (tec < 8'hFF) tec <= tec + 8'd8;
            end

            if (tx_done) begin
                if (tec > 0) tec <= tec - 8'd1;
            end

            if (rx_complete) begin
                canintf[0] <= 1'b1; // Set RX0IF (Single Receive Buffer instance)
                if (rec > 0) rec <= rec - 8'd1;
            end

            // Node Error State Evaluation
            if (tec > 8'd255) begin
                node_state <= ST_BUS_OFF;
                eflg[5]    <= 1'b1; // TXBO (Bus-Off Flag)
                tx_req     <= 1'b0; // Block transmission during Bus-Off
            end else if ((tec >= 8'd128) || (rec >= 8'd128)) begin
                node_state <= ST_ERROR_PASSIVE;
                if (tec >= 8'd128) eflg[4] <= 1'b1; // TXEP
                if (rec >= 8'd128) eflg[3] <= 1'b1; // RXEP
            end else begin
                node_state <= ST_ERROR_ACTIVE;
            end
        end
    end

    // -------------------------------------------------------------------------
    // SPI Read Data Multiplexing & Status Aggregation
    // -------------------------------------------------------------------------
    always @(*) begin
        case (spi_addr)
            8'h0C: spi_rdata = 8'h00; // BFPCTRL stubbed
            8'h1C: spi_rdata = tec;
            8'h1D: spi_rdata = rec;
            8'h28: spi_rdata = 8'h00; // CNF3
            8'h29: spi_rdata = 8'h00; // CNF2
            8'h2A: spi_rdata = 8'h00; // CNF1
            8'h2B: spi_rdata = caninte;
            8'h2C: spi_rdata = canintf;
            8'h2D: spi_rdata = eflg;
            8'h30: spi_rdata = {1'b0, 1'b0, 1'b0, 1'b0, tx_req, 2'b00, tx_buffer_rdy}; // TXB0CTRL
            8'h60: spi_rdata = {1'b0, 2'b00, 1'b0, 1'b0, 1'b0, 1'b0, canintf[0]};   // RXB0CTRL
            8'hEE: spi_rdata = {3'b000, 1'b0, 3'b000, 1'b0}; // CANSTAT (Normal Mode = 000)
            8'hEF: spi_rdata = canctrl;
            default: spi_rdata = 8'h00;
        endcase
    end

    // -------------------------------------------------------------------------
    // Interrupt Routing Logic (Drives physical INT pin low on unmasked events)
    // -------------------------------------------------------------------------
    wire interrupt_pending = |(canintf & caninte);
    assign int_n = interrupt_pending ? 1'b0 : 1'b1;

endmodule