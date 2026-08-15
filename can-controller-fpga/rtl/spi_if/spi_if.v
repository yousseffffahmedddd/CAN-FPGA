// `include "../common/can_defs.vh"

// // =============================================================================
// // Module : spi_if
// // -----------------------------------------------------------------------------
// // Synchronous SPI Slave Interface Module for CAN Controller.
// // Operates on system clock (clk).
// // =============================================================================

// module spi_if (
//     input  wire        clk,
//     input  wire        rst_n,

//     // SPI Physical Pins
//     input  wire        sck,
//     input  wire        si,
//     output wire        so,
//     input  wire        cs_n,

//     // Internal Register & Top Interface
//     output reg  [7:0]  addr,
//     output reg  [7:0]  wdata,
//     input  wire [7:0]  rdata,
//     output reg         we,
//     output reg         reset_pulse,
//     output reg         rts_pulse,
//     output reg         rxbuf_done,
//     input  wire [7:0]  status_byte,
//     input  wire [7:0]  rxstatus_byte
// );

//     // Standard Opcodes
//     localparam [7:0] OP_WRITE       = 8'h02;
//     localparam [7:0] OP_READ        = 8'h03;
//     localparam [7:0] OP_BIT_MODIFY  = 8'h05;
//     localparam [7:0] OP_RESET       = 8'hC0;
//     localparam [7:0] OP_RTS         = 8'h80;
//     localparam [7:0] OP_READ_STATUS = 8'hA0;
//     localparam [7:0] OP_RX_STATUS   = 8'hB0;

//     // Double-flop synchronizers for SCK and SI
//     reg [1:0] sck_sync;
//     reg [1:0] si_sync;

//     always @(posedge clk or negedge rst_n) begin
//         if (!rst_n) begin
//             sck_sync <= 2'b00;
//             si_sync  <= 2'b00;
//         end else begin
//             sck_sync <= {sck_sync[0], sck};
//             si_sync  <= {si_sync[0], si};
//         end
//     end

//     wire sck_rising  = (sck_sync == 2'b01);
//     wire sck_falling = (sck_sync == 2'b10);

//     // Direct CS_N evaluation so MISO tri-state activates with zero delay
//     wire cs_active = ~cs_n;

//     // Internal SPI Registers
//     reg [2:0] bit_cnt;
//     reg [2:0] byte_cnt;
//     reg [7:0] cmd_reg;
//     reg [7:0] rx_shift;
//     reg [7:0] tx_shift;
//     reg [7:0] mask_reg;
//     reg       load_rdata_req;

//     // Tri-state MISO driver
//     wire is_status_op = (cmd_reg == OP_READ_STATUS) || (cmd_reg == OP_RX_STATUS);
//     wire is_read_op   = (cmd_reg == OP_READ) || is_status_op;

//     wire drive_miso   = cs_active && (
//                           (is_status_op && byte_cnt >= 3'd1) ||
//                           (cmd_reg == OP_READ && byte_cnt >= 3'd2)
//                         );

//     assign so = drive_miso ? tx_shift[7] : 1'bz;

//     wire [7:0] incoming_byte = {rx_shift[6:0], si_sync[0]};

//     always @(posedge clk or negedge rst_n) begin
//         if (!rst_n) begin
//             bit_cnt        <= 3'd0;
//             byte_cnt       <= 3'd0;
//             cmd_reg        <= 8'h00;
//             rx_shift       <= 8'h00;
//             tx_shift       <= 8'h00;
//             mask_reg       <= 8'h00;
//             addr           <= 8'h00;
//             wdata          <= 8'h00;
//             we             <= 1'b0;
//             reset_pulse    <= 1'b0;
//             rts_pulse      <= 1'b0;
//             rxbuf_done     <= 1'b0;
//             load_rdata_req <= 1'b0;
//         end else if (!cs_active) begin
//             bit_cnt        <= 3'd0;
//             byte_cnt       <= 3'd0;
//             cmd_reg        <= 8'h00;
//             we             <= 1'b0;
//             reset_pulse    <= 1'b0;
//             rts_pulse      <= 1'b0;
//             rxbuf_done     <= 1'b0;
//             load_rdata_req <= 1'b0;
//         end else begin
//             // Single-cycle default pulses
//             reset_pulse <= 1'b0;
//             rts_pulse   <= 1'b0;
//             rxbuf_done  <= 1'b0;

//             // Deassert write enable after 1 system clock cycle and auto-increment address
//             if (we) begin
//                 we   <= 1'b0;
//                 addr <= addr + 8'd1;
//             end

//             // Load rdata into tx_shift 1 clock cycle after addr is updated
//             if (load_rdata_req) begin
//                 tx_shift       <= rdata;
//                 load_rdata_req <= 1'b0;
//             end

//             // -----------------------------------------------------------------
//             // Sample MOSI on SCK Rising Edge
//             // -----------------------------------------------------------------
//             if (sck_rising) begin
//                 rx_shift <= incoming_byte;

//                 if (bit_cnt == 3'd7) begin
//                     bit_cnt <= 3'd0;

//                     case (byte_cnt)
//                         3'd0: begin
//                             // Byte 0: Opcode
//                             cmd_reg  <= incoming_byte;
//                             byte_cnt <= 3'd1;

//                             case (incoming_byte)
//                                 OP_RESET:       reset_pulse <= 1'b1;
//                                 OP_RTS:         rts_pulse   <= 1'b1;
//                                 OP_READ_STATUS: tx_shift    <= status_byte;
//                                 OP_RX_STATUS:   tx_shift    <= rxstatus_byte;
//                                 default: ;
//                             endcase
//                         end

//                         3'd1: begin
//                             // Byte 1: Address
//                             if (cmd_reg == OP_READ) begin
//                                 addr           <= incoming_byte;
//                                 byte_cnt       <= 3'd2;
//                                 load_rdata_req <= 1'b1;
//                             end else if (cmd_reg == OP_WRITE || cmd_reg == OP_BIT_MODIFY) begin
//                                 addr     <= incoming_byte;
//                                 byte_cnt <= 3'd2;
//                             end
//                         end

//                         3'd2: begin
//                             // Byte 2: Data / Mask
//                             case (cmd_reg)
//                                 OP_WRITE: begin
//                                     wdata <= incoming_byte;
//                                     we    <= 1'b1;
//                                 end
//                                 OP_READ: begin
//                                     addr           <= addr + 8'd1;
//                                     load_rdata_req <= 1'b1;
//                                 end
//                                 OP_BIT_MODIFY: begin
//                                     mask_reg <= incoming_byte;
//                                     byte_cnt <= 3'd3;
//                                 end
//                                 default: ;
//                             endcase
//                         end

//                         3'd3: begin
//                             // Byte 3: Bit Modify Data
//                             if (cmd_reg == OP_BIT_MODIFY) begin
//                                 wdata <= (rdata & ~mask_reg) | (incoming_byte & mask_reg);
//                                 we    <= 1'b1;
//                             end
//                         end

//                         default: ;
//                     endcase
//                 end else begin
//                     bit_cnt <= bit_cnt + 3'd1;
//                 end
//             end

//             // -----------------------------------------------------------------
//             // Shift MISO on SCK Falling Edge
//             // -----------------------------------------------------------------
//             if (sck_falling) begin
//                 // FIX: Only shift if bit_cnt != 0 to prevent losing the MSB right after a byte load
//                 if (is_read_op && drive_miso && !load_rdata_req && bit_cnt != 3'd0) begin
//                     tx_shift <= {tx_shift[6:0], 1'b0};
//                 end
//             end
//         end
//     end

// endmodule

`include "../common/can_defs.vh"

// =============================================================================
// Module : spi_if
// -----------------------------------------------------------------------------
// Synchronous SPI Slave Interface Module for CAN Controller.
// Operates on system clock (clk). Supports standard MCP2515 opcodes.
// =============================================================================

module spi_if (
    input  wire        clk,
    input  wire        rst_n,

    // SPI Physical Pins
    input  wire        sck,
    input  wire        si,
    output wire        so,
    input  wire        cs_n,

    // Internal Register & Top Interface
    output reg  [7:0]  addr,
    output reg  [7:0]  wdata,
    input  wire [7:0]  rdata,
    output reg         we,
    output reg         reset_pulse,
    output reg         rts_pulse,
    output reg         rxbuf_done,
    input  wire [7:0]  status_byte,
    input  wire [7:0]  rxstatus_byte
);

    // Standard Opcodes & Bitmasks
    localparam [7:0] OP_WRITE       = 8'h02;
    localparam [7:0] OP_READ        = 8'h03;
    localparam [7:0] OP_BIT_MODIFY  = 8'h05;
    localparam [7:0] OP_RESET       = 8'hC0;
    localparam [7:0] OP_READ_STATUS = 8'hA0;
    localparam [7:0] OP_RX_STATUS   = 8'hB0;

    // Double-flop synchronizers for SCK and SI
    reg [1:0] sck_sync;
    reg [1:0] si_sync;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sck_sync <= 2'b00;
            si_sync  <= 2'b00;
        end else begin
            sck_sync <= {sck_sync[0], sck};
            si_sync  <= {si_sync[0], si};
        end
    end

    wire sck_rising  = (sck_sync == 2'b01);
    wire sck_falling = (sck_sync == 2'b10);

    // Direct CS_N evaluation so MISO tri-state activates with zero delay
    wire cs_active = ~cs_n;

    // Internal SPI Registers
    reg [2:0] bit_cnt;
    reg [2:0] byte_cnt;
    reg [7:0] cmd_reg;
    reg [7:0] rx_shift;
    reg [7:0] tx_shift;
    reg [7:0] mask_reg;
    reg       load_rdata_req;

    // Opcodes categorization
    wire is_status_op  = (cmd_reg == OP_READ_STATUS) || (cmd_reg == OP_RX_STATUS);
    wire is_read_op    = (cmd_reg == OP_READ) || is_status_op;
    wire is_load_tx    = (cmd_reg[7:3] == 5'b01000); // 0x40 to 0x4F (Load TX Buffer)

    wire drive_miso    = cs_active && (
                           (is_status_op && byte_cnt >= 3'd1) ||
                           (cmd_reg == OP_READ && byte_cnt >= 3'd2)
                         );

    assign so = drive_miso ? tx_shift[7] : 1'bz;

    wire [7:0] incoming_byte = {rx_shift[6:0], si_sync[0]};

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            bit_cnt        <= 3'd0;
            byte_cnt       <= 3'd0;
            cmd_reg        <= 8'h00;
            rx_shift       <= 8'h00;
            tx_shift       <= 8'h00;
            mask_reg       <= 8'h00;
            addr           <= 8'h00;
            wdata          <= 8'h00;
            we             <= 1'b0;
            reset_pulse    <= 1'b0;
            rts_pulse      <= 1'b0;
            rxbuf_done     <= 1'b0;
            load_rdata_req <= 1'b0;
        end else if (!cs_active) begin
            bit_cnt        <= 3'd0;
            byte_cnt       <= 3'd0;
            cmd_reg        <= 8'h00;
            we             <= 1'b0;
            reset_pulse    <= 1'b0;
            rts_pulse      <= 1'b0;
            rxbuf_done     <= 1'b0;
            load_rdata_req <= 1'b0;
        end else begin
            // Single-cycle default pulses
            reset_pulse <= 1'b0;
            rts_pulse   <= 1'b0;
            rxbuf_done  <= 1'b0;

            // Deassert write enable after 1 system clock cycle and auto-increment address
            if (we) begin
                we   <= 1'b0;
                addr <= addr + 8'd1;
            end

            // Load rdata into tx_shift 1 clock cycle after addr is updated
            if (load_rdata_req) begin
                tx_shift       <= rdata;
                load_rdata_req <= 1'b0;
            end

            // -----------------------------------------------------------------
            // Sample MOSI on SCK Rising Edge
            // -----------------------------------------------------------------
            if (sck_rising) begin
                rx_shift <= incoming_byte;

                if (bit_cnt == 3'd7) begin
                    bit_cnt <= 3'd0;

                    case (byte_cnt)
                        3'd0: begin
                            // Byte 0: Opcode
                            cmd_reg  <= incoming_byte;
                            
                            if (incoming_byte == OP_RESET) begin
                                reset_pulse <= 1'b1;
                            end else if (incoming_byte[7:3] == 5'b10000) begin
                                // RTS Command (1000 0nnn -> 0x80 to 0x8F)
                                rts_pulse <= 1'b1;
                            end else if (incoming_byte[7:3] == 5'b01000) begin
                                // Load TX Buffer Command (0100 0nnn -> 0x40 to 0x4F)
                                // Map buffer selection to base address (TXB0SIDH = 0x31)
                                addr     <= 8'h31 + ({5'b0, incoming_byte[2:0]} * 16'd14);
                                byte_cnt <= 3'd2; // Skip address byte, jump straight to data stream
                            end else begin
                                byte_cnt <= 3'd1;
                                case (incoming_byte)
                                    OP_READ_STATUS: tx_shift <= status_byte;
                                    OP_RX_STATUS:   tx_shift <= rxstatus_byte;
                                    default: ;
                                endcase
                            end
                        end

                        3'd1: begin
                            // Byte 1: Address (for WRITE, READ, BIT_MODIFY)
                            if (cmd_reg == OP_READ) begin
                                addr           <= incoming_byte;
                                byte_cnt       <= 3'd2;
                                load_rdata_req <= 1'b1;
                            end else if (cmd_reg == OP_WRITE || cmd_reg == OP_BIT_MODIFY) begin
                                addr     <= incoming_byte;
                                byte_cnt <= 3'd2;
                            end
                        end

                        3'd2: begin
                            // Byte 2: Data / Mask
                            case (cmd_reg)
                                OP_WRITE: begin
                                    wdata <= incoming_byte;
                                    we    <= 1'b1;
                                end
                                OP_READ: begin
                                    addr           <= addr + 8'd1;
                                    load_rdata_req <= 1'b1;
                                end
                                OP_BIT_MODIFY: begin
                                    mask_reg <= incoming_byte;
                                    byte_cnt <= 3'd3;
                                end
                                default: begin
                                    // Handles continuous streaming for Load TX Buffer (0x40+)
                                    if (is_load_tx) begin
                                        wdata <= incoming_byte;
                                        we    <= 1'b1;
                                    end
                                end
                            endcase
                        end

                        3'd3: begin
                            // Byte 3: Bit Modify Data or continued Load TX Buffer stream
                            if (cmd_reg == OP_BIT_MODIFY) begin
                                wdata <= (rdata & ~mask_reg) | (incoming_byte & mask_reg);
                                we    <= 1'b1;
                            end else if (is_load_tx) begin
                                wdata <= incoming_byte;
                                we    <= 1'b1;
                            end
                        end

                        default: begin
                            // Handle any remaining bytes in a continuous load stream
                            if (is_load_tx) begin
                                wdata <= incoming_byte;
                                we    <= 1'b1;
                            end
                        end
                    endcase
                end else begin
                    bit_cnt <= bit_cnt + 3'd1;
                end
            end

            // -----------------------------------------------------------------
            // Shift MISO on SCK Falling Edge
            // -----------------------------------------------------------------
            if (sck_falling) begin
                if (is_read_op && drive_miso && !load_rdata_req && bit_cnt != 3'd0) begin
                    tx_shift <= {tx_shift[6:0], 1'b0};
                end
            end
        end
    end

endmodule