module spi_if (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        sck,
    input  wire        si,
    output wire        so,
    input  wire        cs_n,
    output reg  [7:0]  addr,
    output reg  [7:0]  wdata,
    input  wire [7:0]  rdata,
    output reg          we,
    output reg          reset_pulse,
    output reg          rts_pulse,
    output reg          rxbuf_done,
    input  wire [7:0]  status_byte,
    input  wire [7:0]  rxstatus_byte
);
    localparam [7:0] OP_RESET    = 8'hC0;
    localparam [7:0] OP_READ     = 8'h03;
    localparam [7:0] OP_WRITE    = 8'h02;
    localparam [7:0] OP_READSTAT = 8'hA0;
    localparam [7:0] OP_RXSTAT   = 8'hB0;
    localparam [7:0] LOADTX_MASK = 8'hFE, LOADTX_ID = 8'h40;
    localparam [7:0] RXBUF_MASK  = 8'hFD, RXBUF_ID  = 8'h90;
    localparam [7:0] OP_RTS_TXB0 = 8'h81;
    localparam [7:0] ADDR_TXB_SIDH = 8'h31, ADDR_TXB_D0 = 8'h36;
    localparam [7:0] ADDR_RXB_SIDH = 8'h61, ADDR_RXB_D0 = 8'h66;

    // TXB0's register layout has a gap: SIDL (0x32) is followed by EID8/EID0
    // (0x33/0x34), which this standard-ID-only, single-buffer design does not
    // implement in reg_bank.v. A plain addr+1 auto-increment during a LOAD TX
    // BUFFER stream would walk through that gap and land DLC/data two
    // addresses early. ADDR_TXB_SIDL/ADDR_TXB_DLC name the two ends of the
    // jump this patch adds.
    localparam [7:0] ADDR_TXB_SIDL = 8'h32, ADDR_TXB_DLC = 8'h35;

    reg [2:0] cs_sync, sck_sync;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cs_sync  <= 3'b111;
            sck_sync <= 3'b000;
        end else begin
            cs_sync  <= {cs_sync[1:0],  cs_n};
            sck_sync <= {sck_sync[1:0], sck};
        end
    end
    wire cs_n_s      = cs_sync[2];
    wire sck_rising  = (sck_sync[2:1] == 2'b01);
    wire sck_falling = (sck_sync[2:1] == 2'b10);
    wire cs_falling  = (cs_sync[2:1]  == 2'b10);
    wire cs_rising   = (cs_sync[2:1]  == 2'b01);
    reg [2:0] bit_cnt;
    wire      byte_done_raw = (bit_cnt == 3'd7) && sck_rising && !cs_n_s;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            bit_cnt <= 3'd0;
        else if (cs_falling)
            bit_cnt <= 3'd0;
        else if (sck_rising && !cs_n_s)
            bit_cnt <= byte_done_raw ? 3'd0 : bit_cnt + 3'd1;
    end
    reg byte_done;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) byte_done <= 1'b0;
        else        byte_done <= byte_done_raw;
    end
    wire [7:0] rx_byte;
    shift_reg #(.WIDTH(8)) rx_shreg (
        .clk(clk), .rst_n(rst_n),
        .load(1'b0), .shift_en(sck_rising && !cs_n_s),
        .serial_in(si), .parallel_in(8'h00),
        .serial_out(), .parallel_out(rx_byte)
    );
    localparam S_IDLE        = 3'd0,
               S_OPCODE      = 3'd1,
               S_ADDR        = 3'd2,
               S_READ_DATA   = 3'd3,
               S_WRITE_DATA  = 3'd4,
               S_SEND_STATUS = 3'd5,
               S_WAIT_CS     = 3'd6;
    localparam OP_T_READ  = 1'b0;
    localparam OP_T_WRITE = 1'b1;
    reg [2:0] state;
    reg        tx_load;
    reg [7:0]  tx_load_val;
    wire       tx_serial;
    wire tx_active = (state == S_READ_DATA) || (state == S_SEND_STATUS);
    reg suppress_first_shift;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            suppress_first_shift <= 1'b0;
        end else if (tx_load) begin
            suppress_first_shift <= 1'b1;
        end else if (sck_falling && !cs_n_s && tx_active && suppress_first_shift) begin
            suppress_first_shift <= 1'b0;
        end
    end
    wire tx_shift_en = sck_falling && !cs_n_s && tx_active && !suppress_first_shift;
    shift_reg #(.WIDTH(8)) tx_shreg (
        .clk(clk), .rst_n(rst_n),
        .load(tx_load), .shift_en(tx_shift_en),
        .serial_in(1'b0), .parallel_in(tx_load_val),
        .serial_out(tx_serial), .parallel_out()
    );
    assign so = (cs_n_s || !tx_active) ? 1'bz : tx_serial;
    reg       op_type;
    reg       rxbuf_active;
    reg       we_d;
    reg       pending_load;
    reg       status_sel;

    // Was the CURRENT S_WRITE_DATA stream started by the LOAD TX BUFFER
    // quick-address opcode (as opposed to a plain WRITE)? Only a LOAD TX
    // BUFFER stream needs the SIDL->DLC address skip below; a plain WRITE
    // to any address (including 0x32) is generic per-byte access and must
    // NOT skip anything -- reg_bank.v still owns whether 0x33/0x34 exist,
    // spi_if.v just shouldn't assume it for ordinary WRITEs.
    reg       load_tx_active;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) we_d <= 1'b0;
        else        we_d <= we;
    end
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state          <= S_IDLE;
            addr           <= 8'h00;
            we             <= 1'b0;
            reset_pulse    <= 1'b0;
            rts_pulse      <= 1'b0;
            rxbuf_done     <= 1'b0;
            rxbuf_active   <= 1'b0;
            tx_load        <= 1'b0;
            pending_load   <= 1'b0;
            status_sel     <= 1'b0;
            load_tx_active <= 1'b0;
        end else begin
            we          <= 1'b0;
            reset_pulse <= 1'b0;
            rts_pulse   <= 1'b0;
            rxbuf_done  <= 1'b0;
            tx_load     <= 1'b0;
            if (cs_rising) begin
                state <= S_IDLE;
                if (rxbuf_active) rxbuf_done <= 1'b1;
                rxbuf_active   <= 1'b0;
                load_tx_active <= 1'b0;
            end else begin
                case (state)
                    S_IDLE: begin
                        if (cs_falling) state <= S_OPCODE;
                    end
                    S_OPCODE: begin
                        if (byte_done) begin
                            
                            if (rx_byte == OP_RESET) begin
                                reset_pulse <= 1'b1;
                                state <= S_WAIT_CS;
                            end else if (rx_byte == OP_RTS_TXB0) begin
                                rts_pulse <= 1'b1;
                                state     <= S_WAIT_CS;
                            end else if (rx_byte == OP_READSTAT) begin
                                status_sel  <= 1'b0;
                                tx_load_val <= status_byte;
                                tx_load     <= 1'b1;
                                state       <= S_SEND_STATUS;
                            end else if (rx_byte == OP_RXSTAT) begin
                                status_sel  <= 1'b1;
                                tx_load_val <= rxstatus_byte;
                                tx_load     <= 1'b1;
                                state       <= S_SEND_STATUS;
                            end else if ((rx_byte & LOADTX_MASK) == LOADTX_ID) begin
                                addr           <= rx_byte[0] ? ADDR_TXB_D0 : ADDR_TXB_SIDH;
                                op_type        <= OP_T_WRITE;
                                load_tx_active <= 1'b1;
                                state          <= S_WRITE_DATA;

                            end else if ((rx_byte & RXBUF_MASK) == RXBUF_ID) begin
                                addr         <= rx_byte[1] ? ADDR_RXB_D0 : ADDR_RXB_SIDH;
                                op_type      <= OP_T_READ;
                                rxbuf_active <= 1'b1;
                                pending_load <= 1'b1;
                                state        <= S_READ_DATA;
                                
                            end else if (rx_byte == OP_READ) begin
                                op_type <= OP_T_READ;
                                state   <= S_ADDR;
                            end else if (rx_byte == OP_WRITE) begin
                                op_type        <= OP_T_WRITE;
                                load_tx_active <= 1'b0;
                                state          <= S_ADDR;
                            end else begin
                                state <= S_WAIT_CS;
                            end
                        end
                    end
                    S_ADDR: begin
                        if (byte_done) begin
                            addr <= rx_byte;
                            if (op_type == OP_T_READ) begin
                                pending_load <= 1'b1;
                                state        <= S_READ_DATA;
                            end else begin
                                state <= S_WRITE_DATA;
                            end
                        end
                    end
                    S_READ_DATA: begin
                        if (byte_done) begin
                            addr         <= addr + 8'd1;
                            pending_load <= 1'b1;
                        end
                    end
                    S_WRITE_DATA: begin
                        if (byte_done) begin
                            wdata <= rx_byte;
                            we    <= 1'b1;
                        end else if (we_d) begin
                            // LOAD TX BUFFER's SIDL->DLC skip: only applies
                            // when this stream started at LOAD TX BUFFER's
                            // SIDH entry point and has walked forward to
                            // SIDL. A plain WRITE landing on 0x32 (rare, but
                            // possible if a host explicitly targets it)
                            // increments normally, since load_tx_active is
                            // false for that path.
                            if (load_tx_active && (addr == ADDR_TXB_SIDL))
                                addr <= ADDR_TXB_DLC;
                            else
                                addr <= addr + 8'd1;
                        end
                    end
                    S_SEND_STATUS: begin
                        if (byte_done) begin
                            tx_load_val <= status_sel ? rxstatus_byte : status_byte;
                            tx_load     <= 1'b1;
                        end
                    end
                    S_WAIT_CS: begin
                    end
                    default: state <= S_IDLE;
                endcase
                if (pending_load) begin
                    tx_load_val  <= rdata;
                    tx_load      <= 1'b1;
                    pending_load <= 1'b0;
                end
            end
        end
    end
endmodule
// // `include "../common/can_defs.vh"

// // // =============================================================================
// // // Module : spi_if
// // // -----------------------------------------------------------------------------
// // // Synchronous SPI Slave Interface Module for CAN Controller.
// // // Operates on system clock (clk).
// // // =============================================================================

// // module spi_if (
// //     input  wire        clk,
// //     input  wire        rst_n,

// //     // SPI Physical Pins
// //     input  wire        sck,
// //     input  wire        si,
// //     output wire        so,
// //     input  wire        cs_n,

// //     // Internal Register & Top Interface
// //     output reg  [7:0]  addr,
// //     output reg  [7:0]  wdata,
// //     input  wire [7:0]  rdata,
// //     output reg         we,
// //     output reg         reset_pulse,
// //     output reg         rts_pulse,
// //     output reg         rxbuf_done,
// //     input  wire [7:0]  status_byte,
// //     input  wire [7:0]  rxstatus_byte
// // );

// //     // Standard Opcodes
// //     localparam [7:0] OP_WRITE       = 8'h02;
// //     localparam [7:0] OP_READ        = 8'h03;
// //     localparam [7:0] OP_BIT_MODIFY  = 8'h05;
// //     localparam [7:0] OP_RESET       = 8'hC0;
// //     localparam [7:0] OP_RTS         = 8'h80;
// //     localparam [7:0] OP_READ_STATUS = 8'hA0;
// //     localparam [7:0] OP_RX_STATUS   = 8'hB0;

// //     // Double-flop synchronizers for SCK and SI
// //     reg [1:0] sck_sync;
// //     reg [1:0] si_sync;

// //     always @(posedge clk or negedge rst_n) begin
// //         if (!rst_n) begin
// //             sck_sync <= 2'b00;
// //             si_sync  <= 2'b00;
// //         end else begin
// //             sck_sync <= {sck_sync[0], sck};
// //             si_sync  <= {si_sync[0], si};
// //         end
// //     end

// //     wire sck_rising  = (sck_sync == 2'b01);
// //     wire sck_falling = (sck_sync == 2'b10);

// //     // Direct CS_N evaluation so MISO tri-state activates with zero delay
// //     wire cs_active = ~cs_n;

// //     // Internal SPI Registers
// //     reg [2:0] bit_cnt;
// //     reg [2:0] byte_cnt;
// //     reg [7:0] cmd_reg;
// //     reg [7:0] rx_shift;
// //     reg [7:0] tx_shift;
// //     reg [7:0] mask_reg;
// //     reg       load_rdata_req;

// //     // Tri-state MISO driver
// //     wire is_status_op = (cmd_reg == OP_READ_STATUS) || (cmd_reg == OP_RX_STATUS);
// //     wire is_read_op   = (cmd_reg == OP_READ) || is_status_op;

// //     wire drive_miso   = cs_active && (
// //                           (is_status_op && byte_cnt >= 3'd1) ||
// //                           (cmd_reg == OP_READ && byte_cnt >= 3'd2)
// //                         );

// //     assign so = drive_miso ? tx_shift[7] : 1'bz;

// //     wire [7:0] incoming_byte = {rx_shift[6:0], si_sync[0]};

// //     always @(posedge clk or negedge rst_n) begin
// //         if (!rst_n) begin
// //             bit_cnt        <= 3'd0;
// //             byte_cnt       <= 3'd0;
// //             cmd_reg        <= 8'h00;
// //             rx_shift       <= 8'h00;
// //             tx_shift       <= 8'h00;
// //             mask_reg       <= 8'h00;
// //             addr           <= 8'h00;
// //             wdata          <= 8'h00;
// //             we             <= 1'b0;
// //             reset_pulse    <= 1'b0;
// //             rts_pulse      <= 1'b0;
// //             rxbuf_done     <= 1'b0;
// //             load_rdata_req <= 1'b0;
// //         end else if (!cs_active) begin
// //             bit_cnt        <= 3'd0;
// //             byte_cnt       <= 3'd0;
// //             cmd_reg        <= 8'h00;
// //             we             <= 1'b0;
// //             reset_pulse    <= 1'b0;
// //             rts_pulse      <= 1'b0;
// //             rxbuf_done     <= 1'b0;
// //             load_rdata_req <= 1'b0;
// //         end else begin
// //             // Single-cycle default pulses
// //             reset_pulse <= 1'b0;
// //             rts_pulse   <= 1'b0;
// //             rxbuf_done  <= 1'b0;

// //             // Deassert write enable after 1 system clock cycle and auto-increment address
// //             if (we) begin
// //                 we   <= 1'b0;
// //                 addr <= addr + 8'd1;
// //             end

// //             // Load rdata into tx_shift 1 clock cycle after addr is updated
// //             if (load_rdata_req) begin
// //                 tx_shift       <= rdata;
// //                 load_rdata_req <= 1'b0;
// //             end

// //             // -----------------------------------------------------------------
// //             // Sample MOSI on SCK Rising Edge
// //             // -----------------------------------------------------------------
// //             if (sck_rising) begin
// //                 rx_shift <= incoming_byte;

// //                 if (bit_cnt == 3'd7) begin
// //                     bit_cnt <= 3'd0;

// //                     case (byte_cnt)
// //                         3'd0: begin
// //                             // Byte 0: Opcode
// //                             cmd_reg  <= incoming_byte;
// //                             byte_cnt <= 3'd1;

// //                             case (incoming_byte)
// //                                 OP_RESET:       reset_pulse <= 1'b1;
// //                                 OP_RTS:         rts_pulse   <= 1'b1;
// //                                 OP_READ_STATUS: tx_shift    <= status_byte;
// //                                 OP_RX_STATUS:   tx_shift    <= rxstatus_byte;
// //                                 default: ;
// //                             endcase
// //                         end

// //                         3'd1: begin
// //                             // Byte 1: Address
// //                             if (cmd_reg == OP_READ) begin
// //                                 addr           <= incoming_byte;
// //                                 byte_cnt       <= 3'd2;
// //                                 load_rdata_req <= 1'b1;
// //                             end else if (cmd_reg == OP_WRITE || cmd_reg == OP_BIT_MODIFY) begin
// //                                 addr     <= incoming_byte;
// //                                 byte_cnt <= 3'd2;
// //                             end
// //                         end

// //                         3'd2: begin
// //                             // Byte 2: Data / Mask
// //                             case (cmd_reg)
// //                                 OP_WRITE: begin
// //                                     wdata <= incoming_byte;
// //                                     we    <= 1'b1;
// //                                 end
// //                                 OP_READ: begin
// //                                     addr           <= addr + 8'd1;
// //                                     load_rdata_req <= 1'b1;
// //                                 end
// //                                 OP_BIT_MODIFY: begin
// //                                     mask_reg <= incoming_byte;
// //                                     byte_cnt <= 3'd3;
// //                                 end
// //                                 default: ;
// //                             endcase
// //                         end

// //                         3'd3: begin
// //                             // Byte 3: Bit Modify Data
// //                             if (cmd_reg == OP_BIT_MODIFY) begin
// //                                 wdata <= (rdata & ~mask_reg) | (incoming_byte & mask_reg);
// //                                 we    <= 1'b1;
// //                             end
// //                         end

// //                         default: ;
// //                     endcase
// //                 end else begin
// //                     bit_cnt <= bit_cnt + 3'd1;
// //                 end
// //             end

// //             // -----------------------------------------------------------------
// //             // Shift MISO on SCK Falling Edge
// //             // -----------------------------------------------------------------
// //             if (sck_falling) begin
// //                 // FIX: Only shift if bit_cnt != 0 to prevent losing the MSB right after a byte load
// //                 if (is_read_op && drive_miso && !load_rdata_req && bit_cnt != 3'd0) begin
// //                     tx_shift <= {tx_shift[6:0], 1'b0};
// //                 end
// //             end
// //         end
// //     end

// // endmodule

// `include "../common/can_defs.vh"

// // =============================================================================
// // Module : spi_if
// // -----------------------------------------------------------------------------
// // Synchronous SPI Slave Interface Module for CAN Controller.
// // Operates on system clock (clk). Supports standard MCP2515 opcodes.
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

//     // Standard Opcodes & Bitmasks
//     localparam [7:0] OP_WRITE       = 8'h02;
//     localparam [7:0] OP_READ        = 8'h03;
//     localparam [7:0] OP_BIT_MODIFY  = 8'h05;
//     localparam [7:0] OP_RESET       = 8'hC0;
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
//     reg       read_rx_data_seen;

//     // Opcodes categorization
//     wire is_status_op  = (cmd_reg == OP_READ_STATUS) || (cmd_reg == OP_RX_STATUS);
//     // Reduced design supports the two RXB0 READ RX BUFFER entry points:
//     // 0x90 -> RXB0SIDH, 0x92 -> RXB0D0. RXB1 opcodes are out of scope.
//     wire is_read_rx    = (cmd_reg == 8'h90) || (cmd_reg == 8'h92);
//     wire is_read_op    = (cmd_reg == OP_READ) || is_status_op || is_read_rx;
//     wire is_load_tx    = (cmd_reg == 8'h40) || (cmd_reg == 8'h41);

//     wire drive_miso    = cs_active && (
//                            (is_status_op && byte_cnt >= 3'd1) ||
//                            (cmd_reg == OP_READ && byte_cnt >= 3'd2) ||
//                            (is_read_rx && byte_cnt >= 3'd2)
//                          );

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
//             rxbuf_done       <= 1'b0;
//             load_rdata_req   <= 1'b0;
//             read_rx_data_seen <= 1'b0;
//         end else if (!cs_active) begin
//             bit_cnt        <= 3'd0;
//             byte_cnt       <= 3'd0;
//             cmd_reg        <= 8'h00;
//             we             <= 1'b0;
//             reset_pulse    <= 1'b0;
//             rts_pulse      <= 1'b0;
//             // READ RX BUFFER is considered complete only if at least one data
//             // byte was clocked out before CS rose. Generate a single system-clock
//             // pulse so Control Logic can clear RX0IF and the RX buffer full flag.
//             rxbuf_done       <= read_rx_data_seen;
//             load_rdata_req   <= 1'b0;
//             read_rx_data_seen <= 1'b0;
//         end else begin
//             // Single-cycle default pulses
//             reset_pulse <= 1'b0;
//             rts_pulse   <= 1'b0;
//             rxbuf_done  <= 1'b0;

//             // Deassert write enable after one system-clock cycle. LOAD TX
//             // uses the reduced register map, so skip the omitted EID8/EID0
//             // addresses between SIDL (0x32) and DLC (0x35).
//             if (we) begin
//                 we <= 1'b0;
//                 if (is_load_tx && (addr == 8'h32))
//                     addr <= 8'h35;
//                 else
//                     addr <= addr + 8'd1;
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
                            
//                             if (incoming_byte == OP_RESET) begin
//                                 reset_pulse <= 1'b1;
//                             end else if ((incoming_byte == 8'h90) || (incoming_byte == 8'h92)) begin
//                                 // READ RX BUFFER, reduced to RXB0 only. 0x90 starts
//                                 // at SIDH; 0x92 starts directly at D0.
//                                 addr             <= (incoming_byte == 8'h90) ? 8'h61 : 8'h66;
//                                 byte_cnt         <= 3'd2;
//                                 load_rdata_req   <= 1'b1;
//                                 read_rx_data_seen <= 1'b0;
//                             end else if (incoming_byte == 8'h81) begin
//                                 // Reduced design has TXB0 only; reject TXB1/TXB2
//                                 // selection opcodes instead of aliasing them to TXB0.
//                                 rts_pulse <= 1'b1;
//                             end else if ((incoming_byte == 8'h40) || (incoming_byte == 8'h41)) begin
//                                 // LOAD TX BUFFER has two entry points in the single
//                                 // TX buffer: 0x40 starts at SIDH, 0x41 starts at D0.
//                                 addr     <= (incoming_byte == 8'h40) ? 8'h31 : 8'h36;
//                                 byte_cnt <= 3'd2; // No explicit address byte
//                             end else begin
//                                 byte_cnt <= 3'd1;
//                                 case (incoming_byte)
//                                     OP_READ_STATUS: tx_shift <= status_byte;
//                                     OP_RX_STATUS:   tx_shift <= rxstatus_byte;
//                                     default: ;
//                                 endcase
//                             end
//                         end

//                         3'd1: begin
//                             // Byte 1: address for normal READ/WRITE. Status
//                             // commands remain in this state and reload the live
//                             // status byte after each completed output byte.
//                             if (cmd_reg == OP_READ) begin
//                                 addr           <= incoming_byte;
//                                 byte_cnt       <= 3'd2;
//                                 load_rdata_req <= 1'b1;
//                             end else if (cmd_reg == OP_WRITE || cmd_reg == OP_BIT_MODIFY) begin
//                                 addr     <= incoming_byte;
//                                 byte_cnt <= 3'd2;
//                             end else if (cmd_reg == OP_READ_STATUS) begin
//                                 tx_shift <= status_byte;
//                             end else if (cmd_reg == OP_RX_STATUS) begin
//                                 tx_shift <= rxstatus_byte;
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
//                                 8'h90, 8'h92: begin
//                                     // One RX byte has completed. Continue sequential
//                                     // read-back and remember to pulse rxbuf_done when
//                                     // CS is released.
//                                     read_rx_data_seen <= 1'b1;
//                                     addr              <= addr + 8'd1;
//                                     load_rdata_req    <= 1'b1;
//                                 end
//                                 OP_BIT_MODIFY: begin
//                                     mask_reg <= incoming_byte;
//                                     byte_cnt <= 3'd3;
//                                 end
//                                 default: begin
//                                     // Handles continuous streaming for Load TX Buffer (0x40+)
//                                     if (is_load_tx) begin
//                                         wdata <= incoming_byte;
//                                         we    <= 1'b1;
//                                     end
//                                 end
//                             endcase
//                         end

//                         3'd3: begin
//                             // Byte 3: Bit Modify Data or continued Load TX Buffer stream
//                             if (cmd_reg == OP_BIT_MODIFY) begin
//                                 wdata <= (rdata & ~mask_reg) | (incoming_byte & mask_reg);
//                                 we    <= 1'b1;
//                             end else if (is_load_tx) begin
//                                 wdata <= incoming_byte;
//                                 we    <= 1'b1;
//                             end
//                         end

//                         default: begin
//                             // Handle any remaining bytes in a continuous load stream
//                             if (is_load_tx) begin
//                                 wdata <= incoming_byte;
//                                 we    <= 1'b1;
//                             end
//                         end
//                     endcase
//                 end else begin
//                     bit_cnt <= bit_cnt + 3'd1;
//                 end
//             end

//             // -----------------------------------------------------------------
//             // Shift MISO on SCK Falling Edge
//             // -----------------------------------------------------------------
//             if (sck_falling) begin
//                 if (is_read_op && drive_miso && !load_rdata_req && bit_cnt != 3'd0) begin
//                     tx_shift <= {tx_shift[6:0], 1'b0};
//                 end
//             end
//         end
//     end

// endmodule
