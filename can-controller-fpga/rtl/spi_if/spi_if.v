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
    output reg  [2:0]   rts_pulse,
    output reg          rxbuf_done,
    output reg          rxbuf_sel,
    output reg  [7:0]   bitmod_mask,
    output reg          bitmod_we,

    input  wire [7:0]  status_byte,
    input  wire [7:0]  rxstatus_byte
);

    localparam [7:0] OP_RESET    = 8'hC0;
    localparam [7:0] OP_READ     = 8'h03;
    localparam [7:0] OP_WRITE    = 8'h02;
    localparam [7:0] OP_READSTAT = 8'hA0;
    localparam [7:0] OP_RXSTAT   = 8'hB0;
    localparam [7:0] OP_BITMOD   = 8'h05;
    localparam [7:0] LOADTX_MASK = 8'hF8, LOADTX_ID = 8'h40;
    localparam [7:0] RXBUF_MASK  = 8'hF9, RXBUF_ID  = 8'h90;
    localparam [7:0] RTS_MASK    = 8'hF8, RTS_ID    = 8'h80;

    localparam [7:0] ADDR_TXB0CTRL = 8'h30;
    localparam [7:0] ADDR_TXB1CTRL = 8'h40;
    localparam [7:0] ADDR_TXB2CTRL = 8'h50;
    localparam [7:0] ADDR_RXB0CTRL = 8'h60;
    localparam [7:0] ADDR_RXB1CTRL = 8'h70;
    localparam [7:0] ADDR_CANINTE  = 8'h2B;
    localparam [7:0] ADDR_CANINTF  = 8'h2C;
    localparam [7:0] ADDR_EFLG     = 8'h2D;

    localparam [7:0] ADDR_TXB0SIDH = 8'h31, ADDR_TXB0D0 = 8'h36;
    localparam [7:0] ADDR_TXB1SIDH = 8'h41, ADDR_TXB1D0 = 8'h46;
    localparam [7:0] ADDR_TXB2SIDH = 8'h51, ADDR_TXB2D0 = 8'h56;
    localparam [7:0] ADDR_RXB0SIDH = 8'h61, ADDR_RXB0D0 = 8'h66;
    localparam [7:0] ADDR_RXB1SIDH = 8'h71, ADDR_RXB1D0 = 8'h76;

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

    localparam S_IDLE        = 4'd0,
               S_OPCODE      = 4'd1,
               S_ADDR        = 4'd2,
               S_READ_DATA   = 4'd3,
               S_WRITE_DATA  = 4'd4,
               S_SEND_STATUS = 4'd5,
               S_GET_MASK    = 4'd6,
               S_GET_DATA    = 4'd7,
               S_WAIT_CS     = 4'd8;

    localparam OP_T_READ = 2'd0, OP_T_WRITE = 2'd1, OP_T_BITMOD = 2'd2;

    reg [3:0] state;

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

    reg so_r;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) so_r <= 1'b0;
        else        so_r <= tx_serial;
    end
    assign so = (cs_n_s || !tx_active) ? 1'bz : so_r;

    function is_bitmod_legal(input [7:0] a);
    begin
        is_bitmod_legal =
           (a == ADDR_TXB0CTRL) || (a == ADDR_TXB1CTRL) || (a == ADDR_TXB2CTRL) ||
           (a == ADDR_RXB0CTRL) || (a == ADDR_RXB1CTRL) ||
           (a == ADDR_CANINTE)  || (a == ADDR_CANINTF)  || (a == ADDR_EFLG) ||
           (a == 8'h0F) || (a == 8'h1F) || (a == 8'h2F) || (a == 8'h3F) ||
           (a == 8'h4F) || (a == 8'h5F) || (a == 8'h6F) || (a == 8'h7F);
    end
    endfunction

    reg [1:0] op_type;
    reg       rxbuf_active;
    reg       we_d;
    reg       pending_load;
    reg       status_sel;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) we_d <= 1'b0;
        else        we_d <= we;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= S_IDLE;
            addr         <= 8'h00;
            we           <= 1'b0;
            reset_pulse  <= 1'b0;
            rts_pulse    <= 3'b000;
            rxbuf_done   <= 1'b0;
            rxbuf_sel    <= 1'b0;
            rxbuf_active <= 1'b0;
            tx_load      <= 1'b0;
            pending_load <= 1'b0;
            bitmod_we    <= 1'b0;
            status_sel   <= 1'b0;
        end else begin
            we          <= 1'b0;
            reset_pulse <= 1'b0;
            rts_pulse   <= 3'b000;
            rxbuf_done  <= 1'b0;
            tx_load     <= 1'b0;
            bitmod_we   <= 1'b0;

            if (cs_rising) begin
                state <= S_IDLE;
                if (rxbuf_active) rxbuf_done <= 1'b1;
                rxbuf_active <= 1'b0;
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

                            // end else if ((rx_byte & RTS_MASK) == RTS_ID) begin
                            //     rts_pulse <= rx_byte[2:0];
                            //     state <= S_WAIT_CS;
                            end else if ((rx_byte & RTS_MASK) == RTS_ID) begin
                            if (rx_byte[2:0] != 3'b000) begin
                            rts_pulse <= rx_byte[2:0];
                            end
                            state <= S_WAIT_CS;

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
                                op_type <= OP_T_WRITE;
                                case (rx_byte[2:1])
                                    2'b00: begin addr <= rx_byte[0] ? ADDR_TXB0D0 : ADDR_TXB0SIDH; state <= S_WRITE_DATA; end
                                    2'b01: begin addr <= rx_byte[0] ? ADDR_TXB1D0 : ADDR_TXB1SIDH; state <= S_WRITE_DATA; end
                                    2'b10: begin addr <= rx_byte[0] ? ADDR_TXB2D0 : ADDR_TXB2SIDH; state <= S_WRITE_DATA; end
                                    default: state <= S_WAIT_CS;
                                endcase

                            end else if ((rx_byte & RXBUF_MASK) == RXBUF_ID) begin
                                addr         <= rx_byte[2] ? (rx_byte[1] ? ADDR_RXB1D0 : ADDR_RXB1SIDH)
                                                            : (rx_byte[1] ? ADDR_RXB0D0 : ADDR_RXB0SIDH);
                                op_type      <= OP_T_READ;
                                rxbuf_active <= 1'b1;
                                rxbuf_sel    <= rx_byte[2];
                                pending_load <= 1'b1;
                                state        <= S_READ_DATA;

                            end else if (rx_byte == OP_READ) begin
                                op_type <= OP_T_READ;
                                state   <= S_ADDR;

                            end else if (rx_byte == OP_WRITE) begin
                                op_type <= OP_T_WRITE;
                                state   <= S_ADDR;

                            end else if (rx_byte == OP_BITMOD) begin
                                op_type <= OP_T_BITMOD;
                                state   <= S_ADDR;

                            end else begin
                                state <= S_WAIT_CS;
                            end
                        end
                    end

                    S_ADDR: begin
                        if (byte_done) begin
                            addr <= rx_byte;
                            case (op_type)
                                OP_T_READ: begin
                                    pending_load <= 1'b1;
                                    state <= S_READ_DATA;
                                end
                                OP_T_WRITE: state <= S_WRITE_DATA;
                                OP_T_BITMOD: state <= S_GET_MASK;
                                default: state <= S_WAIT_CS;
                            endcase
                        end
                    end

                    S_GET_MASK: begin
                        if (byte_done) begin
                            bitmod_mask <= is_bitmod_legal(addr) ? rx_byte : 8'hFF;
                            state <= S_GET_DATA;
                        end
                    end

                    S_GET_DATA: begin
                        if (byte_done) begin
                            wdata     <= rx_byte;
                            bitmod_we <= 1'b1;
                            state     <= S_WAIT_CS;
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
