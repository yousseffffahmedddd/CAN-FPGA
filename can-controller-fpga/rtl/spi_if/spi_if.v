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
    localparam [7:0] RXBUF_MASK  = 8'hF9, RXBUF_ID  = 8'h90;
    localparam [7:0] OP_RTS_TXB0 = 8'h81;
    localparam [7:0] ADDR_TXB_SIDH = 8'h31, ADDR_TXB_D0 = 8'h36;
    localparam [7:0] ADDR_RXB_SIDH = 8'h61, ADDR_RXB_D0 = 8'h66;
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
            rts_pulse    <= 1'b0;
            rxbuf_done   <= 1'b0;
            rxbuf_active <= 1'b0;
            tx_load      <= 1'b0;
            pending_load <= 1'b0;
            status_sel   <= 1'b0;
        end else begin
            we          <= 1'b0;
            reset_pulse <= 1'b0;
            rts_pulse   <= 1'b0;
            rxbuf_done  <= 1'b0;
            tx_load     <= 1'b0;
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
                                addr    <= rx_byte[0] ? ADDR_TXB_D0 : ADDR_TXB_SIDH;
                                op_type <= OP_T_WRITE;
                                state   <= S_WRITE_DATA;
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
                                op_type <= OP_T_WRITE;
                                state   <= S_ADDR;
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
