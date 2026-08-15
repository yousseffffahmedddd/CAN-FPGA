`timescale 1ns/1ps

module tb_can_top_complete;

    reg  clk;
    reg  rst_n;
    reg  spi_sck;
    reg  spi_cs_n;
    reg  spi_mosi;
    wire spi_miso;

    wire tx_can;
    wire tx_en;
    wire int_n;
    wire rx_pin;

    localparam [7:0] SPI_WRITE = 8'h02;
    localparam [7:0] SPI_READ  = 8'h03;
    localparam [7:0] SPI_RTS0  = 8'h81;
    localparam [7:0] SPI_RESET = 8'hC0;

    localparam [7:0] ADDR_CANCTRL  = 8'h0F;
    localparam [7:0] ADDR_BFPCTRL  = 8'h0C;
    localparam [7:0] ADDR_CANINTE  = 8'h2B;
    localparam [7:0] ADDR_TXB0SIDH = 8'h31;
    localparam [7:0] ADDR_TXB0SIDL = 8'h32;
    localparam [7:0] ADDR_TXB0DLC  = 8'h35;
    localparam [7:0] ADDR_TXB0D0   = 8'h36;

    integer checks;
    integer errors;
    reg [7:0] rd;

    reg saw_rts_pulse;
    reg saw_tx_start;
    reg saw_tx_ready;

    can_top dut (
        .clk    (clk),
        .rst_n  (rst_n),
        .sck    (spi_sck),
        .si     (spi_mosi),
        .so     (spi_miso),
        .cs_n   (spi_cs_n),
        .rx_pin (rx_pin),
        .tx_can (tx_can),
        .tx_en  (tx_en),
        .int_n  (int_n)
    );

    // Recessive bus when idle; loop back the transmitted level while enabled.
    assign rx_pin = tx_en ? tx_can : 1'b1;

    always #500 clk = ~clk;

    always @(posedge clk) begin
        if (!rst_n) begin
            saw_rts_pulse <= 1'b0;
            saw_tx_start  <= 1'b0;
            saw_tx_ready  <= 1'b0;
        end else begin
            if (dut.u_spi_if.rts_pulse)
                saw_rts_pulse <= 1'b1;
            if (dut.u_control_logic.tx_start)
                saw_tx_start <= 1'b1;
            if (dut.u_tx_buffer.ready)
                saw_tx_ready <= 1'b1;
        end
    end

    task check;
        input condition;
        input [8*100-1:0] message;
        begin
            checks = checks + 1;
            if (condition)
                $display("[PASS] %0t : %0s", $time, message);
            else begin
                errors = errors + 1;
                $display("[FAIL] %0t : %0s", $time, message);
            end
        end
    endtask

    task spi_transfer_byte;
        input  [7:0] data_in;
        output [7:0] data_out;
        integer i;
        reg [7:0] sample_buf;
        begin
            sample_buf = 8'h00;
            for (i = 7; i >= 0; i = i - 1) begin
                spi_mosi = data_in[i];
                #3000;
                spi_sck = 1'b1;
                #1500;
                sample_buf[i] = spi_miso;
                #1500;
                spi_sck = 1'b0;
                #3000;
            end
            data_out = sample_buf;
        end
    endtask

    task spi_write;
        input [7:0] a;
        input [7:0] d;
        reg [7:0] dummy;
        begin
            spi_cs_n = 1'b0;
            #4000;
            spi_transfer_byte(SPI_WRITE, dummy);
            spi_transfer_byte(a, dummy);
            spi_transfer_byte(d, dummy);
            #4000;
            spi_cs_n = 1'b1;
            #6000;
        end
    endtask

    task spi_read;
        input  [7:0] a;
        output [7:0] d;
        reg [7:0] dummy;
        begin
            spi_cs_n = 1'b0;
            #4000;
            spi_transfer_byte(SPI_READ, dummy);
            spi_transfer_byte(a, dummy);
            spi_transfer_byte(8'h00, d);
            #4000;
            spi_cs_n = 1'b1;
            #6000;
        end
    endtask

    task spi_command;
        input [7:0] opcode;
        reg [7:0] dummy;
        begin
            spi_cs_n = 1'b0;
            #4000;
            spi_transfer_byte(opcode, dummy);
            #4000;
            spi_cs_n = 1'b1;
            #6000;
        end
    endtask

    initial begin
        clk       = 1'b0;
        rst_n     = 1'b0;
        spi_sck   = 1'b0;
        spi_cs_n  = 1'b1;
        spi_mosi  = 1'b0;
        checks    = 0;
        errors    = 0;
        saw_rts_pulse = 1'b0;
        saw_tx_start  = 1'b0;
        saw_tx_ready  = 1'b0;

        #4000;
        rst_n = 1'b1;
        #8000;

        $display("============================================================");
        $display("CAN TOP INTEGRATION SMOKE TEST");
        $display("============================================================");

        // Register Bank is reachable through SPI and remains in Normal mode.
        spi_write(ADDR_CANCTRL, 8'hFF);
        spi_read(ADDR_CANCTRL, rd);
        check(rd === 8'h1F, "CANCTRL write reaches Register Bank and mode stays Normal");

        spi_write(ADDR_BFPCTRL, 8'h5A);
        spi_read(ADDR_BFPCTRL, rd);
        check(rd === 8'h5A, "BFPCTRL register is reachable through can_top");

        spi_write(ADDR_CANINTE, 8'hFF);
        spi_read(ADDR_CANINTE, rd);
        check(rd === 8'hA5, "CANINTE reduced interrupt mask is connected");

        // Load a complete TX message through the Register Bank.
        spi_write(ADDR_TXB0SIDH, 8'hAA);
        spi_write(ADDR_TXB0SIDL, 8'hA0); // ID 0x555
        spi_write(ADDR_TXB0DLC,  8'h04);
        spi_write(ADDR_TXB0D0 + 0, 8'h11);
        spi_write(ADDR_TXB0D0 + 1, 8'h22);
        spi_write(ADDR_TXB0D0 + 2, 8'h33);
        spi_write(ADDR_TXB0D0 + 3, 8'h44);
        #5000;

        check(dut.u_reg_bank.txb0_id === 11'h555,
              "Register Bank assembled TX standard ID");
        check(dut.u_reg_bank.txb0_data[63:32] === 32'h1122_3344,
              "Register Bank D0-D3 byte order matches Protocol Engine serializer");
        check(dut.u_tx_buffer.id === 11'h555,
              "TX buffer receives Register Bank ID staging");
        check(dut.u_tx_buffer.data[63:32] === 32'h1122_3344,
              "TX buffer receives correctly ordered payload");

        // RTS must travel SPI -> Control Logic -> TX buffer/Protocol Engine.
        spi_command(SPI_RTS0);
        #5000;
        check(saw_rts_pulse === 1'b1, "SPI decoded the RTS opcode");
        check(saw_tx_start  === 1'b1, "Control Logic produced tx_start from RTS");
        check(saw_tx_ready  === 1'b1, "TX buffer observed the transmission request");

        // Software RESET must travel through Control Logic and reset mutable state.
        spi_command(SPI_RESET);
        #5000;
        check(dut.u_reg_bank.txb0_txreq === 1'b0,
              "software RESET clears Register Bank TXREQ state");
        check(dut.u_reg_bank.caninte === 8'h00,
              "software RESET clears interrupt enables");
        check(int_n === 1'b1, "INT is inactive after software RESET");

        $display("");
        $display("============================================================");
        $display("CAN TOP TESTS: %0d checks, %0d errors", checks, errors);
        if (errors == 0)
            $display("RESULT: PASS");
        else
            $display("RESULT: FAIL");
        $display("============================================================");
        $finish;
    end

endmodule
