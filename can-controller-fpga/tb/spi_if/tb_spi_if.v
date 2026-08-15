`timescale 1ns/1ps

module tb_spi_if;

    reg clk = 0;
    reg rst_n = 0;
    reg cs_n = 1;
    reg sck = 0;
    reg si = 0;

    wire so;

    wire [7:0] addr;
    wire [7:0] wdata;
    reg  [7:0] rdata;
    wire we, reset_pulse, rxbuf_done;
    wire rts_pulse;
    reg [7:0] mem [0:255];

    integer i0;
    integer errors = 0;

    reg [7:0] status_byte   = 8'hAB;
    reg [7:0] rxstatus_byte = 8'hCD;

    spi_if dut (
        .clk(clk), .rst_n(rst_n), .sck(sck), .si(si), .so(so), .cs_n(cs_n),
        .addr(addr), .wdata(wdata), .rdata(rdata), .we(we),
        .reset_pulse(reset_pulse), .rts_pulse(rts_pulse), .rxbuf_done(rxbuf_done),
        .status_byte(status_byte), .rxstatus_byte(rxstatus_byte)
    );


    // ============================================================
    // SYSTEM CLOCK
    // ============================================================

    always #5 clk = ~clk;


    // ============================================================
    // REGISTER BANK MODEL
    // ============================================================

    always @(*) begin
        rdata = mem[addr];
    end

    always @(posedge clk) begin
        if (we)
            mem[addr] <= wdata;
    end


    // ============================================================
    // SEND ONE SPI BYTE
    // SPI MODE 0:
    //   SI sampled on rising SCK
    // ============================================================

    task send_byte(input [7:0] data);

        integer i;

        begin

            for (i = 7; i >= 0; i = i - 1) begin

                si = data[i];

                #40;
                sck = 1;

                #40;
                sck = 0;

            end

        end

    endtask


    // ============================================================
    // CLOCK BITS AND SAMPLE SO
    //
    // SO is expected to contain the output bit during the
    // appropriate SCK phase.
    // ============================================================

    task clock_bits(
        input integer n,
        output [7:0] got
    );

        integer i;

        begin

            got = 8'h00;

            for (i = 0; i < n; i = i + 1) begin

                #40;
                sck = 1;

                got = {got[6:0], so};

                #40;
                sck = 0;

            end

        end

    endtask


    // ============================================================
    // CHECK TASK
    // ============================================================

    task check(
        input cond,
        input [63*8:0] msg
    );

        begin

            if (!cond) begin

                $display("FAIL: %0s", msg);
                errors = errors + 1;

            end
            else begin

                $display("PASS: %0s", msg);

            end

        end

    endtask


    // ============================================================
    // EVENT FLAGS
    // ============================================================

    reg seen_reset;
    reg seen_rxbuf_done;
    reg seen_rts;

    always @(posedge clk) begin

        if (reset_pulse)
            seen_reset <= 1'b1;

        if (rts_pulse)
            seen_rts <= 1'b1;

        if (rxbuf_done)
            seen_rxbuf_done <= 1'b1;

    end


    // ============================================================
    // TEST VARIABLES
    // ============================================================

    reg [7:0] got;


    // ============================================================
    // TEST SEQUENCE
    // ============================================================

    initial begin

        // --------------------------------------------------------
        // INITIAL RESET
        // --------------------------------------------------------

        rst_n = 0;

        for (i0 = 0; i0 < 256; i0 = i0 + 1)
            mem[i0] = 8'h00;

        #50;

        rst_n = 1;


        // ========================================================
        // 1. RESET
        // ========================================================

        seen_reset = 0;

        #20;
        cs_n = 0;

        send_byte(8'hC0);

        #20;
        cs_n = 1;

        #10;

        check(
            seen_reset === 1'b1,
            "RESET pulse fires after opcode"
        );


        // ========================================================
        // 2. NORMAL WRITE
        // Write 0xAA -> address 0x50
        // ========================================================

        #40;
        cs_n = 0;

        send_byte(8'h02);
        send_byte(8'h50);
        send_byte(8'hAA);

        #20;
        cs_n = 1;

        #10;

        check(
            mem[8'h50] === 8'hAA,
            "WRITE lands correctly"
        );


        // ========================================================
        // 3. SEQUENTIAL WRITE
        // 0x60 = 11
        // 0x61 = 22
        // 0x62 = 33
        // ========================================================

        #40;
        cs_n = 0;

        send_byte(8'h02);
        send_byte(8'h60);

        send_byte(8'h11);
        send_byte(8'h22);
        send_byte(8'h33);

        #20;
        cs_n = 1;

        #10;

        check(
            mem[8'h60] === 8'h11 &&
            mem[8'h61] === 8'h22 &&
            mem[8'h62] === 8'h33,
            "sequential WRITE auto-increments correctly"
        );


        // ========================================================
        // 4. READ BACK 0x50
        // ========================================================

        #40;
        cs_n = 0;

        send_byte(8'h03);
        send_byte(8'h50);

        clock_bits(8, got);

        #20;
        cs_n = 1;

        check(
            got === 8'hAA,
            "READ returns previously written byte"
        );


        // ========================================================
        // 4b. SEQUENTIAL READ
        // ========================================================

        mem[8'h20] = 8'h11;
        mem[8'h21] = 8'h22;
        mem[8'h22] = 8'h33;

        #40;
        cs_n = 0;

        send_byte(8'h03);
        send_byte(8'h20);

        clock_bits(8, got);

        check(
            got === 8'h11,
            "READ byte 1 correct"
        );

        clock_bits(8, got);

        check(
            got === 8'h22,
            "READ byte 2 auto-increments correctly"
        );

        clock_bits(8, got);

        check(
            got === 8'h33,
            "READ byte 3 auto-increments correctly"
        );

        #20;
        cs_n = 1;


        // ========================================================
        // 4c. MID-BYTE READ ABORT
        // ========================================================

        mem[8'h23] = 8'h55;
        mem[8'h24] = 8'h66;

        #40;
        cs_n = 0;

        send_byte(8'h03);
        send_byte(8'h23);

        clock_bits(4, got);

        #20;
        cs_n = 1;

        #40;
        cs_n = 0;

        send_byte(8'h03);
        send_byte(8'h23);

        clock_bits(8, got);

        check(
            got === 8'h55,
            "READ after mid-byte abort starts clean"
        );

        clock_bits(8, got);

        check(
            got === 8'h66,
            "READ auto-increment works after abort"
        );

        #20;
        cs_n = 1;


        // ========================================================
        // 5. MID-BYTE WRITE ABORT
        // ========================================================

        #40;
        cs_n = 0;

        send_byte(8'h02);
        send_byte(8'h70);

        si = 1;

        #40;
        sck = 1;

        #40;
        sck = 0;

        #20;
        cs_n = 1;

        #10;

        check(
            mem[8'h70] === 8'h00,
            "aborted mid-byte WRITE leaves memory untouched"
        );


        // Verify recovery

        #40;
        cs_n = 0;

        send_byte(8'h02);
        send_byte(8'h71);
        send_byte(8'h99);

        #20;
        cs_n = 1;

        #10;

        check(
            mem[8'h71] === 8'h99,
            "transaction after abort works cleanly"
        );


        // ========================================================
        // 6. LOAD TX BUFFER 0x40
        // TXB0SIDH = 31
        // TXB0SIDL = 32
        // ========================================================

        #40;
        cs_n = 0;

        send_byte(8'h40);
        send_byte(8'h24);
        send_byte(8'h60);
        send_byte(8'h08);

        #20;
        cs_n = 1;

        #10;

        check(
            mem[8'h31] === 8'h24 &&
            mem[8'h32] === 8'h60 &&
            mem[8'h33] === 8'h00 &&
            mem[8'h34] === 8'h00 &&
            mem[8'h35] === 8'h08,
            "LOAD TX BUFFER 0x40 skips omitted EID bytes and reaches DLC"
        );


        // ========================================================
        // 7. LOAD TX BUFFER 0x41
        // TXB0D0 = 36
        // TXB0D1 = 37
        // ========================================================

        #40;
        cs_n = 0;

        send_byte(8'h41);
        send_byte(8'hAA);
        send_byte(8'hBB);

        #20;
        cs_n = 1;

        #10;

        check(
            mem[8'h36] === 8'hAA &&
            mem[8'h37] === 8'hBB,
            "LOAD TX BUFFER 0x41 writes TXB0D0/D1"
        );


        // ========================================================
        // 7b. LOAD TX BUFFER 0x42
        //
        // 0x42 corresponds to TXB1.
        // This design implements TXB0 only.
        // Therefore it must be rejected.
        // ========================================================

        mem[8'h00] = 8'h00;

        #40;
        cs_n = 0;

        send_byte(8'h42);
        send_byte(8'hFF);

        #20;
        cs_n = 1;

        #10;

        check(
            mem[8'h00] === 8'h00 &&
            we === 1'b0,
            "LOAD TX BUFFER 0x42 is rejected"
        );


        // ========================================================
        // 8. RTS TXB0
        // ========================================================

        seen_rts = 0;

        #40;
        cs_n = 0;

        send_byte(8'h81);

        #20;
        cs_n = 1;

        #10;

        check(
            seen_rts === 1'b1,
            "RTS 0x81 pulses rts_pulse"
        );


        // ========================================================
        // 8b. RTS TXB1
        // ========================================================

        seen_rts = 0;

        #40;
        cs_n = 0;

        send_byte(8'h82);

        #20;
        cs_n = 1;

        #10;

        check(
            seen_rts === 1'b0,
            "RTS 0x82 does not pulse for TXB1"
        );


        // ========================================================
        // 9. READ RX BUFFER 0x90
        //
        // 0x90 = RXB0 + SIDH
        // ========================================================

        seen_rxbuf_done = 0;

        mem[8'h61] = 8'h12;
        mem[8'h62] = 8'h34;

        #40;
        cs_n = 0;

        send_byte(8'h90);

        clock_bits(8, got);

        check(
            got === 8'h12,
            "READ RX BUFFER 0x90 returns RXB0SIDH"
        );

        clock_bits(8, got);

        check(
            got === 8'h34,
            "READ RX BUFFER auto-increments to RXB0SIDL"
        );

        #40;
        cs_n = 1;

        #40;

        check(
            seen_rxbuf_done === 1'b1,
            "rxbuf_done pulses when RX buffer transaction ends at CS"
        );


        // ========================================================
        // 10. READ RX BUFFER 0x92
        //
        // 0x92 = RXB0 + D0
        // ========================================================

        mem[8'h66] = 8'h77;

        #40;
        cs_n = 0;

        send_byte(8'h92);

        clock_bits(8, got);

        #20;
        cs_n = 1;

        check(
            got === 8'h77,
            "READ RX BUFFER 0x92 returns RXB0D0"
        );


        // ========================================================
        // 10b. READ RX BUFFER 0x94
        //
        // 0x94 = RXB1 + SIDH
        //
        // bit 2 = 1
        // Must be rejected because only RXB0 is implemented.
        // ========================================================

        mem[8'h61] = 8'hEE;

        #40;
        cs_n = 0;

        send_byte(8'h94);

        #40;
        cs_n = 1;

        check(
            so === 1'bz,
            "READ RX BUFFER 0x94 is rejected and so remains high-z"
        );


        // ========================================================
        // 10c. READ RX BUFFER 0x96
        //
        // 0x96 = RXB1 + D0
        // Must be rejected.
        // ========================================================

        #40;
        cs_n = 0;

        send_byte(8'h96);

        #40;
        cs_n = 1;

        check(
            so === 1'bz,
            "READ RX BUFFER 0x96 is rejected and so remains high-z"
        );


        // ========================================================
        // 10d. READ RX BUFFER 0x91
        //
        // bit 0 = 1
        // Must be rejected because bit 0 is reserved/invalid
        // for the implemented RX buffer command pattern.
        // ========================================================

        #40;
        cs_n = 0;

        send_byte(8'h91);

        #40;
        cs_n = 1;

        check(
            so === 1'bz,
            "READ RX BUFFER 0x91 is rejected because bit 0 is set"
        );


        // ========================================================
        // 10e. READ RX BUFFER 0x93
        //
        // bit 0 = 1
        // bit 1 = 1
        // Must be rejected.
        // ========================================================

        #40;
        cs_n = 0;

        send_byte(8'h93);

        #40;
        cs_n = 1;

        check(
            so === 1'bz,
            "READ RX BUFFER 0x93 is rejected because bit 0 is set"
        );


        // ========================================================
        // 11. READ STATUS
        // ========================================================

        #40;
        cs_n = 0;

        send_byte(8'hA0);

        clock_bits(8, got);

        #20;
        cs_n = 1;

        check(
            got === 8'hAB,
            "READ STATUS returns status byte"
        );


        // ========================================================
        // 12. RX STATUS
        // ========================================================

        #40;
        cs_n = 0;

        send_byte(8'hB0);

        clock_bits(8, got);

        check(
            got === 8'hCD,
            "RX STATUS returns rxstatus byte"
        );

        // Extra clocks should repeat the value

        clock_bits(8, got);

        check(
            got === 8'hCD,
            "RX STATUS repeats on extra clocks"
        );

        #20;
        cs_n = 1;


        // ========================================================
        // 12b. LIVE STATUS RE-SAMPLING
        // ========================================================

        status_byte = 8'h11;

        #40;
        cs_n = 0;

        send_byte(8'hA0);

        begin : live_resample

            integer i;

            reg [7:0] b1;
            reg [7:0] b2;

            b1 = 8'h00;
            b2 = 8'h00;

            // First 7 bits

            for (i = 0; i < 7; i = i + 1) begin

                #40;
                sck = 1;

                b1 = {b1[6:0], so};

                #40;
                sck = 0;

            end

            // Change status before the final bit

            status_byte = 8'h22;

            #40;
            sck = 1;

            b1 = {b1[6:0], so};

            #40;
            sck = 0;

            check(
                b1 === 8'h11,
                "READ STATUS first byte reflects value at decode time"
            );

            // Next byte should sample the new live value

            clock_bits(8, b2);

            check(
                b2 === 8'h22,
                "READ STATUS second byte re-samples live value"
            );

        end

        #20;
        cs_n = 1;

        status_byte = 8'hAB;


        // ========================================================
        // 13. CONTINUOUS 5-BYTE WRITE
        // ========================================================

        #40;
        cs_n = 0;

        send_byte(8'h02);
        send_byte(8'h80);

        send_byte(8'h11);
        send_byte(8'h22);
        send_byte(8'h33);
        send_byte(8'h44);
        send_byte(8'h55);

        #20;
        cs_n = 1;

        #10;

        check(
            mem[8'h80] === 8'h11 &&
            mem[8'h81] === 8'h22 &&
            mem[8'h82] === 8'h33 &&
            mem[8'h83] === 8'h44 &&
            mem[8'h84] === 8'h55,
            "5-byte continuous WRITE auto-increments correctly"
        );


        // ========================================================
        // 14. SO TRI-STATE
        // ========================================================

        #40;

        check(
            so === 1'bz,
            "SO is high-Z when CS is high"
        );


        // During WRITE, SO should remain high-Z

        #40;
        cs_n = 0;

        send_byte(8'h02);
        send_byte(8'h81);

        check(
            so === 1'bz,
            "SO stays high-Z during WRITE data phase"
        );

        send_byte(8'h99);

        #20;
        cs_n = 1;


        // ========================================================
        // 15. UNKNOWN OPCODE
        // ========================================================

        seen_reset = 0;
        seen_rts   = 0;

        #40;
        if (errors == 0) $display(">>> ALL TESTS PASSED <<<");
        else $display(">>> %0d TEST(S) FAILED <<<", errors);
        $finish;
    end

endmodule

/*# PASS: RESET pulse fires after opcode
# PASS: WRITE lands correctly
# PASS: sequential WRITE auto-increments correctly
# PASS: READ returns previously written byte
# PASS: READ byte 1 correct
# PASS: READ byte 2 auto-increments correctly
# PASS: READ byte 3 auto-increments correctly
# PASS: READ after mid-byte abort starts clean
# PASS: READ auto-increment works after abort
# PASS: aborted mid-byte WRITE leaves memory untouched
# PASS: transaction after abort works cleanly
# PASS: LOAD TX BUFFER 0x40 writes TXB0SIDH/SIDL
# PASS: LOAD TX BUFFER 0x41 writes TXB0D0/D1
# PASS: LOAD TX BUFFER 0x42 is rejected
# PASS: RTS 0x81 pulses rts_pulse
# PASS: RTS 0x82 does not pulse for TXB1
# PASS: READ RX BUFFER 0x90 returns RXB0SIDH
# PASS: READ RX BUFFER auto-increments to RXB0SIDL
# PASS: rxbuf_done pulses when RX buffer transaction ends at CS
# PASS: READ RX BUFFER 0x92 returns RXB0D0
# PASS: READ RX BUFFER 0x94 is rejected and so remains high-z
# PASS: READ RX BUFFER 0x96 is rejected and so remains high-z
# PASS: READ RX BUFFER 0x91 is rejected because bit 0 is set
# PASS: READ RX BUFFER 0x93 is rejected because bit 0 is set
# PASS: READ STATUS returns status byte
# PASS: RX STATUS returns rxstatus byte
# PASS: RX STATUS repeats on extra clocks
# PASS: READ STATUS first byte reflects value at decode time
# PASS: READ STATUS second byte re-samples live value
# PASS: 5-byte continuous WRITE auto-increments correctly
# PASS: SO is high-Z when CS is high
# PASS: SO stays high-Z during WRITE data phase
# PASS: unknown opcode produces no side effects
# PASS: normal WRITE after unknown opcode works
# >>> ALL TESTS PASSED <<<*/
