`timescale 1ns/1ps

module tb_spi_if;
    reg clk = 0, rst_n = 0, cs_n = 1, sck = 0, si = 0;
    wire so;
    wire [7:0] addr, wdata;
    reg  [7:0] rdata;
    wire we, reset_pulse, rxbuf_done, bitmod_we, rxbuf_sel;
    wire [2:0] rts_pulse;
    wire [7:0] bitmod_mask;

    reg [7:0] mem [0:255];
    integer i0;
    integer errors = 0;
 reg [7:0] status_byte = 8'hAB;
    reg [7:0] rxstatus_byte = 8'hCD;
    spi_if dut (
        .clk(clk), .rst_n(rst_n), .sck(sck), .si(si), .so(so), .cs_n(cs_n),
        .addr(addr), .wdata(wdata), .rdata(rdata), .we(we),
        .reset_pulse(reset_pulse), .rts_pulse(rts_pulse), .rxbuf_done(rxbuf_done), .rxbuf_sel(rxbuf_sel),
        .bitmod_mask(bitmod_mask), .bitmod_we(bitmod_we),
        .status_byte(status_byte), .rxstatus_byte(rxstatus_byte)
    );

    always #5 clk = ~clk;
    always @(*) rdata = mem[addr];
    always @(posedge clk) if (we) mem[addr] <= wdata;
    always @(posedge clk) if (bitmod_we) mem[addr] <= (mem[addr] & ~bitmod_mask) | (wdata & bitmod_mask);

    task send_byte(input [7:0] data);
        integer i;
        begin
            for (i = 7; i >= 0; i = i - 1) begin
                si = data[i];
                #40 sck = 1;
                #40 sck = 0;
            end
        end
    endtask

    task clock_bits(input integer n, output [7:0] got);
        integer i;
        begin
            got = 8'h00;
            for (i = 0; i < n; i = i + 1) begin
                #40 sck = 1; got = {got[6:0], so};
                #40 sck = 0;
            end
        end
    endtask

    task check(input cond, input [63*8:0] msg);
        begin
            if (!cond) begin
                $display("FAIL: %0s", msg);
                errors = errors + 1;
            end else
                $display("PASS: %0s", msg);
        end
    endtask

   
    reg [7:0] got;
    reg seen_reset, seen_rxbuf_done, seen_bitmod_we;
    reg [2:0] seen_rts_vec;
    reg seen_rxbuf_sel;

    initial begin
        seen_reset      = 0;
        seen_rxbuf_done = 0;
        seen_bitmod_we  = 0;
        seen_rts_vec    = 3'b000;
        seen_rxbuf_sel  = 0;
    end

    always @(posedge clk) if (reset_pulse)      seen_reset      <= 1'b1;
    always @(posedge clk) if (rts_pulse != 0)   seen_rts_vec    <= seen_rts_vec | rts_pulse;
    always @(posedge clk) if (rxbuf_done) begin seen_rxbuf_done <= 1'b1; seen_rxbuf_sel <= rxbuf_sel; end
    always @(posedge clk) if (bitmod_we)        seen_bitmod_we  <= 1'b1;

    initial begin
        rst_n = 0;
        for (i0 = 0; i0 < 256; i0 = i0 + 1) mem[i0] = 8'h00;
        #50 rst_n = 1;

        // ---- 1: RESET ----
        seen_reset = 0;
        #20 cs_n = 0; send_byte(8'hC0); #20 cs_n = 1;
        #10 check(seen_reset === 1'b1, "RESET pulse fires right after opcode");

        // ---- 2: WRITE 0xAA to 0x50 ----
        #40 cs_n = 0; send_byte(8'h02); send_byte(8'h50); send_byte(8'hAA); #20 cs_n = 1;
        #10 check(mem[8'h50] === 8'hAA, "WRITE lands correctly");

        // ---- 3: sequential WRITE with auto-increment ----
        #40 cs_n = 0; send_byte(8'h02); send_byte(8'h60);
        send_byte(8'h11); send_byte(8'h22); send_byte(8'h33); #20 cs_n = 1;
        #10 check(mem[8'h60]===8'h11 && mem[8'h61]===8'h22 && mem[8'h62]===8'h33,
                   "sequential WRITE auto-increments correctly");

        // ---- 4: READ back 0x50, expect 0xAA ----
        #40 cs_n = 0; send_byte(8'h03); send_byte(8'h50);
        clock_bits(8, got); #20 cs_n = 1;
        check(got === 8'hAA, "READ returns previously written byte");

        // ---- 4b: plain READ, 3 consecutive bytes, verifying auto-increment
        //          on normal READ (not just WRITE or READ RX BUFFER) ----
        mem[8'h20] = 8'h11; mem[8'h21] = 8'h22; mem[8'h22] = 8'h33;
        #40 cs_n = 0; send_byte(8'h03); send_byte(8'h20);
        clock_bits(8, got); check(got === 8'h11, "READ byte 1 of 3 correct");
        clock_bits(8, got); check(got === 8'h22, "READ byte 2 of 3 auto-increments correctly");
        clock_bits(8, got); check(got === 8'h33, "READ byte 3 of 3 auto-increments correctly");
        #20 cs_n = 1;

        // ---- 4c: CS abort mid-READ, then verify a fresh READ starts clean ----
        mem[8'h23] = 8'h55; mem[8'h24] = 8'h66;
        #40 cs_n = 0; send_byte(8'h03); send_byte(8'h23);
        clock_bits(4, got);              // only half a byte clocked out
        #20 cs_n = 1;                    // abort mid-byte
        #40 cs_n = 0; send_byte(8'h03); send_byte(8'h23);
        clock_bits(8, got); check(got === 8'h55, "READ right after a mid-byte abort starts clean at the requested address");
        clock_bits(8, got); check(got === 8'h66, "READ auto-increment still works after a prior abort");
        #20 cs_n = 1;

        // ---- 5: mid-byte CS abort during WRITE doesn't corrupt, and next
        //         transaction starts clean ----
        #40 cs_n = 0; send_byte(8'h02); send_byte(8'h70);
        si = 1; #40 sck = 1; #40 sck = 0;    // only 1 of 8 data bits sent
        #20 cs_n = 1;
        #10 check(mem[8'h70] === 8'h00, "aborted mid-byte WRITE leaves memory untouched");
        #40 cs_n = 0; send_byte(8'h02); send_byte(8'h71); send_byte(8'h99); #20 cs_n = 1;
        #10 check(mem[8'h71] === 8'h99, "transaction right after an abort works cleanly");

        // ---- 6: LOAD TX BUFFER, start-at-ID (0100_0000) ----
        #40 cs_n = 0; send_byte(8'h40); send_byte(8'h24); send_byte(8'h60); #20 cs_n = 1;
        #10 check(mem[8'h31]===8'h24 && mem[8'h32]===8'h60, "LOAD TX BUFFER (SIDH entry) writes TXB0SIDH/SIDL");

        // ---- 7: LOAD TX BUFFER, start-at-data (0100_0001) ----
        #40 cs_n = 0; send_byte(8'h41); send_byte(8'hAA); send_byte(8'hBB); #20 cs_n = 1;
        #10 check(mem[8'h36]===8'hAA && mem[8'h37]===8'hBB, "LOAD TX BUFFER (D0 entry) writes TXB0D0/D1");

        // ---- 7b: LOAD TX BUFFER targeting TXB1 (0100_0010 -> ab=01, c=0) ----
        #40 cs_n = 0; send_byte(8'h42); send_byte(8'h11); send_byte(8'h22); #20 cs_n = 1;
        #10 check(mem[8'h41]===8'h11 && mem[8'h42]===8'h22, "LOAD TX BUFFER (TXB1, SIDH entry) writes TXB1SIDH/SIDL");

        // ---- 7c: LOAD TX BUFFER targeting TXB2 data entry (0100_0101 -> ab=10, c=1) ----
        #40 cs_n = 0; send_byte(8'h45); send_byte(8'h33); #20 cs_n = 1;
        #10 check(mem[8'h56]===8'h33, "LOAD TX BUFFER (TXB2, D0 entry) writes TXB2D0");

        // ---- 7d: LOAD TX BUFFER with reserved ab=11 code -> must be a no-op ----
        mem[8'h00] = 8'h00;
        #40 cs_n = 0; send_byte(8'b0100_0110); send_byte(8'hFF); #20 cs_n = 1;
        #10 check(mem[8'h00] === 8'h00, "LOAD TX BUFFER reserved code (ab=11) is a no-op");

        // ---- 8: RTS targeting TXB0 only (nnn=001) ----
        seen_rts_vec = 3'b000;
        #40 cs_n = 0; send_byte(8'h81); #20 cs_n = 1;
        #10 check(seen_rts_vec === 3'b001, "RTS pulses only TXB0 for nnn=001");

        // ---- 8b: RTS targeting TXB1 only (nnn=010) ----
        seen_rts_vec = 3'b000;
        #40 cs_n = 0; send_byte(8'h82); #20 cs_n = 1;
        #10 check(seen_rts_vec === 3'b010, "RTS pulses only TXB1 for nnn=010");

        // ---- 8c: RTS targeting TXB0 AND TXB2 simultaneously (nnn=101) --
        //          Section 12.7: "Any or all of the last three bits can be
        //          set in a single command" ----
        seen_rts_vec = 3'b000;
        #40 cs_n = 0; send_byte(8'h85); #20 cs_n = 1;
        #10 check(seen_rts_vec === 3'b101, "RTS pulses TXB0 and TXB2 together for nnn=101");

        // ---- 9: RTS with nnn=000 -> must NOT pulse ----
        seen_rts_vec = 3'b000;
        #40 cs_n = 0; send_byte(8'h80); #20 cs_n = 1;
        #10 check(seen_rts_vec === 3'b000, "RTS does not pulse when nnn=000");

        // ---- 10: READ RX BUFFER on RXB0, start-at-ID; check RXnIF-clear
        //          pulse + rxbuf_sel on CS=1 ----
        seen_rxbuf_done = 0;
        mem[8'h61] = 8'h12; mem[8'h62] = 8'h34;
        #40 cs_n = 0; send_byte(8'h90);
        clock_bits(8, got); check(got === 8'h12, "READ RX BUFFER (RXB0, SIDH entry) returns RXB0SIDH");
        clock_bits(8, got); check(got === 8'h34, "READ RX BUFFER auto-increments to RXB0SIDL");
        #20 cs_n = 1;
        #40 check(seen_rxbuf_done === 1'b1 && seen_rxbuf_sel === 1'b0,
                   "RXnIF-clear pulses with rxbuf_sel=0 (RXB0) after READ RX BUFFER on RXB0");

        // ---- 11: READ RX BUFFER, RXB0 start-at-data ----
        mem[8'h66] = 8'h77;
        #40 cs_n = 0; send_byte(8'h92); clock_bits(8, got); #20 cs_n = 1;
        check(got === 8'h77, "READ RX BUFFER (RXB0, D0 entry) returns RXB0D0");

        // ---- 11b: READ RX BUFFER on RXB1 (n=1); check rxbuf_sel=1 ----
        seen_rxbuf_done = 0;
        mem[8'h71] = 8'hCC;
        #40 cs_n = 0; send_byte(8'h94); clock_bits(8, got); #20 cs_n = 1;
        check(got === 8'hCC, "READ RX BUFFER (RXB1, SIDH entry) returns RXB1SIDH");
        #40 check(seen_rxbuf_done === 1'b1 && seen_rxbuf_sel === 1'b1,
                   "RXnIF-clear pulses with rxbuf_sel=1 (RXB1) after READ RX BUFFER on RXB1");

        // ---- 12: READ STATUS ----
        #40 cs_n = 0; send_byte(8'hA0); clock_bits(8, got); #20 cs_n = 1;
        check(got === 8'hAB, "READ STATUS returns assembled status byte");

        // ---- 13: RX STATUS, and re-reads same byte on extra clocks (3
        //          repeats, to be sure it isn't a one-off fluke) ----
        #40 cs_n = 0; send_byte(8'hB0);
        clock_bits(8, got); check(got === 8'hCD, "RX STATUS returns assembled rxstatus byte");
        clock_bits(8, got); check(got === 8'hCD, "RX STATUS repeats same byte on 2nd extra clock while CS low");
        clock_bits(8, got); check(got === 8'hCD, "RX STATUS repeats same byte on 3rd extra clock while CS low");
        #20 cs_n = 1;

        // ---- 15: BIT MODIFY on a legal register (TXB0CTRL=0x30): set only
        //          bit3 (TXREQ), leave the rest of the byte untouched ----
        mem[8'h30] = 8'b0101_0101;   // pretend pre-existing bits
        seen_bitmod_we = 0;
        #40 cs_n = 0; send_byte(8'h05); send_byte(8'h30);
        send_byte(8'b0000_1000);      // mask: only bit3
        send_byte(8'b1111_1111);      // data: try to set every bit...
        #20 cs_n = 1;
        #10 check(seen_bitmod_we === 1'b1, "BIT MODIFY pulses on a legal address");
        check(mem[8'h30] === 8'b0101_1101,
              "BIT MODIFY only changes masked bit(s), leaves the rest alone");

        // ---- 16: exact reviewer example: BIT MODIFY on CANINTF (0x2C),
        //          mask=0x0F, data=0x05, old=0xFF -> new=0xF5 ----
        mem[8'h2C] = 8'hFF;
        seen_bitmod_we = 0;
        #40 cs_n = 0; send_byte(8'h05); send_byte(8'h2C);
        send_byte(8'h0F); send_byte(8'h05);
        #20 cs_n = 1;
        #10 check(seen_bitmod_we === 1'b1, "BIT MODIFY (CANINTF example) pulses");
        check(mem[8'h2C] === 8'hF5, "BIT MODIFY (CANINTF example) computes old/mask/new correctly");

        // ---- 16b: BIT MODIFY on TXB1CTRL (now legal since TXB1 exists) ----
        mem[8'h40] = 8'b0000_0000;
        seen_bitmod_we = 0;
        #40 cs_n = 0; send_byte(8'h05); send_byte(8'h40);
        send_byte(8'b0000_1000); send_byte(8'b1111_1111);   // set only TXREQ (bit3)
        #20 cs_n = 1;
        #10 check(seen_bitmod_we === 1'b1, "BIT MODIFY on TXB1CTRL pulses (now a legal target)");
        check(mem[8'h40] === 8'b0000_1000, "BIT MODIFY on TXB1CTRL only sets the masked bit");

        // ---- 16c: BIT MODIFY on CANCTRL (0x0F) -- now a legal target ----
        mem[8'h0F] = 8'b1000_0111;
        seen_bitmod_we = 0;
        #40 cs_n = 0; send_byte(8'h05); send_byte(8'h0F);
        send_byte(8'b0001_0000); send_byte(8'b0000_0000);   // clear only ABAT (bit4)
        #20 cs_n = 1;
        #10 check(seen_bitmod_we === 1'b1, "BIT MODIFY on CANCTRL pulses (now a legal target)");
        check(mem[8'h0F] === (8'b1000_0111 & ~8'b0001_0000), "BIT MODIFY on CANCTRL only clears the masked bit");

        // ---- 17: BIT MODIFY on an address OUTSIDE the honored-mask set
        //          (e.g. an RXF filter register) -- per Section 12.10 this
        //          must NOT be rejected; mask is forced to FFh, so it still
        //          succeeds as a full overwrite ----
        mem[8'h00] = 8'hFF;
        seen_bitmod_we = 0;
        #40 cs_n = 0; send_byte(8'h05); send_byte(8'h00);
        send_byte(8'h0F);       // mask sent by host -- irrelevant, forced to FF
        send_byte(8'hAB);       // data
        #20 cs_n = 1;
        #10 check(seen_bitmod_we === 1'b1, "BIT MODIFY on an unlisted address still pulses (mask forced to FFh)");
        check(mem[8'h00] === 8'hAB,
              "BIT MODIFY on an unlisted address overwrites fully, matching Section 12.10");

        // ---- 17b: end-to-end datapath check -- WRITE, then BIT MODIFY,
        //           then READ it back over SPI (not a direct mem[] peek),
        //           proving the full addr/wdata/rdata/bitmod path together ----
        // #40 cs_n = 0; send_byte(8'h02); send_byte(8'h90); send_byte(8'b0000_1111); #20 cs_n = 1;
        // #40 cs_n = 0; send_byte(8'h05); send_byte(8'h90);
        // send_byte(8'b1111_0000); send_byte(8'b1010_0000);   // set masked bits per mask/data
        // #20 cs_n = 1;
        // #40 cs_n = 0; send_byte(8'h03); send_byte(8'h90);
        // clock_bits(8, got);
        // #20 cs_n = 1;
        // check(got === 8'b1010_1111,
        //       "READ after WRITE+BIT MODIFY reflects the correct final value via SPI, not just mem[]");


        // ---- 17b: end-to-end datapath check -- WRITE, then BIT MODIFY (on a
//           LEGAL register this time, so the real mask is honored, not
//           forced to FFh), then READ it back over SPI -- proving the
//           full addr/wdata/rdata/bitmod path together ----
#40 cs_n = 0; send_byte(8'h02); send_byte(8'h60); send_byte(8'b0000_1111); #20 cs_n = 1;
#40 cs_n = 0; send_byte(8'h05); send_byte(8'h60);
send_byte(8'b1111_0000); send_byte(8'b1010_0000);   // set masked bits per mask/data
#20 cs_n = 1;
#40 cs_n = 0; send_byte(8'h03); send_byte(8'h60);
clock_bits(8, got);
#20 cs_n = 1;
check(got === 8'b1010_1111,
      "READ after WRITE+BIT MODIFY (legal register) reflects the correct final value via SPI, not just mem[]");





        // ---- 18: continuous 5-byte WRITE, verifying we_d/addr pipeline
        //          holds up across a longer burst, not just 3 bytes ----
        #40 cs_n = 0; send_byte(8'h02); send_byte(8'h80);
        send_byte(8'h11); send_byte(8'h22); send_byte(8'h33);
        send_byte(8'h44); send_byte(8'h55);
        #20 cs_n = 1;
        #10 check(mem[8'h80]===8'h11 && mem[8'h81]===8'h22 && mem[8'h82]===8'h33 &&
                   mem[8'h83]===8'h44 && mem[8'h84]===8'h55,
                   "5-byte continuous WRITE lands at all 5 sequential addresses correctly");

        // ---- 19: SO is high-Z while deselected ----
        #40 check(so === 1'bz, "SO is high-impedance when CS is high");

        // ---- 19b: SO is ALSO high-Z during an active WRITE (CS low, but
        //           not in a data-output state) -- matches Fig 12-2's
        //           "Data Out High-Impedance" during instruction+address ----
        #40 cs_n = 0; send_byte(8'h02); send_byte(8'h81);
        check(so === 1'bz, "SO stays high-Z during WRITE's data phase (never outputting)");
        send_byte(8'h99);
        #20 cs_n = 1;

        // ---- 19c: live status re-sampling -- change status_byte partway
        //           through byte 1's shift-out (before its last bit, which
        //           is when the reload for byte 2 actually happens), then
        //           confirm byte 2 reflects the new value ----
        status_byte = 8'h11;
        #40 cs_n = 0; send_byte(8'hA0);
        begin : live_resample
            integer i;
            reg [7:0] b1, b2;
            b1 = 8'h00;
            b2 = 8'h00;

            for (i = 0; i < 7; i = i + 1) begin
                #40 sck = 1; b1 = {b1[6:0], so};
                #40 sck = 0;
            end

            status_byte = 8'h22;              // change it before byte 1's last bit
            #40 sck = 1; b1 = {b1[6:0], so};   // byte 1's 8th bit -- also triggers the reload for byte 2
            #40 sck = 0;
            check(b1 === 8'h11, "READ STATUS byte 1 still reflects the value at decode time");
            clock_bits(8, b2);
            check(b2 === 8'h22, "READ STATUS byte 2 re-samples the LIVE value, not the frozen one");
        end
        #20 cs_n = 1;
        status_byte = 8'hAB;   // restore

        // ---- 20: unrecognized opcode is ignored gracefully, no side effects ----
        seen_reset = 0; seen_rts_vec = 3'b000; seen_bitmod_we = 0;
        #40 cs_n = 0; send_byte(8'hFF); #20 cs_n = 1;
        #40 check(dut.state === 0, "unknown opcode returns cleanly to Idle");
        check(we === 1'b0 && seen_reset === 1'b0 && seen_rts_vec === 3'b000 && seen_bitmod_we === 1'b0,
              "unknown opcode produces no we/reset_pulse/rts_pulse/bitmod_we side effects");

        #40;
        if (errors == 0) $display(">>> ALL TESTS PASSED <<<");
        else $display(">>> %0d TEST(S) FAILED <<<", errors);
        $finish;
    end
endmodule

// questasim results:- 
/*
# PASS: RESET pulse fires right after opcode
# PASS: WRITE lands correctly
# PASS: sequential WRITE auto-increments correctly
# PASS: READ returns previously written byte
# PASS: READ byte 1 of 3 correct
# PASS: READ byte 2 of 3 auto-increments correctly
# PASS: READ byte 3 of 3 auto-increments correctly
# PASS: READ right after a mid-byte abort starts clean at the requested address
# PASS: READ auto-increment still works after a prior abort
# PASS: aborted mid-byte WRITE leaves memory untouched
# PASS: transaction right after an abort works cleanly
# PASS: LOAD TX BUFFER (SIDH entry) writes TXB0SIDH/SIDL
# PASS: LOAD TX BUFFER (D0 entry) writes TXB0D0/D1
# PASS: LOAD TX BUFFER (TXB1, SIDH entry) writes TXB1SIDH/SIDL
# PASS: LOAD TX BUFFER (TXB2, D0 entry) writes TXB2D0
# PASS: LOAD TX BUFFER reserved code (ab=11) is a no-op
# PASS: RTS pulses only TXB0 for nnn=001
# PASS: RTS pulses only TXB1 for nnn=010
# PASS: RTS pulses TXB0 and TXB2 together for nnn=101
# PASS: RTS does not pulse when nnn=000
# PASS: READ RX BUFFER (RXB0, SIDH entry) returns RXB0SIDH
# PASS: READ RX BUFFER auto-increments to RXB0SIDL
# PASS: ear pulses with rxbuf_sel=0 (RXB0) after READ RX BUFFER on RXB0
# PASS: READ RX BUFFER (RXB0, D0 entry) returns RXB0D0
# PASS: READ RX BUFFER (RXB1, SIDH entry) returns RXB1SIDH
# PASS: ear pulses with rxbuf_sel=1 (RXB1) after READ RX BUFFER on RXB1
# PASS: READ STATUS returns assembled status byte
# PASS: RX STATUS returns assembled rxstatus byte
# PASS: RX STATUS repeats same byte on 2nd extra clock while CS low
# PASS: RX STATUS repeats same byte on 3rd extra clock while CS low
# PASS: BIT MODIFY pulses on a legal address
# PASS: BIT MODIFY only changes masked bit(s), leaves the rest alone
# PASS: BIT MODIFY (CANINTF example) pulses
# PASS: BIT MODIFY (CANINTF example) computes old/mask/new correctly
# PASS: BIT MODIFY on TXB1CTRL pulses (now a legal target)
# PASS: BIT MODIFY on TXB1CTRL only sets the masked bit
# PASS: BIT MODIFY on CANCTRL pulses (now a legal target)
# PASS: BIT MODIFY on CANCTRL only clears the masked bit
# PASS: MODIFY on an unlisted address still pulses (mask forced to FFh)
# PASS: on an unlisted address overwrites fully, matching Section 12.10
# PASS: READ after WRITE+BIT MODIFY (legal register) reflects the correct final value via SPI, not just mem[]
# PASS: 5-byte continuous WRITE lands at all 5 sequential addresses correctly
# PASS: SO is high-impedance when CS is high
# PASS: SO stays high-Z during WRITE's data phase (never outputting)
# PASS: READ STATUS byte 1 still reflects the value at decode time
# PASS: EAD STATUS byte 2 re-samples the LIVE value, not the frozen one
# PASS: unknown opcode returns cleanly to Idle
# PASS: unknown opcode produces no we/reset_pulse/rts_pulse/bitmod_we side effects
# >>> ALL TESTS PASSED <<<

*/
