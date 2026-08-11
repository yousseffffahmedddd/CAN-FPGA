// =============================================================================
// Testbench      : tb_crc_gen_check
// DUT            : crc_gen_check.v
// Purpose        : Verify CRC-15 LFSR behavior against the public reference
//                  vector "123456789" and basic RX mismatch reporting.
// =============================================================================
`timescale 1ns/1ps

module tb_crc_gen_check;

    reg clk;
    reg rst_n;
    reg crc_init;
    reg bit_strobe;
    reg logical_bit;
    reg crc_field_rx;
    reg crc_latch;

    wire [14:0] crc_out;
    wire crc_error;

    integer checks;
    integer errors;
    integer i;
    integer byte_idx;
    integer bit_idx;
    reg [7:0] byte_val;

    crc_gen_check dut (
        .clk         (clk),
        .rst_n       (rst_n),
        .crc_init    (crc_init),
        .bit_strobe  (bit_strobe),
        .logical_bit (logical_bit),
        .crc_field_rx(crc_field_rx),
        .crc_latch   (crc_latch),
        .crc_out     (crc_out),
        .crc_error   (crc_error)
    );

    always #5 clk = ~clk;

    task automatic pulse_reg;
        input reg sig;
        output reg out_sig;
        begin
            out_sig = 1'b1;
            @(posedge clk);
            #1;
            out_sig = 1'b0;
            @(posedge clk);
        end
    endtask

    task automatic drive_bit;
        input bit_val;
        begin
            logical_bit = bit_val;
            bit_strobe = 1'b1;
            @(posedge clk);
            #1;
            bit_strobe = 1'b0;
            @(posedge clk);
        end
    endtask

    task automatic check;
        input cond;
        input [255:0] msg;
        begin
            checks = checks + 1;
            if (!cond) begin
                errors = errors + 1;
                $display("[FAIL] %0t : %s", $time, msg);
            end
            else begin
                $display("[PASS] %0t : %s", $time, msg);
            end
        end
    endtask

    initial begin
        clk = 0;
        rst_n = 0;
        crc_init = 0;
        bit_strobe = 0;
        logical_bit = 0;
        crc_field_rx = 0;
        crc_latch = 0;
        checks = 0;
        errors = 0;

        repeat (3) @(posedge clk);
        rst_n = 1;
        @(posedge clk);

        // Test 1: known reference vector "123456789" -> 15'h059E
        crc_init = 1'b1;
        @(posedge clk);
        #1;
        crc_init = 1'b0;
        @(posedge clk);

        byte_idx = 0;
        while (byte_idx < 9) begin
            case (byte_idx)
                0: byte_val = 8'h31;
                1: byte_val = 8'h32;
                2: byte_val = 8'h33;
                3: byte_val = 8'h34;
                4: byte_val = 8'h35;
                5: byte_val = 8'h36;
                6: byte_val = 8'h37;
                7: byte_val = 8'h38;
                8: byte_val = 8'h39;
                default: byte_val = 8'h00;
            endcase

            bit_idx = 0;
            while (bit_idx < 8) begin
                drive_bit(byte_val[7-bit_idx]);
                bit_idx = bit_idx + 1;
            end
            byte_idx = byte_idx + 1;
        end

        crc_latch = 1'b1;
        @(posedge clk);
        #1;
        crc_latch = 1'b0;
        @(posedge clk);

        check(crc_out == 15'h059E, "CRC-15 reference vector yields 15'h059E");

        // Test 2: RX CRC field mismatch asserts crc_error for one cycle
        crc_init = 1'b1;
        @(posedge clk);
        #1;
        crc_init = 1'b0;
        @(posedge clk);

        i = 0;
        while (i < 8) begin
            drive_bit(1'b0);
            i = i + 1;
        end

        crc_latch = 1'b1;
        @(posedge clk);
        #1;
        crc_latch = 1'b0;
        @(posedge clk);

        crc_field_rx = 1'b1;
        i = 0;
        while (i < 15) begin
            if (i == 0)
                logical_bit = 1'b0;
            else if (i == 14)
                logical_bit = 1'b1;
            else
                logical_bit = 1'b0;
            bit_strobe = 1'b1;
            @(posedge clk);
            #1;
            bit_strobe = 1'b0;
            @(posedge clk);
            i = i + 1;
        end
        crc_field_rx = 1'b0;

        check(crc_error == 1'b1, "CRC mismatch generates crc_error pulse");

        $display("--------------------------------------------------");
        $display("tb_crc_gen_check: %0d checks, %0d failures", checks, errors);
        if (errors == 0)
            $display("RESULT: PASS");
        else
            $display("RESULT: FAIL");
        $display("--------------------------------------------------");
        $finish;
    end

endmodule
