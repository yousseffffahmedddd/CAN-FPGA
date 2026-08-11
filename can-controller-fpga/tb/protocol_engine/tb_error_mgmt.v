`timescale 1ns/1ps

module tb_error_mgmt;

    reg clk;
    reg rst_n;
    reg crc_err_pulse;
    reg stuff_err_pulse;
    reg form_err_pulse;
    reg bit_err_pulse;
    reg ack_err_pulse;
    reg err_flag_bit_err_pulse;
    reg node_is_tx;
    reg tx_success_pulse;
    reg rx_success_pulse;
    reg recessive11_pulse;

    wire [7:0] tec;
    wire [7:0] rec;
    wire err_active;
    wire err_passive;
    wire bus_off;
    wire ewarn;    reg tx_err_state;
    integer errors;
    integer checks;
    integer i;

    task check;
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

    task pulse_tx_error;
        begin
            node_is_tx = 1'b1;
            crc_err_pulse = 1'b0;
            @(posedge clk);
            crc_err_pulse = 1'b1;
            @(posedge clk);
            #1;
            crc_err_pulse = 1'b0;
            node_is_tx = 1'b0;
            @(posedge clk);
            #1;
        end
    endtask

    error_mgmt dut (
        .clk                  (clk),
        .rst_n                (rst_n),
        .crc_err_pulse        (crc_err_pulse),
        .stuff_err_pulse      (stuff_err_pulse),
        .form_err_pulse       (form_err_pulse),
        .bit_err_pulse        (bit_err_pulse),
        .ack_err_pulse        (ack_err_pulse),
        .err_flag_bit_err_pulse(err_flag_bit_err_pulse),
        .node_is_tx           (node_is_tx),
        .tx_success_pulse     (tx_success_pulse),
        .rx_success_pulse     (rx_success_pulse),
        .recessive11_pulse    (recessive11_pulse),
        .tec                  (tec),
        .rec                  (rec),
        .err_active           (err_active),
        .err_passive          (err_passive),
        .bus_off              (bus_off),
        .ewarn                (ewarn)
    );

    always #5 clk = ~clk;

    initial begin
        clk = 0;
        rst_n = 0;
        crc_err_pulse = 0;
        stuff_err_pulse = 0;
        form_err_pulse = 0;
        bit_err_pulse = 0;
        ack_err_pulse = 0;
        err_flag_bit_err_pulse = 0;
        node_is_tx = 0;
        tx_err_state = 0;
        tx_success_pulse = 0;
        rx_success_pulse = 0;
        recessive11_pulse = 0;
        checks = 0;
        errors = 0;

        repeat (3) @(posedge clk);
        rst_n = 1;
        @(posedge clk);

        check(tec == 8'd0 && rec == 8'd0, "reset clears counters");
        check(err_active == 1'b1 && err_passive == 1'b0 && bus_off == 1'b0,
              "reset leaves node in error-active");

        pulse_tx_error();
        check(tec == 8'd8, "CRC error while TX increments TEC by 8");

        i = 0;
        while (i < 15) begin
            pulse_tx_error();
            i = i + 1;
        end
        check(err_passive == 1'b1 && tec == 8'd128,
              "16 TX errors reach error-passive threshold");

        i = 0;
        while (i < 16) begin
            pulse_tx_error();
            i = i + 1;
        end
        check(bus_off == 1'b1, "TEC overflows into bus-off");

        i = 0;
        while (i < 128) begin
            recessive11_pulse = 1'b1;
            @(posedge clk);
            #1;
            recessive11_pulse = 1'b0;
            @(posedge clk);
            i = i + 1;
        end

        check(tec == 8'd0 && rec == 8'd0 && bus_off == 1'b0,
              "128 recessive11 sequences recover from bus-off");

        $display("--------------------------------------------------");
        $display("tb_error_mgmt: %0d checks, %0d failures", checks, errors);
        if (errors == 0)
            $display("RESULT: PASS");
        else
            $display("RESULT: FAIL");
        $display("--------------------------------------------------");
        $finish;
    end

endmodule
