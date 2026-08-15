`timescale 1ns/1ps

module tb_ctrl_int_regs;

    reg         clk;
    reg         rst_n;
    reg  [7:0]  addr;
    reg  [7:0]  wdata;
    reg         we;

    reg         control_reset;
    reg         txreq_set;
    reg         txreq_clear;
    reg         set_rx0if;
    reg         clear_rx0if;
    reg         set_tx0if;
    reg         clear_tx0if;
    reg         set_merrf;
    reg         clear_merrf;
    reg         set_errif;

    reg  [7:0]  tec_in;
    reg  [7:0]  rec_in;

    wire [7:0]  rdata;
    wire [7:0]  canctrl;
    wire [7:0]  bfpctrl;
    wire [7:0]  caninte;
    wire [7:0]  canintf;
    wire [2:0]  opmod;
    wire        config_mode;
    wire        normal_mode;

    wire [10:0] txb0_id;
    wire [63:0] txb0_data;
    wire [3:0]  txb0_dlc;
    wire        txb0_rtr;
    wire        txb0_txreq;
    wire        txb0_wr;

    reg  [10:0] rxb0_id;
    reg  [63:0] rxb0_data;
    reg  [3:0]  rxb0_dlc;
    reg         rxb0_rtr;

    wire [10:0] rxm0_mask;
    wire [10:0] rxf0_id;

    integer checks;
    integer errors;

    reg_bank dut (
        .clk           (clk),
        .rst_n         (rst_n),
        .addr          (addr),
        .wdata         (wdata),
        .we            (we),
        .rdata         (rdata),
        .control_reset (control_reset),
        .txreq_set     (txreq_set),
        .txreq_clear   (txreq_clear),
        .set_rx0if     (set_rx0if),
        .clear_rx0if   (clear_rx0if),
        .set_tx0if     (set_tx0if),
        .clear_tx0if   (clear_tx0if),
        .set_merrf     (set_merrf),
        .clear_merrf   (clear_merrf),
        .set_errif     (set_errif),
        .tec_in        (tec_in),
        .rec_in        (rec_in),
        .canctrl       (canctrl),
        .bfpctrl       (bfpctrl),
        .caninte       (caninte),
        .canintf       (canintf),
        .opmod         (opmod),
        .config_mode   (config_mode),
        .normal_mode   (normal_mode),
        .txb0_id       (txb0_id),
        .txb0_data     (txb0_data),
        .txb0_dlc      (txb0_dlc),
        .txb0_rtr      (txb0_rtr),
        .txb0_txreq    (txb0_txreq),
        .txb0_wr       (txb0_wr),
        .rxb0_id       (rxb0_id),
        .rxb0_data     (rxb0_data),
        .rxb0_dlc      (rxb0_dlc),
        .rxb0_rtr      (rxb0_rtr),
        .rxm0_mask     (rxm0_mask),
        .rxf0_id       (rxf0_id)
    );

    always #5 clk = ~clk;

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

    task write_reg;
        input [7:0] a;
        input [7:0] d;
        begin
            @(negedge clk);
            addr  = a;
            wdata = d;
            we    = 1'b1;
            @(posedge clk);
            #1;
            we    = 1'b0;
        end
    endtask

    task read_check;
        input [7:0] a;
        input [7:0] expected;
        input [8*100-1:0] message;
        begin
            addr = a;
            #1;
            check(rdata === expected, message);
        end
    endtask

    task pulse_set_rx;
        begin
            @(negedge clk);
            set_rx0if = 1'b1;
            @(posedge clk);
            #1;
            set_rx0if = 1'b0;
        end
    endtask

    initial begin
        clk           = 1'b0;
        rst_n         = 1'b0;
        addr          = 8'h00;
        wdata         = 8'h00;
        we            = 1'b0;
        control_reset = 1'b0;
        txreq_set     = 1'b0;
        txreq_clear   = 1'b0;
        set_rx0if     = 1'b0;
        clear_rx0if   = 1'b0;
        set_tx0if     = 1'b0;
        clear_tx0if   = 1'b0;
        set_merrf     = 1'b0;
        clear_merrf   = 1'b0;
        set_errif     = 1'b0;
        tec_in        = 8'h00;
        rec_in        = 8'h00;

        rxb0_id       = 11'h5A5;
        rxb0_data     = 64'h1122_3344_5566_7788;
        rxb0_dlc      = 4'd8;
        rxb0_rtr      = 1'b0;

        checks = 0;
        errors = 0;

        // Hardware reset defaults.
        #20;
        rst_n = 1'b1;
        @(posedge clk);
        #1;

        check(opmod === 3'b000 && normal_mode === 1'b1 && config_mode === 1'b0,
              "controller is fixed in Normal mode");
        check(canctrl === 8'h00, "CANCTRL reset value is Normal mode");
        check(bfpctrl === 8'h00, "BFPCTRL resets to zero");
        check(caninte === 8'h00 && canintf === 8'h00,
              "interrupt registers reset to zero");
        check(txb0_txreq === 1'b0, "TXREQ resets clear");
        check(rxm0_mask === 11'h7FF && rxf0_id === 11'h000,
              "filter/mask use hardcoded SOW values");

        // Writable SOW registers, with unsupported interrupt bits masked off.
        write_reg(8'h0F, 8'hFF);
        read_check(8'h0F, 8'h1F, "CANCTRL mode bits remain locked to Normal");

        write_reg(8'h0C, 8'hA6);
        read_check(8'h0C, 8'hA6, "BFPCTRL write/read works");

        write_reg(8'h2B, 8'hFF);
        read_check(8'h2B, 8'hA5, "CANINTE retains only MERRF ERRIF TX0IF RX0IF");

        // Filter and mask are readable but immutable.
        write_reg(8'h00, 8'hAA);
        write_reg(8'h20, 8'h00);
        read_check(8'h00, 8'h00, "RXF0SIDH ignores host writes");
        read_check(8'h20, 8'hFF, "RXM0SIDH ignores host writes");
        read_check(8'h21, 8'hE0, "RXM0SIDL remains hardcoded");

        // TX message packing. D0 is the MSB byte because the Protocol Engine
        // serializes txb0_data MSB-first.
        write_reg(8'h31, 8'hAA);
        write_reg(8'h32, 8'hA0);
        write_reg(8'h35, 8'h47); // RTR=1, DLC=7
        write_reg(8'h36, 8'h11);
        write_reg(8'h37, 8'h22);
        write_reg(8'h38, 8'h33);
        write_reg(8'h39, 8'h44);
        write_reg(8'h3A, 8'h55);
        write_reg(8'h3B, 8'h66);
        write_reg(8'h3C, 8'h77);
        write_reg(8'h3D, 8'h88);

        check(txb0_id === 11'h555, "TX SIDH/SIDL assemble standard ID correctly");
        check(txb0_rtr === 1'b1 && txb0_dlc === 4'd7,
              "TX DLC register maps RTR and DLC correctly");
        check(txb0_data === 64'h1122_3344_5566_7788,
              "TX D0..D7 map to serializer order correctly");
        read_check(8'h36, 8'h11, "TXB0D0 readback is first payload byte");
        read_check(8'h3D, 8'h88, "TXB0D7 readback is last payload byte");

        // DLC is constrained to the SOW maximum of 8 bytes.
        write_reg(8'h35, 8'h0F);
        check(txb0_dlc === 4'd8, "DLC values above 8 saturate to 8");

        // TXREQ SET wins a simultaneous CLEAR.
        @(negedge clk);
        txreq_set   = 1'b1;
        txreq_clear = 1'b1;
        @(posedge clk);
        #1;
        check(txb0_txreq === 1'b1, "TXREQ SET wins simultaneous CLEAR");
        txreq_set   = 1'b0;
        txreq_clear = 1'b0;

        @(negedge clk);
        txreq_clear = 1'b1;
        @(posedge clk);
        #1;
        txreq_clear = 1'b0;
        check(txb0_txreq === 1'b0, "TXREQ clears on control event");

        // CANINTF hardware set and host clear behavior.
        pulse_set_rx;
        check(canintf[0] === 1'b1, "hardware RX event sets RX0IF");
        write_reg(8'h2C, 8'hFE);
        check(canintf[0] === 1'b0, "host write-zero clears RX0IF");

        // Hardware SET wins simultaneous hardware CLEAR.
        @(negedge clk);
        set_rx0if   = 1'b1;
        clear_rx0if = 1'b1;
        @(posedge clk);
        #1;
        set_rx0if   = 1'b0;
        clear_rx0if = 1'b0;
        check(canintf[0] === 1'b1, "RX0IF SET wins simultaneous CLEAR");

        // Error and TX flags are stored in their reduced-design positions.
        @(negedge clk);
        set_tx0if = 1'b1;
        set_errif = 1'b1;
        set_merrf = 1'b1;
        @(posedge clk);
        #1;
        set_tx0if = 1'b0;
        set_errif = 1'b0;
        set_merrf = 1'b0;
        check(canintf === 8'hA5, "all supported hardware interrupt flags latch");

        // Error counter snapshots.
        tec_in = 8'h96;
        rec_in = 8'h35;
        @(posedge clk);
        #1;
        read_check(8'h1C, 8'h96, "TEC readback mirrors Control Logic/Protocol Engine");
        read_check(8'h1D, 8'h35, "REC readback mirrors Control Logic/Protocol Engine");

        // RX window presents the physical RX buffer in host byte order.
        read_check(8'h61, rxb0_id[10:3], "RXB0SIDH reads stored RX ID");
        read_check(8'h65, 8'h08, "RXB0DLC reads stored RX DLC");
        read_check(8'h66, 8'h11, "RXB0D0 is first received payload byte");
        read_check(8'h6D, 8'h88, "RXB0D7 is last received payload byte");

        // Software/control reset clears all mutable storage.
        @(negedge clk);
        control_reset = 1'b1;
        @(posedge clk);
        #1;
        control_reset = 1'b0;
        check(canctrl === 8'h00 && bfpctrl === 8'h00,
              "control reset restores control-register defaults");
        check(caninte === 8'h00 && canintf === 8'h00 && txb0_txreq === 1'b0,
              "control reset clears interrupt and TXREQ state");

        $display("");
        $display("============================================================");
        $display("REGISTER BANK TESTS: %0d checks, %0d errors", checks, errors);
        if (errors == 0)
            $display("RESULT: PASS");
        else
            $display("RESULT: FAIL");
        $display("============================================================");
        $finish;
    end

endmodule
