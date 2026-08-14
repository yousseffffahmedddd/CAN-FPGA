module bit_clk_gen #(
    parameter integer CLKS_PER_TQ = 1,
    parameter integer TQ_PER_BIT  = 8
)(
    input  wire clk,
    input  wire rst_n,
    input  wire hard_sync,

    output reg  tq_pulse,
    output reg  [($clog2(TQ_PER_BIT) > 0 ? $clog2(TQ_PER_BIT) : 1)-1:0] tq_idx
);

    localparam integer CLK_CNT_W = (CLKS_PER_TQ <= 1) ? 1 : $clog2(CLKS_PER_TQ);
    localparam integer TQ_CNT_W  = (TQ_PER_BIT <= 1) ? 1 : $clog2(TQ_PER_BIT);

    reg [CLK_CNT_W-1:0] clk_div_cnt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            clk_div_cnt <= {CLK_CNT_W{1'b0}};
            tq_pulse    <= 1'b0;
            tq_idx      <= {TQ_CNT_W{1'b0}};
        end
        else if (hard_sync) begin
            clk_div_cnt <= {CLK_CNT_W{1'b0}};
            tq_pulse    <= 1'b1;
            tq_idx      <= {TQ_CNT_W{1'b0}};
        end
        else if (CLKS_PER_TQ == 1) begin
            clk_div_cnt <= {CLK_CNT_W{1'b0}};
            tq_pulse    <= 1'b1;
            if (tq_idx == TQ_PER_BIT - 1)
                tq_idx <= {TQ_CNT_W{1'b0}};
            else
                tq_idx <= tq_idx + 1'b1;
        end
        else if (clk_div_cnt == CLKS_PER_TQ - 1) begin
            clk_div_cnt <= {CLK_CNT_W{1'b0}};
            tq_pulse    <= 1'b1;
            if (tq_idx == TQ_PER_BIT - 1)
                tq_idx <= {TQ_CNT_W{1'b0}};
            else
                tq_idx <= tq_idx + 1'b1;
        end
        else begin
            clk_div_cnt <= clk_div_cnt + 1'b1;
            tq_pulse    <= 1'b0;
        end
    end
endmodule
