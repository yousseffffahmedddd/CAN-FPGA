module flag_reg #(parameter WIDTH = 1) (
    input  wire                  clk,
    input  wire                  rst_n,
    input  wire [WIDTH-1:0]      set_value,
    input  wire                  set,
    input  wire                  clear,
    output reg  [WIDTH-1:0]      q
);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            q <= {WIDTH{1'b0}};
        else if (set)
            q <= set_value;
        else if (clear)
            q <= {WIDTH{1'b0}};
    end
endmodule
