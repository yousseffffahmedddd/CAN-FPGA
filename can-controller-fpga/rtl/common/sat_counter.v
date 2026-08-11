module sat_counter #(
    parameter integer WIDTH = 8,
    parameter integer MAX_VAL = 8'hff
)(
    input  wire                  clk,
    input  wire                  rst_n,
    input  wire                  enable,
    input  wire                  load,
    input  wire [WIDTH-1:0]      load_value,
    output reg  [WIDTH-1:0]      count,
    output reg                   overflow
);

    localparam [WIDTH-1:0] MAX_COUNT = MAX_VAL[WIDTH-1:0];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            count    <= {WIDTH{1'b0}};
            overflow <= 1'b0;
        end
        else if (load) begin
            count    <= load_value;
            overflow <= 1'b0;
        end
        else if (enable) begin
            if (count == MAX_COUNT) begin
                count    <= MAX_COUNT;
                overflow <= 1'b1;
            end
            else begin
                count    <= count + 1'b1;
                overflow <= 1'b0;
            end
        end
        else begin
            overflow <= 1'b0;
        end
    end
endmodule
