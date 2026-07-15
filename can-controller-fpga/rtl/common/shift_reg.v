module shift_reg #( parameter WIDTH = 8) (
    input  wire             clk,
    input  wire             rst_n,      // active-low async reset
    input  wire             load,       // parallel load put entire byte to shift reg
    input  wire             shift_en,   // enable shifting
    input  wire             serial_in,
    input  wire [WIDTH-1:0] parallel_in,
    output wire             serial_out,
    output wire [WIDTH-1:0] parallel_out
);
    reg [WIDTH-1:0] data_reg;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            data_reg <= {WIDTH{1'b0}};
        else if (load)
            data_reg <= parallel_in;
        else if (shift_en)
            data_reg <= {data_reg[WIDTH-2:0], serial_in};
    end

    assign parallel_out = data_reg;
    assign serial_out   = data_reg[WIDTH-1];   // MSB-first
endmodule
