module rxcan_sync (
    input  wire clk,
    input  wire rst_n,
    input  wire rx_pin,
    output reg  rx_can_sync
);

    reg rx_ff0, rx_ff1;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_ff0     <= 1'b1;
            rx_ff1     <= 1'b1;
            rx_can_sync <= 1'b1;
        end
        else begin
            rx_ff0      <= rx_pin;
            rx_ff1      <= rx_ff0;
            rx_can_sync <= rx_ff1;
        end
    end
endmodule
