module rx_buffer (
    input  wire        clk,
    input  wire        reset,        // synchronous, active-high
    // Write side (from CAN Receiver, gated by Acceptance Filter)
    input  wire        write_enable, // = accept signal from filter
    input  wire [10:0] rx_id,
    input  wire [63:0] rx_data,
    input  wire [3:0]  rx_dlc,
    input  wire        rx_rtr,       // 1 = remote frame (no data bytes), 0 = data frame
    // Read side (from CPU / Register Bank)
    input  wire        cpu_read,     // CPU has finished reading
    output reg         full,         // 1 = message waiting for CPU
    output reg  [10:0] id,
    output reg  [63:0] data,
    output reg  [3:0]  dlc,
    output reg         rtr
);
    always @(posedge clk) begin
        if (reset) begin
            id   <= 11'b0;
            data <= 64'b0;
            dlc  <= 4'b0;
            rtr  <= 1'b0;
            full <= 1'b0;
        end
        else begin
            // Store new message only if buffer is not already full
            if (write_enable && !full) begin
                id   <= rx_id;
                data <= rx_data;
                dlc  <= rx_dlc;
                rtr  <= rx_rtr;
                full <= 1'b1;
            end
            // CPU has read the message -> buffer becomes empty
            else if (cpu_read) begin
                full <= 1'b0;
            end
        end
    end
endmodule