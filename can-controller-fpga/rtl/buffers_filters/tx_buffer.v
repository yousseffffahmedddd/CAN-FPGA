module tx_buffer (
    input  wire        clk,
    input  wire        reset,        // synchronous, active-high
    // Write side (from Register Bank / SPI)
    input  wire        write_enable, // CPU writes a new message
    input  wire [10:0] tx_id,        // 11-bit standard CAN ID
    input  wire [63:0] tx_data,      // up to 8 data bytes
    input  wire [3:0]  tx_dlc,       // data length code (0-8)
    input  wire        tx_rtr,       // 1 = remote frame (no data bytes), 0 = data frame
    input  wire        txreq,        // "this message is ready to send"
    // Read side (to CAN Transmitter)
    input  wire        tx_done,      // transmitter finished sending
    output reg  [10:0] id,
    output reg  [63:0] data,
    output reg  [3:0]  dlc,
    output reg         rtr,
    output reg         ready         // 1 = message pending transmission
);
    always @(posedge clk) begin
        if (reset) begin
            id    <= 11'b0;
            data  <= 64'b0;
            dlc   <= 4'b0;
            rtr   <= 1'b0;
            ready <= 1'b0;
        end
        else begin
            // Store a new message only if no message is currently pending
            // transmission -- prevents an in-flight message from being
            // overwritten before it's actually sent (SOW: "retain stored
            // data until transmission is completed").
            if (write_enable && !ready) begin
                id   <= tx_id;
                data <= tx_data;
                dlc  <= tx_dlc;
                rtr  <= tx_rtr;
            end
            // Mark ready when CPU requests transmission
            if (txreq)
                ready <= 1'b1;
            // Clear ready once the transmitter reports done
            else if (tx_done)
                ready <= 1'b0;
        end
    end
endmodule
