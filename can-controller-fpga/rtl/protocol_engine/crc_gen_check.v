`include "common/can_defs.vh"

// =============================================================================
// Module      : crc_gen_check
// Description : CAN CRC-15 accumulator/checker for the Protocol Engine.
//               Implements the public CAN 2.0B CRC-15 polynomial without
//               reflection or final XOR, using the same active-low reset and
//               dominant/recessive polarity conventions as the rest of the
//               protocol engine.
// =============================================================================

module crc_gen_check (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        crc_init,
    input  wire        bit_strobe,
    input  wire        logical_bit,
    input  wire        crc_field_rx,
    input  wire        crc_latch,
    output reg  [14:0] crc_out,
    output reg         crc_error
);

    localparam [14:0] CRC_POLY = `CAN_CRC15_POLY;

    reg [14:0] crc_accum;
    reg [14:0] crc_rx_capture;
    reg [4:0]  crc_rx_bit_count;
    reg [14:0] crc_accum_next;

    function [14:0] crc_step;
        input [14:0] current;
        input        bit_in;
        begin
            crc_step = current << 1;
            if ((current[14] ^ bit_in) == 1'b1)
                crc_step = crc_step ^ CRC_POLY;
        end
    endfunction

    always @(*) begin
        crc_accum_next = crc_accum;
        if (bit_strobe && !crc_field_rx)
            crc_accum_next = crc_step(crc_accum, logical_bit);
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            crc_accum      <= 15'h0000;
            crc_out        <= 15'h0000;
            crc_rx_capture <= 15'h0000;
            crc_rx_bit_count <= 5'd0;
            crc_error      <= 1'b0;
        end
        else begin
            crc_error <= 1'b0;

            if (crc_init) begin
                crc_accum       <= 15'h0000;
                crc_out         <= 15'h0000;
                crc_rx_capture  <= 15'h0000;
                crc_rx_bit_count <= 5'd0;
            end

            if (bit_strobe) begin
                if (crc_field_rx) begin
                    // Capture the 15 received CRC bits after the accumulate window.
                    if (crc_rx_bit_count == 5'd14) begin
                        crc_rx_capture <= {crc_rx_capture[13:0], logical_bit};
                        crc_rx_bit_count <= 5'd0;
                        if ({crc_rx_capture[13:0], logical_bit} != crc_out)
                            crc_error <= 1'b1;
                    end
                    else begin
                        crc_rx_capture <= {crc_rx_capture[13:0], logical_bit};
                        crc_rx_bit_count <= crc_rx_bit_count + 5'd1;
                    end
                end
                else begin
                    crc_accum <= crc_accum_next;
                end
            end

            if (crc_latch)
                crc_out <= crc_accum_next;
        end
    end

endmodule
