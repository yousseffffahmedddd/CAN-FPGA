// module cdc_handshake #(
//     parameter integer DATA_WIDTH = 8
// )(
//     input  wire                     src_clk,
//     input  wire                     src_rst_n,
//     input  wire [DATA_WIDTH-1:0]    src_data,
//     input  wire                     src_valid,
//     output reg                      src_busy,

//     input  wire                     dst_clk,
//     input  wire                     dst_rst_n,
//     output reg [DATA_WIDTH-1:0]     dst_data,
//     output reg                      valid_pulse,
//     output reg                      dst_ack
// );

//     reg [DATA_WIDTH-1:0] src_hold_data;
//     reg                 req;
//     wire                req_to_dst;
//     wire                ack_from_dst;
//     wire                ack_sync_pulse;

//     sync_2ff_edge req_sync_inst (
//         .clk          (dst_clk),
//         .rst_n        (dst_rst_n),
//         .d            (req),
//         .sync_out     (req_to_dst),
//         .posedge_pulse(),
//         .negedge_pulse()
//     );

//     sync_2ff_edge ack_sync_inst (
//         .clk          (src_clk),
//         .rst_n        (src_rst_n),
//         .d            (dst_ack),
//         .sync_out     (ack_from_dst),
//         .posedge_pulse(ack_sync_pulse),
//         .negedge_pulse()
//     );

//     always @(posedge src_clk or negedge src_rst_n) begin
//         if (!src_rst_n) begin
//             src_hold_data <= {DATA_WIDTH{1'b0}};
//             src_busy      <= 1'b0;
//             req           <= 1'b0;
//         end else begin
//             if (src_valid && !src_busy) begin
//                 src_hold_data <= src_data;
//                 req           <= 1'b1;
//                 src_busy      <= 1'b1;
//             end else if (ack_sync_pulse) begin
//                 req           <= 1'b0;
//                 src_busy      <= 1'b0;
//             end
//         end
//     end

//     always @(posedge dst_clk or negedge dst_rst_n) begin
//         if (!dst_rst_n) begin
//             dst_data     <= {DATA_WIDTH{1'b0}};
//             valid_pulse  <= 1'b0;
//             dst_ack      <= 1'b0;
//         end else begin
//             if (req_to_dst) begin
//                 dst_data    <= src_hold_data;
//                 valid_pulse <= 1'b1;
//                 dst_ack     <= 1'b1;
//             end else begin
//                 valid_pulse <= 1'b0;
//                 dst_ack     <= 1'b0;
//             end
//         end
//     end
// endmodule
