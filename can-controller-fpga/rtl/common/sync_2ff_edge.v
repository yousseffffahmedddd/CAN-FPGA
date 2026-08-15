// module sync_2ff_edge (
//     input  wire clk,
//     input  wire rst_n,
//     input  wire d,
//     output reg  sync_out,
//     output reg  posedge_pulse,
//     output reg  negedge_pulse
// );

//     reg q0;
//     reg q1;
//     reg q2;

//     always @(posedge clk or negedge rst_n) begin
//         if (!rst_n) begin
//             q0             <= 1'b0;
//             q1             <= 1'b0;
//             q2             <= 1'b0;
//             sync_out       <= 1'b0;
//             posedge_pulse  <= 1'b0;
//             negedge_pulse  <= 1'b0;
//         end else begin
//             q0            <= d;
//             q1            <= q0;
//             q2            <= q1;
//             sync_out      <= q1;
//             posedge_pulse <= q1 && !q2;
//             negedge_pulse <= !q1 && q2;
//         end
//     end

// endmodule
