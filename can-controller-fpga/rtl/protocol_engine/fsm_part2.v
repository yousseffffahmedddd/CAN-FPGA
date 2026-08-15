// // `include "../common/can_defs.vh"

// // // =============================================================================
// // // Module      : fsm_part2
// // // Description : Core field-level FSM for the CAN Protocol Engine.
// // //                Sequences IDLE -> SOF -> ARBITRATION -> CONTROL -> DATA -> CRC.
// // //                Handles non-destructive arbitration, PISO/SIPO handshaking,
// // //                and DLC latching. CRC field bit-count/exit logic and beyond
// // //                (CRC_DELIM, ACK, EOF, Intermission) are out of scope for this
// // //                module and are intended to be implemented in fsm_part3.
// // // Coding style: 3-process FSM (state register / next-state comb / datapath).
// // // =============================================================================
module fsm_part2 (
    input wire clk,
    input wire rst_n,
    input wire bit_tick,
    input wire rx_can_sync,
    input wire txb_txreq,           // Added: TX request input
    input wire [3:0] dlc,           // Added: DLC input for 44+8n frame support
    output reg tx_can,
    output reg tx_en,
    input wire piso_data_in,
    input wire piso_valid,
    output reg piso_req,
    output reg sipo_data_out,
    output reg sipo_valid,
    output reg bit_error,
    output reg arb_lost,
    output reg [3:0] latched_dlc,
    output reg [2:0] current_state
);

    // State definitions
    localparam STATE_IDLE        = 3'd0;
    localparam STATE_SOF         = 3'd1;
    localparam STATE_ARBITRATION = 3'd2;
    localparam STATE_CONTROL     = 3'd3;
    localparam STATE_DATA        = 3'd4;
    localparam STATE_CRC         = 3'd5;
    localparam STATE_ACK         = 3'd6;
    localparam STATE_EOF         = 3'd7;

    reg [2:0] next_state;
    reg rx_prev;
    reg [6:0] bit_cnt;

    // Edge detector for incoming frames
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            rx_prev <= 1'b1;
        else if (bit_tick)
            rx_prev <= rx_can_sync;
    end

    wire sof_edge = bit_tick && (rx_prev == 1'b1) && (rx_can_sync == 1'b0);

    // Next State Logic
    always @(*) begin
        next_state = current_state;
        case (current_state)
            STATE_IDLE: begin
                // Fixed stall: transition on explicit transmission request or incoming bus edge
                if (txb_txreq || sof_edge)
                    next_state = STATE_SOF;
            end

            STATE_SOF: begin
                if (bit_tick)
                    next_state = STATE_ARBITRATION;
            end

            STATE_ARBITRATION: begin
                if (bit_tick && (bit_cnt == 7'd11))
                    next_state = STATE_CONTROL;
            end

            STATE_CONTROL: begin
                if (bit_tick && (bit_cnt == 7'd5))
                    next_state = STATE_DATA;
            end

            STATE_DATA: begin
                // Supports 44 + 8n frame structure using dlc
                if (bit_tick && (bit_cnt == ((dlc == 4'd0) ? 7'd0 : (dlc * 8 - 1))))
                    next_state = STATE_CRC;
            end

            STATE_CRC: begin
                if (bit_tick && (bit_cnt == 7'd15))
                    next_state = STATE_ACK;
            end

            STATE_ACK: begin
                if (bit_tick && (bit_cnt == 7'd1))
                    next_state = STATE_EOF;
            end

            STATE_EOF: begin
                if (bit_tick && (bit_cnt == 7'd6))
                    next_state = STATE_IDLE;
            end

            default: next_state = STATE_IDLE;
        endcase
    end

    // Sequential Logic and Outputs
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            current_state <= STATE_IDLE;
            bit_cnt       <= 7'd0;
            tx_en         <= 1'b0;
            tx_can        <= 1'b1;
            arb_lost      <= 1'b0;
            piso_req      <= 1'b0;
            latched_dlc   <= 4'd0;
        end else if (bit_tick) begin
            if (current_state != next_state) begin
                bit_cnt       <= 7'd0;
                current_state <= next_state;
                if (next_state == STATE_CONTROL)
                    latched_dlc <= dlc;
            end else begin
                bit_cnt       <= bit_cnt + 7'd1;
            end

            case (next_state)
                STATE_IDLE: begin
                    tx_en    <= txb_txreq;
                    tx_can   <= txb_txreq ? 1'b0 : 1'b1;
                    arb_lost <= 1'b0;
                    piso_req <= 1'b0;
                end

                STATE_SOF: begin
                    tx_en    <= 1'b1;
                    tx_can   <= 1'b0; // Dominant SOF
                    piso_req <= 1'b1;
                end

                STATE_ARBITRATION, STATE_CONTROL, STATE_DATA: begin
                    tx_en    <= txb_txreq;
                    tx_can   <= piso_data_in;
                    piso_req <= piso_valid;
                end

                default: begin
                    tx_en    <= 1'b0;
                    tx_can   <= 1'b1;
                    piso_req <= 1'b0;
                end
            endcase
        end
    end

endmodule

// // module fsm_part2 (
// //     // ---------------------------------------------------------------
// //     // Clock / Reset
// //     // ---------------------------------------------------------------
// //     input  wire        clk,            // System clock
// //     input  wire        rst_n,          // no

// //     // ---------------------------------------------------------------
// //     // Bit timing / bus interface
// //     // ---------------------------------------------------------------
// //     input  wire        bit_tick,       // Pulse once per bit period @ sample point
// //     input  wire        rx_can_sync,    // Synchronized RXCAN (1=recessive,0=dominant)
// //     output reg         tx_can,         // TX CAN bit (1=recessive, 0=dominant)
// //     output reg         tx_en,          // 1 = driving bus, 0 = Hi-Z / receive-only

// //     // ---------------------------------------------------------------
// //     // PISO interface (Frame Constructor -> this FSM, bits to transmit)
// //     // ---------------------------------------------------------------
// //     input  wire        piso_data_in,   // 1-bit data from PISO
// //     input  wire        piso_valid,     // piso_data_in valid for consumption
// //     output reg         piso_req,       // Bit request to PISO interface

// //     // ---------------------------------------------------------------
// //     // SIPO interface (this FSM -> Frame Deconstructor, observed bits)
// //     // ---------------------------------------------------------------
// //     output reg         sipo_data_out,  // 1-bit data to SIPO (= observed rx_can_sync)
// //     output reg         sipo_valid,     // High for 1 cycle per bit produced

// //     // ---------------------------------------------------------------
// //     // Status / error flags
// //     // ---------------------------------------------------------------
// //     output reg         bit_error,      // Bit error flag
// //     output reg         arb_lost,       // Arbitration lost flag
// //     output reg  [3:0]  latched_dlc,    // Latched 4-bit Data Length Code
// //     output reg  [2:0]  current_state   // Debug/monitoring state output
// // );

// //     // =========================================================================
// //     // Req #2: Explicit FSM state encoding
// //     // =========================================================================
// //     localparam [2:0] STATE_IDLE        = `CAN_STATE_IDLE;
// //     localparam [2:0] STATE_SOF         = `CAN_STATE_SOF;
// //     localparam [2:0] STATE_ARBITRATION = `CAN_STATE_ARBITRATION;
// //     localparam [2:0] STATE_CONTROL     = `CAN_STATE_CONTROL;
// //     localparam [2:0] STATE_DATA        = `CAN_STATE_DATA;
// //     localparam [2:0] STATE_CRC         = `CAN_STATE_CRC;

// //     // Field bit widths (used for bit_cnt terminal-count comparisons)
// //     localparam integer ARB_BITS = 12; // 11-bit ID + 1-bit RTR
// //     localparam integer CTRL_BITS = 6; // IDE(1) + RB0(1) + DLC(4)

// //     // =========================================================================
// //     // Internal registers
// //     // =========================================================================
// //     reg [2:0] state, next_state;

// //     reg [6:0] bit_cnt;      // Shared per-field bit counter, reset on state entry
// //     reg [6:0] data_bit_target; // Latched (latched_dlc*8 - 1) target for DATA state

// //     reg       rx_prev;      // Previous sampled rx_can_sync, for edge detection (Req #1)
// //     reg       tx_bit_reg;   // Bit latched from PISO, drives tx_can during TX fields

// //     wire      sof_edge;     // Recessive->Dominant edge detected at bit_tick (Req #1)
// //     assign    sof_edge = bit_tick && (rx_prev == 1'b1) && (rx_can_sync == 1'b0);

// //     // =========================================================================
// //     // Process 1: State register (sequential)
// //     // =========================================================================
// //     always @(posedge clk or negedge rst_n) begin
// //         if (!rst_n)
// //             state <= STATE_IDLE;
// //         else
// //             state <= next_state;
// //     end

// //     always @(*) current_state = state; // Req: current_state debug output

// //     // =========================================================================
// //     // Process 2: Next-state combinational logic
// //     // =========================================================================
// //       reg [3:0] latched_dlc_next;
// //     always @(*) begin
// //         next_state = state; // default: hold

// //         case (state)

// //             // -----------------------------------------------------------
// //             // Req #1 - IDLE: watch for SOF edge on every bit_tick
// //             // -----------------------------------------------------------
// //             STATE_IDLE: begin
// //                 if (sof_edge)
// //                     next_state = STATE_SOF;
// //             end

// //             // -----------------------------------------------------------
// //             // Req #2 - SOF: exactly one bit_tick, then move on
// //             // -----------------------------------------------------------
// //             STATE_SOF: begin
// //                 if (bit_tick)
// //                     next_state = STATE_ARBITRATION;
// //             end

// //             // -----------------------------------------------------------
// //             // Req #3 - ARBITRATION: 12 bits (ID[10:0] + RTR)
// //             // -----------------------------------------------------------
// //             STATE_ARBITRATION: begin
// //                 if (bit_tick && (bit_cnt == ARB_BITS - 1))
// //                     next_state = STATE_CONTROL;
// //             end

// //             // -----------------------------------------------------------
// //             // Req #4 - CONTROL: 6 bits (IDE, RB0, DLC[3:0])
// //             //          Bypass DATA state entirely if latched_dlc == 0
// //             // -----------------------------------------------------------
// //             STATE_CONTROL: begin
// //                 if (bit_tick && (bit_cnt == CTRL_BITS - 1)) begin
// //                     // latched_dlc has been fully shifted in by this tick (Process 3)
// //                     if (latched_dlc_next == 4'd0)
// //                         next_state = STATE_CRC;
// //                     else
// //                         next_state = STATE_DATA;
// //                 end
// //             end

// //             // -----------------------------------------------------------
// //             // Req #5 - DATA: latched_dlc * 8 bits
// //             // -----------------------------------------------------------
// //             STATE_DATA: begin
// //                 if (bit_tick && (bit_cnt == data_bit_target))
// //                     next_state = STATE_CRC;
// //             end

// //             // -----------------------------------------------------------
// //             // STATE_CRC: entry point only - CRC field bit handling and
// //             // exit transition are implemented in fsm_part3 (out of scope here)
// //             // -----------------------------------------------------------
// //             STATE_CRC: begin
// //                 next_state = STATE_CRC; // placeholder hold
// //             end

// //             default: next_state = STATE_IDLE;
// //         endcase
// //     end

// //     // =========================================================================
// //     // Combinational lookahead for DLC (needed by next-state logic above,
// //     // since latched_dlc itself is only updated in Process 3 on the same edge)
// //     // =========================================================================
  
// //     always @(*) begin
// //         latched_dlc_next = latched_dlc;
// //         if (state == STATE_CONTROL && bit_tick && bit_cnt >= 2)
// //             latched_dlc_next = {latched_dlc[2:0], rx_can_sync};
// //     end

// //     // =========================================================================
// //     // Process 3: Sequential datapath - counters, flags, TX/RX bit handling
// //     // =========================================================================
// //     always @(posedge clk or negedge rst_n) begin
// //         if (!rst_n) begin
// //             bit_cnt         <= 7'd0;
// //             data_bit_target <= 7'd0;
// //             rx_prev         <= 1'b1;      // bus idles recessive
// //             tx_bit_reg      <= `CAN_RECESSIVE;
// //             tx_can          <= `CAN_RECESSIVE;      // recessive by default
// //             tx_en           <= 1'b0;
// //             piso_req        <= 1'b0;
// //             sipo_data_out   <= 1'b0;
// //             sipo_valid      <= 1'b0;
// //             latched_dlc     <= 4'd0;
// //         end
// //         else begin

// //             // ---- Req #1: track previous rx_can_sync for edge detection ----
// //             if (bit_tick)
// //                 rx_prev <= rx_can_sync;

// //             // ---- default: single-cycle pulses deassert unless re-asserted ----
// //             sipo_valid <= 1'b0;
// //             bit_error  <= 1'b0;

// //             case (state)

// //                 // -------------------------------------------------------
// //                 // IDLE: nothing driven, wait for sof_edge (see Process 2)
// //                 // -------------------------------------------------------
// //                 STATE_IDLE: begin
// //                     bit_cnt  <= 7'd0;
// //                     tx_en    <= 1'b0;
// //                     tx_can   <= `CAN_RECESSIVE;    // recessive
// //                     arb_lost <= 1'b0;    // clear stale flag entering new frame
// //                 end

// //                 // -------------------------------------------------------
// //                 // Req #2 - SOF: drive one dominant bit for one bit_tick
// //                 // -------------------------------------------------------
// //                 STATE_SOF: begin
// //                     tx_en  <= 1'b1;
// //                     tx_can <= `CAN_DOMINANT;      // dominant
// //                     if (bit_tick) begin
// //                         bit_cnt       <= 7'd0;      // reset for next field (ARBITRATION)
// //                         // Req #6: SIPO tap - SOF bit is also "observed"
// //                         sipo_data_out <= rx_can_sync;
// //                         sipo_valid    <= 1'b1;
// //                     end
// //                 end

// //                 // -------------------------------------------------------
// //                 // Req #3 - ARBITRATION: 12 bits, non-destructive arbitration
// //                 // -------------------------------------------------------
// //                 STATE_ARBITRATION: begin
// //                     // Request/consume next bit from PISO whenever still transmitting
// //                     piso_req <= (tx_en == 1'b1);
// //                     if (piso_req && piso_valid)
// //                         tx_bit_reg <= piso_data_in;

// //                     if (tx_en)
// //                         tx_can <= tx_bit_reg;   // drive our arbitration bit
// //                     // else: arbitration already lost, stay receive-only (tx_can holds recessive)

// //                     if (bit_tick) begin
// //                         // --- Non-destructive arbitration compare ---
// //                         if (tx_en && (tx_can == 1'b1) && (rx_can_sync == 1'b0)) begin
// //                             arb_lost <= 1'b1;   // lost: we sent recessive, bus is dominant
// //                             tx_en    <= 1'b0;   // drop to receive-only, NOT a bit_error
// //                         end

// //                         // Req #6: SIPO tap - always the observed bus bit
// //                         sipo_data_out <= rx_can_sync;
// //                         sipo_valid    <= 1'b1;

// //                         if (bit_cnt == ARB_BITS - 1)
// //                             bit_cnt <= 7'd0;    // reset for next field (CONTROL)
// //                         else
// //                             bit_cnt <= bit_cnt + 7'd1;
// //                     end
// //                 end

// //                 // -------------------------------------------------------
// //                 // Req #4 - CONTROL: IDE, RB0, DLC[3:0] (MSB-first)
// //                 // -------------------------------------------------------
// //                 STATE_CONTROL: begin
// //                     piso_req <= (tx_en == 1'b1);
// //                     if (piso_req && piso_valid)
// //                         tx_bit_reg <= piso_data_in;

// //                     if (tx_en)
// //                         tx_can <= tx_bit_reg;

// //                     if (bit_tick) begin
// //                         // Genuine bit error: outside arbitration, driven != observed
// //                         if (tx_en && (tx_can != rx_can_sync))
// //                             bit_error <= 1'b1;

// //                         // DLC accumulation begins at bit index 2 (bits 0,1 = IDE, RB0)
// //                         if (bit_cnt >= 2)
// //                             latched_dlc <= {latched_dlc[2:0], rx_can_sync};

// //                         // Req #6: SIPO tap
// //                         sipo_data_out <= rx_can_sync;
// //                         sipo_valid    <= 1'b1;

// //                         if (bit_cnt == CTRL_BITS - 1) begin
// //                             bit_cnt         <= 7'd0; // reset for next field
// //                             // Precompute DATA field target bit count = DLC*8 - 1
// //                             data_bit_target <= (latched_dlc_next << 3) - 7'd1;
// //                         end
// //                         else
// //                             bit_cnt <= bit_cnt + 7'd1;
// //                     end
// //                 end

// //                 // -------------------------------------------------------
// //                 // Req #5 - DATA: latched_dlc * 8 bits
// //                 // -------------------------------------------------------
// //                 STATE_DATA: begin
// //                     piso_req <= (tx_en == 1'b1);
// //                     if (piso_req && piso_valid)
// //                         tx_bit_reg <= piso_data_in;

// //                     if (tx_en)
// //                         tx_can <= tx_bit_reg;

// //                     if (bit_tick) begin
// //                         if (tx_en && (tx_can != rx_can_sync))
// //                             bit_error <= 1'b1;

// //                         // Req #6: SIPO tap
// //                         sipo_data_out <= rx_can_sync;
// //                         sipo_valid    <= 1'b1;

// //                         if (bit_cnt == data_bit_target)
// //                             bit_cnt <= 7'd0;    // reset for next field (CRC)
// //                         else
// //                             bit_cnt <= bit_cnt + 7'd1;
// //                     end
// //                 end

// //                 // -------------------------------------------------------
// //                 // STATE_CRC: entry only, handled by fsm_part3
// //                 // -------------------------------------------------------
// //                 STATE_CRC: begin
// //                     piso_req <= 1'b0; // no further PISO consumption in this module
// //                 end

// //                 default: begin
// //                     bit_cnt <= 7'd0;
// //                 end
// //             endcase
// //         end
// //     end

// // endmodule
// module fsm_part2 (
//     input wire clk,
//     input wire rst_n,
//     input wire bit_tick,
//     input wire rx_can_sync,
//     input wire txb_txreq,           // TX request input
//     input wire [3:0] dlc,           // DLC input for 44+8n frame support
//     output reg tx_can,
//     output reg tx_en,
//     input wire piso_data_in,
//     input wire piso_valid,
//     output reg piso_req,
//     output reg sipo_data_out,
//     output reg sipo_valid,
//     output reg bit_error,
//     output reg arb_lost,
//     output reg [3:0] latched_dlc,
//     output reg [2:0] current_state
// );

//     // State definitions
//     localparam STATE_IDLE        = 3'd0;
//     localparam STATE_SOF         = 3'd1;
//     localparam STATE_ARBITRATION = 3'd2;
//     localparam STATE_CONTROL     = 3'd3;
//     localparam STATE_DATA        = 3'd4;
//     localparam STATE_CRC         = 3'd5;
//     localparam STATE_ACK         = 3'd6;
//     localparam STATE_EOF         = 3'd7;

//     reg [2:0] next_state;
//     reg rx_prev;
//     reg [6:0] bit_cnt;

//     // Edge detector for incoming frames
//     always @(posedge clk or negedge rst_n) begin
//         if (!rst_n)
//             rx_prev <= 1'b1;
//         else if (bit_tick)
//             rx_prev <= rx_can_sync;
//     end

//     wire sof_edge = bit_tick && (rx_prev == 1'b1) && (rx_can_sync == 1'b0);

//     // Next State Logic
//     always @(*) begin
//         next_state = current_state;
//         case (current_state)
//             STATE_IDLE: begin
//                 // Fixed stall: transition on explicit transmission request or incoming bus edge
//                 if (txb_txreq || sof_edge)
//                     next_state = STATE_SOF;
//             end

//             STATE_SOF: begin
//                 if (bit_tick)
//                     next_state = STATE_ARBITRATION;
//             end

//             STATE_ARBITRATION: begin
//                 if (bit_tick && (bit_cnt == 7'd11))
//                     next_state = STATE_CONTROL;
//             end

//             STATE_CONTROL: begin
//                 if (bit_tick && (bit_cnt == 7'd5))
//                     next_state = STATE_DATA;
//             end

//             STATE_DATA: begin
//                 // Supports 44 + 8n frame structure using dlc
//                 if (bit_tick && (bit_cnt == ((dlc == 4'd0) ? 7'd0 : (dlc * 8 - 1))))
//                     next_state = STATE_CRC;
//             end

//             STATE_CRC: begin
//                 if (bit_tick && (bit_cnt == 7'd15))
//                     next_state = STATE_ACK;
//             end

//             STATE_ACK: begin
//                 if (bit_tick && (bit_cnt == 7'd1))
//                     next_state = STATE_EOF;
//             end

//             STATE_EOF: begin
//                 if (bit_tick && (bit_cnt == 7'd6))
//                     next_state = STATE_IDLE;
//             end

//             default: next_state = STATE_IDLE;
//         endcase
//     end

//     // Sequential Logic and Outputs
//     always @(posedge clk or negedge rst_n) begin
//         if (!rst_n) begin
//             current_state <= STATE_IDLE;
//             bit_cnt       <= 7'd0;
//             tx_en         <= 1'b0;
//             tx_can        <= 1'b1;
//             arb_lost      <= 1'b0;
//             piso_req      <= 1'b0;
//             latched_dlc   <= 4'd0;
//         end else if (bit_tick) begin
//             if (current_state != next_state) begin
//                 bit_cnt       <= 7'd0;
//                 current_state <= next_state;
//                 if (next_state == STATE_CONTROL)
//                     latched_dlc <= dlc;
//             end else begin
//                 bit_cnt       <= bit_cnt + 7'd1;
//             end

//             case (next_state)
//                 STATE_IDLE: begin
//                     tx_en    <= txb_txreq;
//                     tx_can   <= txb_txreq ? 1'b0 : 1'b1;
//                     arb_lost <= 1'b0;
//                     piso_req <= 1'b0;
//                 end

//                 STATE_SOF: begin
//                     tx_en    <= 1'b1;
//                     tx_can   <= 1'b0; // Dominant SOF
//                     piso_req <= 1'b1;
//                 end

//                 STATE_ARBITRATION, STATE_CONTROL, STATE_DATA: begin
//                     tx_en    <= txb_txreq;
//                     tx_can   <= piso_data_in;
//                     piso_req <= piso_valid;
//                 end

//                 default: begin
//                     tx_en    <= 1'b0;
//                     tx_can   <= 1'b1;
//                     piso_req <= 1'b0;
//                 end
//             endcase
//         end
//     end

// endmodule

// =============================================================================
// Module      : fsm_part2
// Description : Core field-level FSM for the CAN Protocol Engine.
//               Sequences IDLE -> SOF -> ARBITRATION -> CONTROL -> DATA -> CRC.
//               Handles non-destructive arbitration, PISO/SIPO handshaking,
//               and DLC latching. 
// =============================================================================

// module fsm_part2 (
//     input  wire        clk,            // System clock
//     input  wire        rst_n,          // Asynchronous active-low reset
//     input  wire        bit_tick,       // Pulse once per bit period @ sample point
//     input  wire        rx_can_sync,    // Synchronized RXCAN (1=recessive, 0=dominant)
//     output reg         tx_can,         // TX CAN bit (1=recessive, 0=dominant)
//     output reg         tx_en,          // 1 = driving bus, 0 = Hi-Z / receive-only

//     // PISO interface (Frame Constructor -> this FSM, bits to transmit)
//     input  wire        piso_data_in,   // 1-bit data from PISO
//     input  wire        piso_valid,     // piso_data_in valid for consumption
//     output reg         piso_req,       // Bit request to PISO interface

//     // SIPO interface (this FSM -> Frame Deconstructor, observed bits)
//     output reg         sipo_data_out,  // 1-bit data to SIPO (= observed rx_can_sync)
//     output reg         sipo_valid,     // High for 1 cycle per bit produced

//     // Status / error flags
//     output reg         bit_error,      // Bit error flag
//     output reg         arb_lost,       // Arbitration lost flag
//     output reg  [3:0]  latched_dlc,    // Latched 4-bit Data Length Code
//     output reg  [2:0]  current_state ,  // Debug/monitoring state output
//            input wire       txb_txreq,   // Ensure this is declared if used
//     input wire [3:0] dlc   // Add this line if it's missing
// );

//     // =========================================================================
//     // FSM State Encoding
//     // =========================================================================
//     localparam [2:0] STATE_IDLE        = 3'd0;
//     localparam [2:0] STATE_SOF         = 3'd1;
//     localparam [2:0] STATE_ARBITRATION = 3'd2;
//     localparam [2:0] STATE_CONTROL     = 3'd3;
//     localparam [2:0] STATE_DATA        = 3'd4;
//     localparam [2:0] STATE_CRC         = 3'd5;

//     // Field bit widths (Terminal counts)
//     localparam integer ARB_BITS  = 12; // 11-bit ID + 1-bit RTR
//     localparam integer CTRL_BITS = 6;  // IDE(1) + RB0(1) + DLC(4)

//     // =========================================================================
//     // Internal Registers
//     // =========================================================================
//     reg [2:0] state, next_state;

//     reg [6:0] bit_cnt;          // Shared per-field bit counter
//     reg [6:0] data_bit_target;  // Latched target for DATA state (dlc * 8 - 1)

//     reg       rx_prev;          // Previous sampled rx_can_sync for edge detection
//     reg       tx_bit_reg;       // Bit latched from PISO, drives tx_can

//     wire      sof_edge;         // Recessive -> Dominant edge detected at bit_tick
//     assign    sof_edge = bit_tick && (rx_prev == 1'b1) && (rx_can_sync == 1'b0);

//     // =========================================================================
//     // Process 1: State register (sequential)
//     // =========================================================================
//     always @(posedge clk or negedge rst_n) begin
//         if (!rst_n)
//             state <= STATE_IDLE;
//         else
//             state <= next_state;
//     end

//     always @(*) current_state = state;

//     // =========================================================================
//     // Process 2: Next-state combinational logic
//     // =========================================================================
//     reg [3:0] latched_dlc_next;

//     always @(*) begin
//         next_state = state; // default: hold

//         case (state)
//             STATE_IDLE: begin
//                 if (sof_edge)
//                     next_state = STATE_SOF;
//             end

//             STATE_SOF: begin
//                 if (bit_tick)
//                     next_state = STATE_ARBITRATION;
//             end

//             STATE_ARBITRATION: begin
//                 if (bit_tick && (bit_cnt == ARB_BITS - 1))
//                     next_state = STATE_CONTROL;
//             end

//             STATE_CONTROL: begin
//                 if (bit_tick && (bit_cnt == CTRL_BITS - 1)) begin
//                     if (latched_dlc_next == 4'd0)
//                         next_state = STATE_CRC;
//                     else
//                         next_state = STATE_DATA;
//                 end
//             end

//             STATE_DATA: begin
//                 if (bit_tick && (bit_cnt == data_bit_target))
//                     next_state = STATE_CRC;
//             end

//             STATE_CRC: begin
//                 next_state = STATE_CRC; // Handled externally / Part 3
//             end

//             default: next_state = STATE_IDLE;
//         endcase
//     end

//     // Combinational lookahead for DLC during control field shift-in
//     always @(*) begin
//         latched_dlc_next = latched_dlc;
//         if (state == STATE_CONTROL && bit_tick && bit_cnt >= 2)
//             latched_dlc_next = {latched_dlc[2:0], rx_can_sync};
//     end

//     // =========================================================================
//     // Process 3: Sequential datapath - counters, flags, TX/RX bit handling
//     // =========================================================================
//     always @(posedge clk or negedge rst_n) begin
//         if (!rst_n) begin
//             bit_cnt         <= 7'd0;
//             data_bit_target <= 7'd0;
//             rx_prev         <= 1'b1;
//             tx_bit_reg      <= 1'b1;
//             tx_can          <= 1'b1;
//             tx_en           <= 1'b0;
//             piso_req        <= 1'b0;
//             sipo_data_out   <= 1'b0;
//             sipo_valid      <= 1'b0;
//             latched_dlc     <= 4'd0;
//             bit_error       <= 1'b0;
//             arb_lost        <= 1'b0;
//         end
//         else begin
//             // Track previous rx state for edge detection
//             if (bit_tick)
//                 rx_prev <= rx_can_sync;

//             // Default single-cycle pulse clearing
//             sipo_valid <= 1'b0;
//             bit_error  <= 1'b0;

//             case (state)
//                 STATE_IDLE: begin
//                     bit_cnt  <= 7'd0;
//                     tx_en    <= 1'b0;
//                     tx_can   <= 1'b1;
//                     arb_lost <= 1'b0;
//                     piso_req <= 1'b0;
//                 end

//                 STATE_SOF: begin
//                     tx_en  <= 1'b1;
//                     tx_can <= 1'b0; // Dominant SOF bit
                    
//                     // Pre-assert piso_req so the first ID bit is ready 
//                     // precisely at the start of STATE_ARBITRATION
//                     piso_req <= 1'b1; 

//                     if (bit_tick) begin
//                         bit_cnt       <= 7'd0;
//                         sipo_data_out <= rx_can_sync;
//                         sipo_valid    <= 1'b1;
//                     end
//                 end

//                 STATE_ARBITRATION: begin
//                     // Keep requesting bits continuously while transmitting
//                     piso_req <= tx_en;
//                     if (piso_valid)
//                         tx_bit_reg <= piso_data_in;

//                     if (tx_en)
//                         tx_can <= tx_bit_reg;

//                     if (bit_tick) begin
//                         // Non-destructive arbitration check
//                         if (tx_en && (tx_can == 1'b1) && (rx_can_sync == 1'b0)) begin
//                             arb_lost <= 1'b1;
//                             tx_en    <= 1'b0; // Drop to receive-only mode
//                         end

//                         sipo_data_out <= rx_can_sync;
//                         sipo_valid    <= 1'b1;

//                         if (bit_cnt == ARB_BITS - 1)
//                             bit_cnt <= 7'd0;
//                         else
//                             bit_cnt <= bit_cnt + 7'd1;
//                     end
//                 end

//                 STATE_CONTROL: begin
//                     piso_req <= tx_en;
//                     if (piso_valid)
//                         tx_bit_reg <= piso_data_in;

//                     if (tx_en)
//                         tx_can <= tx_bit_reg;

//                     if (bit_tick) begin
//                         if (tx_en && (tx_can != rx_can_sync))
//                             bit_error <= 1'b1;

//                         // Latch IDE, r0, and DLC bits (MSB first)
//                         if (bit_cnt >= 2)
//                             latched_dlc <= {latched_dlc[2:0], rx_can_sync};

//                         sipo_data_out <= rx_can_sync;
//                         sipo_valid    <= 1'b1;

//                         if (bit_cnt == CTRL_BITS - 1) begin
//                             bit_cnt         <= 7'd0;
//                             data_bit_target <= (latched_dlc_next << 3) - 7'd1;
//                         end
//                         else begin
//                             bit_cnt <= bit_cnt + 7'd1;
//                         end
//                     end
//                 end

//                 STATE_DATA: begin
//                     piso_req <= tx_en;
//                     if (piso_valid)
//                         tx_bit_reg <= piso_data_in;

//                     if (tx_en)
//                         tx_can <= tx_bit_reg;

//                     if (bit_tick) begin
//                         if (tx_en && (tx_can != rx_can_sync))
//                             bit_error <= 1'b1;

//                         sipo_data_out <= rx_can_sync;
//                         sipo_valid    <= 1'b1;

//                         if (bit_cnt == data_bit_target)
//                             bit_cnt <= 7'd0;
//                         else
//                             bit_cnt <= bit_cnt + 7'd1;
//                     end
//                 end

//                 STATE_CRC: begin
//                     piso_req <= 1'b0;
//                 end

//                 default: begin
//                     bit_cnt <= 7'd0;
//                 end
//             endcase
//         end
//     end

// endmodule

