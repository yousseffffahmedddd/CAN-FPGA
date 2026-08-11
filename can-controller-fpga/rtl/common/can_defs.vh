`ifndef CAN_DEFS_VH
`define CAN_DEFS_VH

// Bus polarity values
`define CAN_RECESSIVE 1'b1
`define CAN_DOMINANT  1'b0

// Explicit FSM state encodings used by the protocol engine
`define CAN_STATE_IDLE        3'b000
`define CAN_STATE_SOF         3'b001
`define CAN_STATE_ARBITRATION 3'b010
`define CAN_STATE_CONTROL     3'b011
`define CAN_STATE_DATA        3'b100
`define CAN_STATE_CRC         3'b101

// Default timing configuration for fsm_part1
`define CAN_CLKS_PER_TQ   25
`define CAN_TQ_PER_BIT    8
`define CAN_SAMPLE_TQ_IDX 6
`define CAN_SAM_MODE      1'b1

// Bit-stuffing configuration shared by bit_stuffer
`define CAN_STUFF_RUN_LIMIT 5

`endif
