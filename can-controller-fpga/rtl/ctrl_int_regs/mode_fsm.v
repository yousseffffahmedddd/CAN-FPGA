// =============================================================================
// mode_fsm.v -- Operating-mode FSM (Module 5, SOW 8.4.8).
//
// Owns the CANCTRL.REQOP -> CANSTAT.OPMOD relationship and the write gate that
// protects the configuration registers.
//
// REDUCED SCOPE: two modes only, Configuration and Normal. The MCP2515 also
// defines Sleep (001), Loopback (010) and Listen-Only (011), but all three
// need cooperation from the protocol engine, whose own SOW (Section 3.0)
// excludes them. Rather than accept a REQOP value and then not honour it --
// which would leave CANSTAT.OPMOD lying about the state of the device -- those
// three encodings are decoded and REJECTED: the request is dropped and OPMOD
// keeps reporting the mode actually in effect.
//
// The real device does not switch modes mid-frame either (datasheet Section
// 10.0: "the mode will not actually change until all pending message
// transmissions are complete"), so a request is held until bus_idle.
// =============================================================================

module mode_fsm (
    input  wire       clk,
    input  wire       rst_n,        // active-low async reset
    input  wire       sync_reset,   // SPI RESET instruction (datasheet Sec 12.2)

    // Requested mode: the REQOP field of CANCTRL, plus a one-cycle pulse on
    // the clock edge the host actually writes CANCTRL. The level alone is not
    // enough -- rewriting the same value must not re-trigger a transition.
    input  wire [2:0] reqop,
    input  wire       reqop_we,

    // From the protocol engine. Held high whenever no frame is in progress.
    input  wire       bus_idle,

    output reg  [2:0] opmod,        // -> CANSTAT.OPMOD: the mode IN EFFECT
    output wire       config_mode,  // write gate for RXF*/RXM*/CNF*
    output wire       normal_mode
);

    // Datasheet Register 10-1 (CANCTRL) REQOP encoding.
    localparam [2:0] MODE_NORMAL = 3'b000,
                     MODE_SLEEP  = 3'b001,   // out of scope -- rejected
                     MODE_LOOPBK = 3'b010,   // out of scope -- rejected
                     MODE_LISTEN = 3'b011,   // out of scope -- rejected
                     MODE_CONFIG = 3'b100;

    wire req_legal = (reqop == MODE_NORMAL) || (reqop == MODE_CONFIG);

    reg       pending;      // a legal request is waiting for the bus to go idle
    reg [2:0] req_latched;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            opmod       <= MODE_CONFIG;   // datasheet: device powers up in Configuration
            pending     <= 1'b0;
            req_latched <= MODE_CONFIG;
        end else if (sync_reset) begin
            opmod       <= MODE_CONFIG;
            pending     <= 1'b0;
            req_latched <= MODE_CONFIG;
        end else begin
            // A fresh host write takes priority over completing an older
            // request. This matters for the same-cycle case: if the host
            // writes REQOP on the exact edge a held request would have
            // completed, the NEW value is what the host asked for last, so it
            // is the one that must survive. Re-evaluating `pending` against
            // the current opmod also makes "write back the mode we are
            // already in" cancel a pending change, rather than queueing a
            // redundant transition.
            if (reqop_we && req_legal) begin
                req_latched <= reqop;
                pending     <= (reqop != opmod);
            end else if (pending && bus_idle) begin
                opmod   <= req_latched;
                pending <= 1'b0;
            end
            // reqop_we with an out-of-scope encoding falls through both
            // branches: no latch, no pending, opmod untouched (SOW 8.4.8.53).
        end
    end

    assign config_mode = (opmod == MODE_CONFIG);
    assign normal_mode = (opmod == MODE_NORMAL);

endmodule
