`timescale 1ns/1ps
// =============================================================================
// tb_buffers_filters.v -- self-checking testbench for rx_buffer.v / tx_buffer.v
// Style matches tb_spi_if.v: plain Verilog, `check` task, PASS/FAIL log,
// final error count. accept_filter.v is covered end-to-end via the RXB0/RXB1
// pair wired downstream of it, including rtr passthrough.
// =============================================================================

module tb_buffers_filters;

    integer errors = 0;

    task check(input cond, input [95*8:0] msg);
        begin
            if (!cond) begin
                $display("FAIL: %0s", msg);
                errors = errors + 1;
            end else
                $display("PASS: %0s", msg);
        end
    endtask

    // =========================================================================
    // rx_buffer DUT
    // =========================================================================
    reg         clk = 0;
    reg         reset;
    reg         write_enable;
    reg  [10:0] rx_id;
    reg  [63:0] rx_data;
    reg  [3:0]  rx_dlc;
    reg         rx_rtr;
    reg         cpu_read;
    wire        full;
    wire [10:0] id;
    wire [63:0] data;
    wire [3:0]  dlc;
    wire        rtr;

    rx_buffer rx_dut (
        .clk(clk), .reset(reset),
        .write_enable(write_enable), .rx_id(rx_id), .rx_data(rx_data), .rx_dlc(rx_dlc), .rx_rtr(rx_rtr),
        .cpu_read(cpu_read),
        .full(full), .id(id), .data(data), .dlc(dlc), .rtr(rtr)
    );

    // =========================================================================
    // tx_buffer DUT
    // =========================================================================
    reg         tx_write_enable;
    reg  [10:0] tx_id;
    reg  [63:0] tx_data;
    reg  [3:0]  tx_dlc;
    reg         tx_rtr;
    reg         txreq;
    reg         tx_done;
    wire [10:0] t_id;
    wire [63:0] t_data;
    wire [3:0]  t_dlc;
    wire        t_rtr;
    wire        ready;

    tx_buffer tx_dut (
        .clk(clk), .reset(reset),
        .write_enable(tx_write_enable), .tx_id(tx_id), .tx_data(tx_data), .tx_dlc(tx_dlc), .tx_rtr(tx_rtr), .txreq(txreq),
        .tx_done(tx_done),
        .id(t_id), .data(t_data), .dlc(t_dlc), .rtr(t_rtr), .ready(ready)
    );

    // =========================================================================
    // accept_filter DUT, wired into its own pair of rx_buffer instances
    // (RXB0/RXB1) so rollover and buffer-priority behavior can be tested
    // end-to-end, not just the combinational match logic in isolation.
    // =========================================================================
    reg         af_frame_valid;
    reg  [10:0] af_rx_id;
    reg  [63:0] af_rx_data;
    reg  [3:0]  af_rx_dlc;
    reg         af_rx_rtr;
    reg  [10:0] af_rxm0_mask, af_rxf0_id, af_rxf1_id;
    reg         af_rxb0_accept_all, af_bukt;
    reg  [10:0] af_rxm1_mask, af_rxf2_id, af_rxf3_id, af_rxf4_id, af_rxf5_id;
    reg         af_rxb1_accept_all;
    wire        af_accept_rxb0, af_accept_rxb1;
    wire [10:0] af_rx_id_out;
    wire [63:0] af_rx_data_out;
    wire [3:0]  af_rx_dlc_out;
    wire        af_rx_rtr_out;
    wire        af_filhit0;
    wire [2:0]  af_filhit1;

    // RXB0 / RXB1 -- real rx_buffer instances downstream of the filter, so
    // "full" feeds back into the filter's rollover logic just like on the
    // real chip. rtr rides through the filter's passthrough same as dlc.
    wire        af_rxb0_full, af_rxb1_full;
    wire [10:0] af_rxb0_id, af_rxb1_id;
    wire        af_rxb0_rtr, af_rxb1_rtr;
    reg         af_rxb0_read, af_rxb1_read;

    accept_filter af_dut (
        .frame_valid(af_frame_valid), .rx_id_in(af_rx_id), .rx_data_in(af_rx_data), .rx_dlc_in(af_rx_dlc), .rx_rtr_in(af_rx_rtr),
        .rxm0_mask(af_rxm0_mask), .rxf0_id(af_rxf0_id), .rxf1_id(af_rxf1_id),
        .rxb0_accept_all(af_rxb0_accept_all), .bukt(af_bukt),
        .rxm1_mask(af_rxm1_mask), .rxf2_id(af_rxf2_id), .rxf3_id(af_rxf3_id), .rxf4_id(af_rxf4_id), .rxf5_id(af_rxf5_id),
        .rxb1_accept_all(af_rxb1_accept_all),
        .rxb0_full(af_rxb0_full),
        .accept_rxb0(af_accept_rxb0), .accept_rxb1(af_accept_rxb1),
        .rx_id_out(af_rx_id_out), .rx_data_out(af_rx_data_out), .rx_dlc_out(af_rx_dlc_out), .rx_rtr_out(af_rx_rtr_out),
        .filhit0(af_filhit0), .filhit1(af_filhit1)
    );

    rx_buffer rxb0_dut (
        .clk(clk), .reset(reset),
        .write_enable(af_accept_rxb0), .rx_id(af_rx_id_out), .rx_data(af_rx_data_out), .rx_dlc(af_rx_dlc_out), .rx_rtr(af_rx_rtr_out),
        .cpu_read(af_rxb0_read),
        .full(af_rxb0_full), .id(af_rxb0_id), .data(), .dlc(), .rtr(af_rxb0_rtr)
    );

    rx_buffer rxb1_dut (
        .clk(clk), .reset(reset),
        .write_enable(af_accept_rxb1), .rx_id(af_rx_id_out), .rx_data(af_rx_data_out), .rx_dlc(af_rx_dlc_out), .rx_rtr(af_rx_rtr_out),
        .cpu_read(af_rxb1_read),
        .full(af_rxb1_full), .id(af_rxb1_id), .data(), .dlc(), .rtr(af_rxb1_rtr)
    );

    always #5 clk = ~clk;

    task cycle;
        begin
            @(posedge clk);
            #1; // settle
        end
    endtask

    initial begin
        // ---------------------------------------------------------------
        // Common reset
        // ---------------------------------------------------------------
        reset = 1;
        write_enable = 0; rx_id = 0; rx_data = 0; rx_dlc = 0; rx_rtr = 0; cpu_read = 0;
        tx_write_enable = 0; tx_id = 0; tx_data = 0; tx_dlc = 0; tx_rtr = 0; txreq = 0; tx_done = 0;
        cycle; cycle;

        check(full == 0, "rx_buffer: full=0 after reset");
        check(id   == 0, "rx_buffer: id=0 after reset");
        check(data == 0, "rx_buffer: data=0 after reset");
        check(dlc  == 0, "rx_buffer: dlc=0 after reset");
        check(rtr  == 0, "rx_buffer: rtr=0 after reset");

        check(ready  == 0, "tx_buffer: ready=0 after reset");
        check(t_id   == 0, "tx_buffer: id=0 after reset");
        check(t_data == 0, "tx_buffer: data=0 after reset");
        check(t_dlc  == 0, "tx_buffer: dlc=0 after reset");
        check(t_rtr  == 0, "tx_buffer: rtr=0 after reset");

        reset = 0;
        cycle;

        // =================================================================
        // rx_buffer tests
        // =================================================================

        // --- basic write when empty ---
        rx_id = 11'h123; rx_data = 64'hDEAD_BEEF_0000_0001; rx_dlc = 4'h5; rx_rtr = 1'b0;
        write_enable = 1;
        cycle;
        write_enable = 0;
        check(full == 1,                 "rx_buffer: full=1 after accepted write");
        check(id   == 11'h123,           "rx_buffer: id captured correctly");
        check(data == 64'hDEAD_BEEF_0000_0001, "rx_buffer: data captured correctly");
        check(dlc  == 4'h5,              "rx_buffer: dlc captured correctly");
        check(rtr  == 1'b0,              "rx_buffer: rtr=0 captured correctly (data frame)");

        // --- new write while full is ignored (message not overwritten) ---
        rx_id = 11'h456; rx_data = 64'h1111_2222_3333_4444; rx_dlc = 4'h8; rx_rtr = 1'b1;
        write_enable = 1;
        cycle;
        write_enable = 0;
        check(full == 1,       "rx_buffer: still full, second write not accepted");
        check(id   == 11'h123, "rx_buffer: id unchanged while full (no overwrite)");
        check(data == 64'hDEAD_BEEF_0000_0001, "rx_buffer: data unchanged while full");
        check(rtr  == 1'b0,    "rx_buffer: rtr unchanged while full (no overwrite)");

        // --- cpu_read clears full, old data still visible for that cycle ---
        cpu_read = 1;
        cycle;
        cpu_read = 0;
        check(full == 0, "rx_buffer: full=0 after cpu_read");

        // --- remote frame (rtr=1) captured correctly once buffer is free ---
        rx_id = 11'h456; rx_data = 64'h0; rx_dlc = 4'h8; rx_rtr = 1'b1;
        write_enable = 1;
        cycle;
        write_enable = 0;
        check(full == 1,       "rx_buffer: accepts remote frame after buffer emptied");
        check(id   == 11'h456, "rx_buffer: remote frame id captured");
        check(rtr  == 1'b1,    "rx_buffer: rtr=1 captured correctly (remote frame)");
        cpu_read = 1; cycle; cpu_read = 0; cycle;

        // --- simultaneous write_enable + cpu_read while full: write loses ---
        // load a message first
        rx_id = 11'h001; rx_data = 64'hAAAA; rx_dlc = 4'h1; rx_rtr = 1'b0;
        write_enable = 1; cycle; write_enable = 0;
        check(full == 1, "rx_buffer: setup message loaded before contention test");

        rx_id = 11'h7FF; rx_data = 64'hFFFF; rx_dlc = 4'hF; rx_rtr = 1'b1;
        write_enable = 1; cpu_read = 1;
        cycle;
        write_enable = 0; cpu_read = 0;
        check(full == 0,       "rx_buffer: contention -- cpu_read wins, full clears");
        check(id   == 11'h001, "rx_buffer: contention -- new id NOT loaded same cycle (known limitation: re-pulse write_enable next cycle)");

        // --- buffer accepts again once empty ---
        rx_id = 11'h7FF; rx_data = 64'hCAFEBABE; rx_dlc = 4'h4; rx_rtr = 1'b0;
        write_enable = 1;
        cycle;
        write_enable = 0;
        check(full == 1,          "rx_buffer: accepts new message after buffer emptied");
        check(id   == 11'h7FF,    "rx_buffer: new id captured after re-accept");
        check(rtr  == 1'b0,       "rx_buffer: rtr=0 captured after re-accept");

        // --- reset clears mid-operation ---
        reset = 1;
        cycle;
        check(full == 0 && id == 0 && data == 0 && dlc == 0 && rtr == 0, "rx_buffer: reset clears buffer mid-operation (incl. rtr)");
        reset = 0;
        cycle;

        // =================================================================
        // tx_buffer tests
        // =================================================================

        // --- write loads id/data/dlc/rtr, ready stays low without txreq ---
        tx_id = 11'h200; tx_data = 64'h0102_0304_0506_0708; tx_dlc = 4'h8; tx_rtr = 1'b0;
        tx_write_enable = 1;
        cycle;
        tx_write_enable = 0;
        check(t_id   == 11'h200, "tx_buffer: id captured on write");
        check(t_data == 64'h0102_0304_0506_0708, "tx_buffer: data captured on write");
        check(t_dlc  == 4'h8,    "tx_buffer: dlc captured on write");
        check(t_rtr  == 1'b0,    "tx_buffer: rtr=0 captured on write (data frame)");
        check(ready  == 0,       "tx_buffer: ready stays 0 without txreq");

        // --- txreq sets ready ---
        txreq = 1;
        cycle;
        txreq = 0;
        check(ready == 1, "tx_buffer: ready=1 after txreq");

        // --- tx_done clears ready ---
        tx_done = 1;
        cycle;
        tx_done = 0;
        check(ready == 0, "tx_buffer: ready=0 after tx_done");

        // --- remote frame request (rtr=1): dlc conveys requested length, no data ---
        tx_id = 11'h201; tx_data = 64'h0; tx_dlc = 4'h3; tx_rtr = 1'b1;
        tx_write_enable = 1;
        cycle;
        tx_write_enable = 0;
        check(t_id  == 11'h201, "tx_buffer: remote frame id captured");
        check(t_dlc == 4'h3,    "tx_buffer: remote frame dlc (requested length) captured");
        check(t_rtr == 1'b1,    "tx_buffer: rtr=1 captured on write (remote frame)");

        // --- txreq and tx_done asserted together: txreq wins (per current RTL priority) ---
        txreq = 1; tx_done = 1;
        cycle;
        txreq = 0; tx_done = 0;
        check(ready == 1, "tx_buffer: txreq takes priority over simultaneous tx_done (documents current behavior)");
        // clear it back down for the next check
        tx_done = 1; cycle; tx_done = 0;
        check(ready == 0, "tx_buffer: ready clears once txreq is no longer asserted");

        // --- KNOWN GAP: write_enable while ready=1 overwrites an in-flight message ---
        // This currently succeeds in the RTL (no !ready guard). Kept as a regression
        // test: if a busy-guard is added later, this check should be updated to
        // expect id/data/dlc/rtr to stay at the in-flight message instead.
        tx_id = 11'h300; tx_data = 64'hAAAA; tx_dlc = 4'h2; tx_rtr = 1'b0;
        tx_write_enable = 1; txreq = 1;
        cycle;
        tx_write_enable = 0; txreq = 0;
        check(ready == 1, "tx_buffer: in-flight message marked ready");

        tx_id = 11'h301; tx_data = 64'hBBBB; tx_dlc = 4'h3; tx_rtr = 1'b1;
        tx_write_enable = 1; // no txreq this time, tx_done not asserted either
        cycle;
        tx_write_enable = 0;
        check(t_id  == 11'h301, "tx_buffer: GAP -- write_enable overwrote id while ready was still 1 (no busy guard yet)");
        check(t_rtr == 1'b1,    "tx_buffer: GAP -- write_enable also overwrote rtr while ready was still 1");
        check(ready == 1,      "tx_buffer: ready remains 1 even though contents changed underneath it");

        // --- reset clears mid-operation ---
        reset = 1;
        cycle;
        check(ready == 0 && t_id == 0 && t_data == 0 && t_dlc == 0 && t_rtr == 0, "tx_buffer: reset clears buffer mid-operation (incl. rtr)");
        reset = 0;
        cycle;

        // =================================================================
        // accept_filter tests
        // =================================================================
        // Reset the RXB0/RXB1 pair via the shared reset before starting.
        reset = 1; cycle; reset = 0; cycle;

        af_frame_valid = 0; af_rx_id = 0; af_rx_data = 0; af_rx_dlc = 0; af_rx_rtr = 0;
        af_rxm0_mask = 0; af_rxf0_id = 0; af_rxf1_id = 0; af_rxb0_accept_all = 0; af_bukt = 0;
        af_rxm1_mask = 0; af_rxf2_id = 0; af_rxf3_id = 0; af_rxf4_id = 0; af_rxf5_id = 0; af_rxb1_accept_all = 0;
        af_rxb0_read = 0; af_rxb1_read = 0;
        #1;

        // NOTE ON TIMING: accept_rxb0/accept_rxb1/filhit* are purely
        // combinational, driven partly by rxb0_full which itself only
        // updates ON the clock edge that just consumed the acceptance
        // decision. So each check below samples the combinational outputs
        // with #2 (mid-cycle, comb settled, before the next edge) and only
        // THEN calls cycle to actually latch the message into the buffer
        // and let "full" update for the next step.

        // --- RXB0 accept-all mode: any ID routes to RXB0, not RXB1 ---
        af_rxb0_accept_all = 1;
        af_frame_valid = 1; af_rx_id = 11'h555; af_rx_data = 64'hDEAD; af_rx_dlc = 4'h3; af_rx_rtr = 1'b0;
        #2;
        check(af_accept_rxb0 == 1, "accept_filter: RXB0 accept-all takes arbitrary ID");
        check(af_accept_rxb1 == 0, "accept_filter: RXB0 accept-all -- message NOT also sent to RXB1");
        check(af_rx_rtr_out == 1'b0, "accept_filter: rtr passthrough=0 for data frame");
        cycle;
        check(af_rxb0_full == 1,   "accept_filter: RXB0 buffer actually latched the message");
        check(af_rxb0_rtr == 1'b0, "accept_filter: RXB0 buffer latched rtr=0 correctly");
        af_frame_valid = 0; af_rxb0_read = 1; cycle; af_rxb0_read = 0; cycle;
        af_rxb0_accept_all = 0;

        // --- RXB0 exact-match filter (RXF0), RXM0 = all-ones ---
        af_rxm0_mask = 11'h7FF; af_rxf0_id = 11'h123; af_rxf1_id = 11'h456;
        af_frame_valid = 1; af_rx_id = 11'h123; af_rx_rtr = 1'b0;
        #2;
        check(af_accept_rxb0 == 1, "accept_filter: RXB0 exact match on RXF0 accepts");
        check(af_filhit0 == 0,     "accept_filter: FILHIT0 reports RXF0 (0) matched");
        cycle;
        af_frame_valid = 0; af_rxb0_read = 1; cycle; af_rxb0_read = 0; cycle;

        // --- remote frame (rtr=1) matching RXF1: filter logic ignores rtr,
        //     but the accepted buffer must still carry rtr=1 through ---
        af_frame_valid = 1; af_rx_id = 11'h456; af_rx_rtr = 1'b1;
        #2;
        check(af_accept_rxb0 == 1, "accept_filter: RXB0 exact match on RXF1 accepts (remote frame)");
        check(af_filhit0 == 1,     "accept_filter: FILHIT0 reports RXF1 (1) matched (rtr does not affect filter hit)");
        check(af_rx_rtr_out == 1'b1, "accept_filter: rtr passthrough=1 for remote frame");
        cycle;
        check(af_rxb0_rtr == 1'b1, "accept_filter: RXB0 buffer latched rtr=1 (remote frame) correctly");
        af_frame_valid = 0; af_rxb0_read = 1; cycle; af_rxb0_read = 0; cycle;
        af_rx_rtr = 1'b0;

        // Give RXB1 a restrictive mask before this check -- otherwise
        // rxm1_mask is still its init value of 0, which means "accept
        // all" by design (see accept_filter.v header), and this ID would
        // legitimately fall through to RXB1.
        af_rxm1_mask = 11'h7FF; af_rxf2_id = 11'h700; af_rxf3_id = 11'h701; af_rxf4_id = 11'h702; af_rxf5_id = 11'h703;

        af_frame_valid = 1; af_rx_id = 11'h789;
        #2;
        check(af_accept_rxb0 == 0, "accept_filter: RXB0 rejects ID matching neither RXF0 nor RXF1");
        check(af_accept_rxb1 == 0, "accept_filter: rejected ID also doesn't match any RXB1 filter -- dropped entirely");
        cycle;
        af_frame_valid = 0; cycle;

        // --- RXB1 filter match (RXF3), independent of RXB0's filters ---
        af_rxf2_id = 11'h001; af_rxf3_id = 11'h002; af_rxf4_id = 11'h003; af_rxf5_id = 11'h004;
        af_frame_valid = 1; af_rx_id = 11'h002; // doesn't match RXF0/RXF1, does match RXF3
        #2;
        check(af_accept_rxb0 == 0, "accept_filter: RXB1-only match does not go to RXB0");
        check(af_accept_rxb1 == 1, "accept_filter: RXB1 exact match on RXF3 accepts");
        check(af_filhit1 == 1,     "accept_filter: FILHIT1 reports RXF3 (index 1) matched");
        cycle;
        af_frame_valid = 0; af_rxb1_read = 1; cycle; af_rxb1_read = 0; cycle;

        // --- RXB0 priority: an ID matching BOTH RXB0 and RXB1 filters goes to RXB0 only ---
        af_rxf0_id = 11'h123; // re-affirm (unchanged from above)
        af_rxf2_id = 11'h123; // deliberately overlap with RXF0
        af_frame_valid = 1; af_rx_id = 11'h123;
        #2;
        check(af_accept_rxb0 == 1, "accept_filter: overlapping match -- RXB0 wins (higher priority)");
        check(af_accept_rxb1 == 0, "accept_filter: overlapping match -- RXB1 does NOT also receive it");
        cycle;
        af_frame_valid = 0; af_rxb0_read = 1; cycle; af_rxb0_read = 0; cycle;
        af_rxf2_id = 11'h001; // restore

        // --- Rollover: RXB0 full + BUKT=1 + matching message -> spills into RXB1 ---
        af_bukt = 1;
        af_frame_valid = 1; af_rx_id = 11'h123; // matches RXF0
        cycle; // first message fills RXB0
        check(af_rxb0_full == 1, "accept_filter: rollover setup -- RXB0 now full");

        af_rx_id = 11'h456; af_rx_rtr = 1'b1; // matches RXF1, RXB0 still full, remote frame this time
        #2;
        check(af_accept_rxb0 == 0, "accept_filter: rollover -- RXB0 itself does not accept (already full)");
        check(af_accept_rxb1 == 1, "accept_filter: rollover -- message redirected into RXB1 instead");
        check(af_filhit1 == 5,     "accept_filter: rollover FILHIT1 encodes RXF1-via-rollover (5)");
        cycle;
        check(af_rxb1_rtr == 1'b1, "accept_filter: rolled-over remote frame still carries rtr=1 into RXB1");
        af_frame_valid = 0; af_rx_rtr = 1'b0;
        af_rxb0_read = 1; af_rxb1_read = 1; cycle; af_rxb0_read = 0; af_rxb1_read = 0; cycle;
        af_bukt = 0;

        // --- Rollover disabled (BUKT=0): message lost, neither buffer accepts ---
        af_frame_valid = 1; af_rx_id = 11'h123;
        cycle; // fill RXB0 again
        af_rx_id = 11'h456;
        #2;
        check(af_accept_rxb0 == 0, "accept_filter: BUKT=0 -- RXB0 still full, doesn't accept");
        check(af_accept_rxb1 == 0, "accept_filter: BUKT=0 -- no rollover, message dropped as expected");
        cycle;
        af_frame_valid = 0;
        af_rxb0_read = 1; cycle; af_rxb0_read = 0; cycle;

        // =================================================================
        if (errors == 0)
            $display(">>> ALL TESTS PASSED <<<");
        else
            $display(">>> %0d TEST(S) FAILED <<<", errors);

        $finish;
    end

endmodule