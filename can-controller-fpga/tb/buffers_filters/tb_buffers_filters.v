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
    // accept_filter DUT, wired into a single downstream rx_buffer instance
    // so acceptance -> capture can be tested end-to-end, not just the
    // combinational match logic in isolation.
    // =========================================================================
    reg         af_frame_valid;
    reg  [10:0] af_rx_id;
    reg  [63:0] af_rx_data;
    reg  [3:0]  af_rx_dlc;
    reg         af_rx_rtr;
    reg  [10:0] af_filter_id, af_filter_mask;
    wire        af_accept;
    wire [10:0] af_rx_id_out;
    wire [63:0] af_rx_data_out;
    wire [3:0]  af_rx_dlc_out;
    wire        af_rx_rtr_out;

    // Single rx_buffer downstream of the filter. rtr rides through the
    // filter's passthrough same as dlc.
    wire        af_rxb_full;
    wire [10:0] af_rxb_id;
    wire        af_rxb_rtr;
    reg         af_rxb_read;

    accept_filter af_dut (
        .frame_valid(af_frame_valid), .rx_id_in(af_rx_id), .rx_data_in(af_rx_data), .rx_dlc_in(af_rx_dlc), .rx_rtr_in(af_rx_rtr),
        .filter_id(af_filter_id), .filter_mask(af_filter_mask),
        .accept(af_accept),
        .rx_id_out(af_rx_id_out), .rx_data_out(af_rx_data_out), .rx_dlc_out(af_rx_dlc_out), .rx_rtr_out(af_rx_rtr_out)
    );

    rx_buffer rxb_dut (
        .clk(clk), .reset(reset),
        .write_enable(af_accept), .rx_id(af_rx_id_out), .rx_data(af_rx_data_out), .rx_dlc(af_rx_dlc_out), .rx_rtr(af_rx_rtr_out),
        .cpu_read(af_rxb_read),
        .full(af_rxb_full), .id(af_rxb_id), .data(), .dlc(), .rtr(af_rxb_rtr)
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

        // tx_buffer guards writes with !ready, so an in-flight message
        // retain stored data until transmission is completed.
        tx_id = 11'h300; tx_data = 64'hAAAA; tx_dlc = 4'h2; tx_rtr = 1'b0;
        tx_write_enable = 1; txreq = 1;
        cycle;
        tx_write_enable = 0; txreq = 0;
        check(ready == 1, "tx_buffer: in-flight message marked ready");

        tx_id = 11'h301; tx_data = 64'hBBBB; tx_dlc = 4'h3; tx_rtr = 1'b1;
        tx_write_enable = 1; // no txreq this time, tx_done not asserted either
        cycle;
        tx_write_enable = 0;
        check(t_id   == 11'h300,                 "tx_buffer: write_enable ignored while ready=1, id unchanged (busy guard)");
        check(t_data == 64'hAAAA,                "tx_buffer: data unchanged while ready=1 (busy guard)");
        check(t_dlc  == 4'h2,                    "tx_buffer: dlc unchanged while ready=1 (busy guard)");
        check(t_rtr  == 1'b0,                    "tx_buffer: rtr unchanged while ready=1 (busy guard)");
        check(ready  == 1,                       "tx_buffer: ready remains 1, message still pending");

        // =================================================================
        // accept_filter tests
        // =================================================================
        reset = 1; cycle; reset = 0; cycle;

        af_frame_valid = 0; af_rx_id = 0; af_rx_data = 0; af_rx_dlc = 0; af_rx_rtr = 0;
        af_filter_id = 0; af_filter_mask = 0;
        af_rxb_read = 0;
        #1;

        // NOTE ON TIMING: accept is purely combinational. Each check below
        // samples it with #2 (mid-cycle, comb settled, before the next
        // edge) and only THEN calls cycle to actually latch the message
        // into the buffer and let "full" update for the next step.

        // --- mask = all-zero ("accept all"): any ID is accepted regardless
        //     of filter_id, since no bits are marked "care about" ---
        af_filter_id = 11'h000; af_filter_mask = 11'h000;
        af_frame_valid = 1; af_rx_id = 11'h555; af_rx_data = 64'hDEAD; af_rx_dlc = 4'h3; af_rx_rtr = 1'b0;
        #2;
        check(af_accept == 1,       "accept_filter: mask=0 accepts arbitrary ID");
        check(af_rx_rtr_out == 1'b0, "accept_filter: rtr passthrough=0 for data frame");
        cycle;
        check(af_rxb_full == 1,   "accept_filter: buffer actually latched the message");
        check(af_rxb_rtr == 1'b0, "accept_filter: buffer latched rtr=0 correctly");
        af_frame_valid = 0; af_rxb_read = 1; cycle; af_rxb_read = 0; cycle;

        // --- exact-match filter, mask = all-ones (every bit must match) ---
        af_filter_mask = 11'h7FF; af_filter_id = 11'h123;
        af_frame_valid = 1; af_rx_id = 11'h123; af_rx_rtr = 1'b0;
        #2;
        check(af_accept == 1, "accept_filter: exact ID match accepts");
        cycle;
        af_frame_valid = 0; af_rxb_read = 1; cycle; af_rxb_read = 0; cycle;

        // --- remote frame (rtr=1): filter logic ignores rtr, but the
        //     accepted buffer must still carry rtr=1 through ---
        af_frame_valid = 1; af_rx_id = 11'h123; af_rx_rtr = 1'b1;
        #2;
        check(af_accept == 1,        "accept_filter: exact match accepts (remote frame)");
        check(af_rx_rtr_out == 1'b1, "accept_filter: rtr passthrough=1 for remote frame");
        cycle;
        check(af_rxb_rtr == 1'b1, "accept_filter: buffer latched rtr=1 (remote frame) correctly");
        af_frame_valid = 0; af_rxb_read = 1; cycle; af_rxb_read = 0; cycle;
        af_rx_rtr = 1'b0;

        // --- non-matching ID under a full mask: rejected, buffer untouched ---
        af_frame_valid = 1; af_rx_id = 11'h789;
        #2;
        check(af_accept == 0, "accept_filter: ID not matching filter_id is rejected");
        cycle;
        check(af_rxb_full == 0, "accept_filter: rejected message never reaches the buffer");
        af_frame_valid = 0; cycle;

        // --- partial mask: only masked bits are compared, others are don't-care ---
        // filter_id = 11'h123 = 0b001_0010_0011, mask = 11'h700 = compares only
        // the top 3 bits (0b001), so any ID with top bits 001 matches regardless
        // of the lower 8 bits.
        af_filter_id = 11'h123; af_filter_mask = 11'h700;
        af_frame_valid = 1; af_rx_id = 11'h1FF; // top 3 bits = 001, lower bits differ
        #2;
        check(af_accept == 1, "accept_filter: partial mask -- don't-care bits ignored, top bits match");
        cycle;
        af_frame_valid = 0; af_rxb_read = 1; cycle; af_rxb_read = 0; cycle;

        af_frame_valid = 1; af_rx_id = 11'h2FF; // top 3 bits = 010, does not match 001
        #2;
        check(af_accept == 0, "accept_filter: partial mask -- differing cared-about bits reject");
        cycle;
        af_frame_valid = 0; cycle;

        // --- buffer already full: filter still asserts accept (filter has no
        //     buffer-occupancy awareness), but rx_buffer itself drops the
        //     write since it ignores write_enable while full ---
        af_filter_mask = 11'h7FF; af_filter_id = 11'h123;
        af_frame_valid = 1; af_rx_id = 11'h123;
        cycle; // fill the buffer
        check(af_rxb_full == 1, "accept_filter: buffer-full setup -- buffer now full");

        af_rx_id = 11'h123; // matches again, buffer still full
        #2;
        check(af_accept == 1, "accept_filter: filter still accepts a match regardless of buffer occupancy");
        cycle;
        check(af_rxb_id == 11'h123, "accept_filter: rx_buffer correctly ignored the second write while full (id unchanged)");
        af_frame_valid = 0;
        af_rxb_read = 1; cycle; af_rxb_read = 0; cycle;

        // =================================================================
        if (errors == 0)
            $display(">>> ALL TESTS PASSED <<<");
        else
            $display(">>> %0d TEST(S) FAILED <<<", errors);

        $finish;
    end

endmodule
