`timescale 1ns/1ps
// =============================================================================
// tb_buffers_filters.v -- self-checking testbench for rx_buffer.v / tx_buffer.v
// Style matches tb_spi_if.v: plain Verilog, `check` task, PASS/FAIL log,
// final error count. accept_filter.v is not yet written, so it is not
// covered here -- add its tests to this file once it exists.
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
    reg         cpu_read;
    wire        full;
    wire [10:0] id;
    wire [63:0] data;
    wire [3:0]  dlc;

    rx_buffer rx_dut (
        .clk(clk), .reset(reset),
        .write_enable(write_enable), .rx_id(rx_id), .rx_data(rx_data), .rx_dlc(rx_dlc),
        .cpu_read(cpu_read),
        .full(full), .id(id), .data(data), .dlc(dlc)
    );

    // =========================================================================
    // tx_buffer DUT
    // =========================================================================
    reg         tx_write_enable;
    reg  [10:0] tx_id;
    reg  [63:0] tx_data;
    reg  [3:0]  tx_dlc;
    reg         txreq;
    reg         tx_done;
    wire [10:0] t_id;
    wire [63:0] t_data;
    wire [3:0]  t_dlc;
    wire        ready;

    tx_buffer tx_dut (
        .clk(clk), .reset(reset),
        .write_enable(tx_write_enable), .tx_id(tx_id), .tx_data(tx_data), .tx_dlc(tx_dlc), .txreq(txreq),
        .tx_done(tx_done),
        .id(t_id), .data(t_data), .dlc(t_dlc), .ready(ready)
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
        write_enable = 0; rx_id = 0; rx_data = 0; rx_dlc = 0; cpu_read = 0;
        tx_write_enable = 0; tx_id = 0; tx_data = 0; tx_dlc = 0; txreq = 0; tx_done = 0;
        cycle; cycle;

        check(full == 0, "rx_buffer: full=0 after reset");
        check(id   == 0, "rx_buffer: id=0 after reset");
        check(data == 0, "rx_buffer: data=0 after reset");
        check(dlc  == 0, "rx_buffer: dlc=0 after reset");

        check(ready  == 0, "tx_buffer: ready=0 after reset");
        check(t_id   == 0, "tx_buffer: id=0 after reset");
        check(t_data == 0, "tx_buffer: data=0 after reset");
        check(t_dlc  == 0, "tx_buffer: dlc=0 after reset");

        reset = 0;
        cycle;

        // =================================================================
        // rx_buffer tests
        // =================================================================

        // --- basic write when empty ---
        rx_id = 11'h123; rx_data = 64'hDEAD_BEEF_0000_0001; rx_dlc = 4'h5;
        write_enable = 1;
        cycle;
        write_enable = 0;
        check(full == 1,                 "rx_buffer: full=1 after accepted write");
        check(id   == 11'h123,           "rx_buffer: id captured correctly");
        check(data == 64'hDEAD_BEEF_0000_0001, "rx_buffer: data captured correctly");
        check(dlc  == 4'h5,              "rx_buffer: dlc captured correctly");

        // --- new write while full is ignored (message not overwritten) ---
        rx_id = 11'h456; rx_data = 64'h1111_2222_3333_4444; rx_dlc = 4'h8;
        write_enable = 1;
        cycle;
        write_enable = 0;
        check(full == 1,       "rx_buffer: still full, second write not accepted");
        check(id   == 11'h123, "rx_buffer: id unchanged while full (no overwrite)");
        check(data == 64'hDEAD_BEEF_0000_0001, "rx_buffer: data unchanged while full");

        // --- cpu_read clears full, old data still visible for that cycle ---
        cpu_read = 1;
        cycle;
        cpu_read = 0;
        check(full == 0, "rx_buffer: full=0 after cpu_read");

        // --- simultaneous write_enable + cpu_read while full: write loses ---
        // load a message first
        rx_id = 11'h001; rx_data = 64'hAAAA; rx_dlc = 4'h1;
        write_enable = 1; cycle; write_enable = 0;
        check(full == 1, "rx_buffer: setup message loaded before contention test");

        rx_id = 11'h7FF; rx_data = 64'hFFFF; rx_dlc = 4'hF;
        write_enable = 1; cpu_read = 1;
        cycle;
        write_enable = 0; cpu_read = 0;
        check(full == 0,       "rx_buffer: contention -- cpu_read wins, full clears");
        check(id   == 11'h001, "rx_buffer: contention -- new id NOT loaded same cycle (known limitation: re-pulse write_enable next cycle)");

        // --- buffer accepts again once empty ---
        rx_id = 11'h7FF; rx_data = 64'hCAFEBABE; rx_dlc = 4'h4;
        write_enable = 1;
        cycle;
        write_enable = 0;
        check(full == 1,          "rx_buffer: accepts new message after buffer emptied");
        check(id   == 11'h7FF,    "rx_buffer: new id captured after re-accept");

        // --- reset clears mid-operation ---
        reset = 1;
        cycle;
        check(full == 0 && id == 0 && data == 0 && dlc == 0, "rx_buffer: reset clears buffer mid-operation");
        reset = 0;
        cycle;

        // =================================================================
        // tx_buffer tests
        // =================================================================

        // --- write loads id/data/dlc, ready stays low without txreq ---
        tx_id = 11'h200; tx_data = 64'h0102_0304_0506_0708; tx_dlc = 4'h8;
        tx_write_enable = 1;
        cycle;
        tx_write_enable = 0;
        check(t_id   == 11'h200, "tx_buffer: id captured on write");
        check(t_data == 64'h0102_0304_0506_0708, "tx_buffer: data captured on write");
        check(t_dlc  == 4'h8,    "tx_buffer: dlc captured on write");
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
        // expect id/data/dlc to stay at the in-flight message instead.
        tx_id = 11'h300; tx_data = 64'hAAAA; tx_dlc = 4'h2;
        tx_write_enable = 1; txreq = 1;
        cycle;
        tx_write_enable = 0; txreq = 0;
        check(ready == 1, "tx_buffer: in-flight message marked ready");

        tx_id = 11'h301; tx_data = 64'hBBBB; tx_dlc = 4'h3;
        tx_write_enable = 1; // no txreq this time, tx_done not asserted either
        cycle;
        tx_write_enable = 0;
        check(t_id == 11'h301, "tx_buffer: GAP -- write_enable overwrote id while ready was still 1 (no busy guard yet)");
        check(ready == 1,      "tx_buffer: ready remains 1 even though contents changed underneath it");

        // --- reset clears mid-operation ---
        reset = 1;
        cycle;
        check(ready == 0 && t_id == 0 && t_data == 0 && t_dlc == 0, "tx_buffer: reset clears buffer mid-operation");
        reset = 0;
        cycle;

        // =================================================================
        if (errors == 0)
            $display(">>> ALL TESTS PASSED <<<");
        else
            $display(">>> %0d TEST(S) FAILED <<<", errors);

        $finish;
    end

endmodule