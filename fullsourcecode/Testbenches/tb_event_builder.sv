// tb_event_builder.sv
//
// Unit testbench for event_builder.
// Drives pos and mag streams (via separate procedures running in parallel)
// at programmable rates and tag patterns, and verifies:
//   - Matched events appear on out_valid with correct tag/x/y/mag
//   - drop_pos asserts when pos_tag is older than mag head
//   - drop_mag asserts when mag_tag is older than pos head
//   - mag_force_pop asserts when a new mag arrives while mag FIFO is full
//   - out_valid, drop_pos, drop_mag, mag_force_pop are 1-cycle pulses
//   - out_valid is mutually exclusive with drop_pos / drop_mag in the same cycle
//
// Coverage:
//   - In-order matched stream
//   - Pos stream leads mag stream (mag arrives later)
//   - Mag stream leads pos stream (pos arrives later)
//   - Stale pos entries (pos has tags mag will never have)
//   - Stale mag entries (mag has tags pos will never have)
//   - Burst that overfills the mag FIFO -> mag_force_pop should fire
//   - Same-cycle pos and mag pushes
//
// Style follows tb_full_dsp_pipeline.sv: drive/capture/analysis split,
// explicit per-stream stim arrays with deterministic order.

`timescale 1ns / 1ps

module tb_event_builder;

    // -----------------------------------------------------------------
    // Parameters (must match dsp_pipeline.v defaults)
    // -----------------------------------------------------------------
    localparam integer TAG_WIDTH  = 64;
    localparam integer POS_WIDTH  = 32;
    localparam integer MAG_WIDTH  = 16;
    localparam integer DEPTH_LOG2 = 2;
    localparam integer DEPTH      = (1 << DEPTH_LOG2);  // 4

    localparam integer N_STREAM_MAX = 64;

    // -----------------------------------------------------------------
    // Clock / reset
    // -----------------------------------------------------------------
    logic clk;
    logic rst;
    initial clk = 0;
    always #5 clk = ~clk;

    // -----------------------------------------------------------------
    // DUT signals
    // -----------------------------------------------------------------
    logic                       pos_valid;
    logic [TAG_WIDTH-1:0]       pos_tag;
    logic signed [POS_WIDTH-1:0] pos_x, pos_y;

    logic                       mag_valid;
    logic [TAG_WIDTH-1:0]       mag_tag;
    logic signed [MAG_WIDTH-1:0] mag_val;

    wire                        out_valid;
    wire [TAG_WIDTH-1:0]        out_tag;
    wire signed [POS_WIDTH-1:0] out_x, out_y;
    wire signed [MAG_WIDTH-1:0] out_mag;

    wire drop_pos, drop_mag, mag_force_pop;

    event_builder #(
        .TAG_WIDTH (TAG_WIDTH),
        .POS_WIDTH (POS_WIDTH),
        .MAG_WIDTH (MAG_WIDTH),
        .DEPTH_LOG2(DEPTH_LOG2)
    ) dut (
        .clk          (clk),
        .rst          (rst),
        .pos_valid    (pos_valid),
        .pos_tag      (pos_tag),
        .pos_x        (pos_x),
        .pos_y        (pos_y),
        .mag_valid    (mag_valid),
        .mag_tag      (mag_tag),
        .mag_val      (mag_val),
        .out_valid    (out_valid),
        .out_tag      (out_tag),
        .out_x        (out_x),
        .out_y        (out_y),
        .out_mag      (out_mag),
        .drop_pos     (drop_pos),
        .drop_mag     (drop_mag),
        .mag_force_pop(mag_force_pop)
    );

    // -----------------------------------------------------------------
    // Per-stream item records
    // -----------------------------------------------------------------
    typedef struct {
        logic [TAG_WIDTH-1:0]        tag;
        logic signed [POS_WIDTH-1:0] x, y;
        int                          delay_after; // cycles of pos_valid=0 after pulse
    } pos_item_t;

    typedef struct {
        logic [TAG_WIDTH-1:0]        tag;
        logic signed [MAG_WIDTH-1:0] val;
        int                          delay_after;
    } mag_item_t;

    // -----------------------------------------------------------------
    // Expected matches (computed up front from the stim arrays)
    // -----------------------------------------------------------------
    typedef struct {
        logic [TAG_WIDTH-1:0]        tag;
        logic signed [POS_WIDTH-1:0] x, y;
        logic signed [MAG_WIDTH-1:0] val;
        logic                        seen;
    } match_t;

    // -----------------------------------------------------------------
    // Per-test stim and expected-match storage
    // -----------------------------------------------------------------
    pos_item_t pos_stim [0:N_STREAM_MAX-1];
    mag_item_t mag_stim [0:N_STREAM_MAX-1];
    int        n_pos_stim, n_mag_stim;

    match_t    expected [0:N_STREAM_MAX-1];
    int        n_expected;
    int        n_expected_drops_pos;
    int        n_expected_drops_mag;

    // -----------------------------------------------------------------
    // Capture counters and flags
    // -----------------------------------------------------------------
    int n_out_valid_pulses;
    int n_drop_pos_pulses;
    int n_drop_mag_pulses;
    int n_mag_force_pulses;
    int n_pulse_violations;
    int n_mutex_violations;
    int n_unexpected_match;
    int n_match_field_err;

    logic prev_out_valid, prev_drop_pos, prev_drop_mag, prev_mag_force_pop;

    logic pos_done, mag_done, test_running;
    int   tests_passed, tests_failed;

    // -----------------------------------------------------------------
    // Helper: precompute expected matches/drops from the stim arrays
    // using the same logic the DUT applies (pop both on equal, drop the
    // older head otherwise). The mag FIFO is depth-DEPTH with force-pop
    // of oldest on overflow; the pos FIFO drops *new* pushes on full.
    // -----------------------------------------------------------------
    task automatic compute_expected();
        // Local mirrors of the two FIFOs
        pos_item_t pq [$];
        mag_item_t mq [$];
        int        pi, mi;

        n_expected           = 0;
        n_expected_drops_pos = 0;
        n_expected_drops_mag = 0;

        // Push everything into the mirrors honoring DEPTH constraints.
        // (We don't care about *cycle-accurate* arrival order for the
        // scoreboard; the DUT sees both streams and matches by tag, so
        // as long as both streams individually preserve order, the
        // expected match set is independent of interleaving ? except
        // when overflow forces a drop. Since our test sizing keeps both
        // streams within DEPTH at any given instant, no force-pop
        // occurs in the matched-pair tests; the dedicated overflow
        // test handles that case separately.)
        for (pi = 0; pi < n_pos_stim; pi++) pq.push_back(pos_stim[pi]);
        for (mi = 0; mi < n_mag_stim; mi++) mq.push_back(mag_stim[mi]);

        while ((pq.size() > 0) && (mq.size() > 0)) begin
            if (pq[0].tag == mq[0].tag) begin
                expected[n_expected].tag  = pq[0].tag;
                expected[n_expected].x    = pq[0].x;
                expected[n_expected].y    = pq[0].y;
                expected[n_expected].val  = mq[0].val;
                expected[n_expected].seen = 0;
                n_expected++;
                pq.delete(0);
                mq.delete(0);
            end else if (pq[0].tag < mq[0].tag) begin
                pq.delete(0);
                n_expected_drops_pos++;
            end else begin
                mq.delete(0);
                n_expected_drops_mag++;
            end
        end
    endtask

    // -----------------------------------------------------------------
    // Find a queued expected match by tag (linear scan; n_expected is small)
    // -----------------------------------------------------------------
    function automatic int find_expected(input logic [TAG_WIDTH-1:0] t);
        for (int i = 0; i < n_expected; i++)
            if (!expected[i].seen && expected[i].tag == t) return i;
        return -1;
    endfunction

    // -----------------------------------------------------------------
    // Stream drivers (run as parallel processes per test)
    // -----------------------------------------------------------------
    task automatic drive_pos_stream();
        for (int i = 0; i < n_pos_stim; i++) begin
            @(posedge clk);
            pos_valid <= 1'b1;
            pos_tag   <= pos_stim[i].tag;
            pos_x     <= pos_stim[i].x;
            pos_y     <= pos_stim[i].y;
            @(posedge clk);
            pos_valid <= 1'b0;
            if (pos_stim[i].delay_after > 0)
                repeat (pos_stim[i].delay_after) @(posedge clk);
        end
        pos_done = 1;
    endtask

    task automatic drive_mag_stream();
        for (int i = 0; i < n_mag_stim; i++) begin
            @(posedge clk);
            mag_valid <= 1'b1;
            mag_tag   <= mag_stim[i].tag;
            mag_val   <= mag_stim[i].val;
            @(posedge clk);
            mag_valid <= 1'b0;
            if (mag_stim[i].delay_after > 0)
                repeat (mag_stim[i].delay_after) @(posedge clk);
        end
        mag_done = 1;
    endtask

    // -----------------------------------------------------------------
    // Run one test: drive both streams in parallel, wait, then check
    // -----------------------------------------------------------------
    task automatic run_test(input string name);
        int t_pulses_before, t_drop_pos_before, t_drop_mag_before;
        int t_match_pulses, t_drop_pos_pulses, t_drop_mag_pulses;
        int local_failures;

        $display("\n  -- %s --", name);
        $display("     pos stim=%0d, mag stim=%0d, expected matches=%0d, drops(pos/mag)=%0d/%0d",
                 n_pos_stim, n_mag_stim, n_expected,
                 n_expected_drops_pos, n_expected_drops_mag);

        t_pulses_before   = n_out_valid_pulses;
        t_drop_pos_before = n_drop_pos_pulses;
        t_drop_mag_before = n_drop_mag_pulses;

        pos_done = 0; mag_done = 0; test_running = 1;
        fork
            drive_pos_stream();
            drive_mag_stream();
        join

        // Drain time: 4 cycles per FIFO entry pair is plenty for the
        // 3-stage pipelined matcher. Use a generous fixed soak.
        repeat (200) @(posedge clk);
        test_running = 0;

        t_match_pulses    = n_out_valid_pulses - t_pulses_before;
        t_drop_pos_pulses = n_drop_pos_pulses  - t_drop_pos_before;
        t_drop_mag_pulses = n_drop_mag_pulses  - t_drop_mag_before;

        local_failures = 0;

        // Check every expected match was seen
        for (int i = 0; i < n_expected; i++) begin
            if (!expected[i].seen) begin
                $display("     FAIL: expected match tag=%h not observed",
                         expected[i].tag);
                local_failures++;
            end
        end

        // Check pulse counts
        if (t_match_pulses != n_expected) begin
            $display("     FAIL: out_valid pulses got=%0d expected=%0d",
                     t_match_pulses, n_expected);
            local_failures++;
        end
        if (t_drop_pos_pulses != n_expected_drops_pos) begin
            $display("     FAIL: drop_pos pulses got=%0d expected=%0d",
                     t_drop_pos_pulses, n_expected_drops_pos);
            local_failures++;
        end
        if (t_drop_mag_pulses != n_expected_drops_mag) begin
            $display("     FAIL: drop_mag pulses got=%0d expected=%0d",
                     t_drop_mag_pulses, n_expected_drops_mag);
            local_failures++;
        end

        if (local_failures == 0) begin
            $display("     PASS");
            tests_passed++;
        end else begin
            $display("     FAILED (%0d issue%s)", local_failures,
                     (local_failures == 1) ? "" : "s");
            tests_failed++;
        end
    endtask

    // -----------------------------------------------------------------
    // Test builders
    // -----------------------------------------------------------------
    task automatic clear_stim();
        n_pos_stim = 0;
        n_mag_stim = 0;
    endtask

    task automatic add_pos(input logic [TAG_WIDTH-1:0]        t,
                           input logic signed [POS_WIDTH-1:0] x,
                           input logic signed [POS_WIDTH-1:0] y,
                           input int                          delay_after);
        pos_stim[n_pos_stim].tag         = t;
        pos_stim[n_pos_stim].x           = x;
        pos_stim[n_pos_stim].y           = y;
        pos_stim[n_pos_stim].delay_after = delay_after;
        n_pos_stim++;
    endtask

    task automatic add_mag(input logic [TAG_WIDTH-1:0]        t,
                           input logic signed [MAG_WIDTH-1:0] v,
                           input int                          delay_after);
        mag_stim[n_mag_stim].tag         = t;
        mag_stim[n_mag_stim].val         = v;
        mag_stim[n_mag_stim].delay_after = delay_after;
        n_mag_stim++;
    endtask

    // Test 1: in-order matched stream
    task automatic build_test_inorder();
        clear_stim();
        for (int i = 0; i < 6; i++) begin
            add_pos({32'hABCD_0000, 32'(i+1)},  i+10,  -(i+5),  2);
            add_mag({32'hABCD_0000, 32'(i+1)},  16'(100+i),     2);
        end
        compute_expected();
    endtask

    // Test 2: pos stream leads (mag arrives several cycles later)
    // We achieve this by giving each pos a small post-delay and each mag
    // a larger post-delay, but starting the streams together.
    task automatic build_test_pos_leads();
        clear_stim();
        for (int i = 0; i < 4; i++) begin
            add_pos({32'h1111_0000, 32'(i+1)}, i+1, i+2, 1);
            add_mag({32'h1111_0000, 32'(i+1)}, 16'(50+i), 6);
        end
        compute_expected();
    endtask

    // Test 3: mag stream leads (mag arrives ahead in absolute time)
    task automatic build_test_mag_leads();
        clear_stim();
        for (int i = 0; i < 4; i++) begin
            add_pos({32'h2222_0000, 32'(i+1)}, i*7, i*9, 6);
            add_mag({32'h2222_0000, 32'(i+1)}, 16'(20+i), 1);
        end
        compute_expected();
    endtask

    // Test 4: stale position entries (pos has tags mag will never have)
    task automatic build_test_stale_pos();
        clear_stim();
        // Pos sends tags 1, 2, 3, 4 but mag only has 3, 4
        // -> drop_pos should fire twice (for 1 and 2)
        add_pos({32'h3333_0000, 32'd1}, 1, 1, 1);
        add_pos({32'h3333_0000, 32'd2}, 2, 2, 1);
        add_pos({32'h3333_0000, 32'd3}, 3, 3, 1);
        add_pos({32'h3333_0000, 32'd4}, 4, 4, 1);
        add_mag({32'h3333_0000, 32'd3}, 16'h0033, 1);
        add_mag({32'h3333_0000, 32'd4}, 16'h0044, 1);
        compute_expected();
    endtask

    // Test 5: stale magnitude entries (mag has tags pos will never have)
    task automatic build_test_stale_mag();
        clear_stim();
        add_pos({32'h4444_0000, 32'd3}, 3, 3, 1);
        add_pos({32'h4444_0000, 32'd4}, 4, 4, 1);
        add_mag({32'h4444_0000, 32'd1}, 16'h0011, 1);
        add_mag({32'h4444_0000, 32'd2}, 16'h0022, 1);
        add_mag({32'h4444_0000, 32'd3}, 16'h0033, 1);
        add_mag({32'h4444_0000, 32'd4}, 16'h0044, 1);
        compute_expected();
    endtask

    // -----------------------------------------------------------------
    // Test 6: dedicated mag-FIFO overflow (force-pop) test.
    // This one we don't run through compute_expected() because the
    // expected behavior depends on cycle-accurate force-pop ordering.
    // Instead we just verify that mag_force_pop fires at least once
    // and no pulse-shape rules are violated.
    // -----------------------------------------------------------------
    int n_mag_force_before;
    task automatic build_and_run_test_overflow();
        $display("\n  -- mag FIFO overflow / force-pop --");
        clear_stim();
        n_mag_force_before = n_mag_force_pulses;

        // Dump 8 mags back-to-back with no pos to drain them.
        // Mag FIFO is DEPTH=4, so the last 4 should each force-pop
        // the oldest entry => mag_force_pop fires 4 times.
        for (int i = 0; i < 8; i++)
            add_mag({32'h5555_0000, 32'(i+1)}, 16'(i+1), 0);

        // No pos at all in this test.
        n_pos_stim   = 0;
        n_expected   = 0;
        n_expected_drops_pos = 0;
        n_expected_drops_mag = 0;

        pos_done = 1; mag_done = 0; test_running = 1;
        fork
            drive_mag_stream();
        join
        repeat (50) @(posedge clk);
        test_running = 0;

        if ((n_mag_force_pulses - n_mag_force_before) >= 1) begin
            $display("     PASS: mag_force_pop fired %0d times (>=1 expected)",
                     n_mag_force_pulses - n_mag_force_before);
            tests_passed++;
        end else begin
            $display("     FAIL: mag_force_pop never fired");
            tests_failed++;
        end
    endtask

    // =================================================================
    // CAPTURE PROCESS
    // =================================================================
    initial begin
        int idx;
        @(negedge rst);
        @(posedge clk);
        prev_out_valid     = 0;
        prev_drop_pos      = 0;
        prev_drop_mag      = 0;
        prev_mag_force_pop = 0;

        forever begin
            @(posedge clk);
            #1;

            // Mutual-exclusion checks
            if ((out_valid && drop_pos) ||
                (out_valid && drop_mag) ||
                (drop_pos  && drop_mag)) begin
                $display("[%0t] ERROR: illegal flag combination ov=%0b dp=%0b dm=%0b",
                         $time, out_valid, drop_pos, drop_mag);
                n_mutex_violations++;
            end

            // Pulse-shape checks (each diag must be 1-cycle)
            if (prev_out_valid     && out_valid)     n_pulse_violations++;
            if (prev_drop_pos      && drop_pos)      n_pulse_violations++;
            if (prev_drop_mag      && drop_mag)      n_pulse_violations++;
            if (prev_mag_force_pop && mag_force_pop) n_pulse_violations++;

            if (out_valid) begin
                n_out_valid_pulses++;
                idx = find_expected(out_tag);
                if (idx < 0) begin
                    $display("[%0t] ERROR: unexpected out_valid tag=%h",
                             $time, out_tag);
                    n_unexpected_match++;
                end else begin
                    expected[idx].seen = 1;
                    if (out_x   !== expected[idx].x ||
                        out_y   !== expected[idx].y ||
                        out_mag !== expected[idx].val) begin
                        $display("[%0t] ERROR: tag=%h field mismatch x=(g=%0d e=%0d) y=(g=%0d e=%0d) mag=(g=%0d e=%0d)",
                                 $time, out_tag,
                                 out_x,   expected[idx].x,
                                 out_y,   expected[idx].y,
                                 out_mag, expected[idx].val);
                        n_match_field_err++;
                    end
                end
            end
            if (drop_pos)      n_drop_pos_pulses++;
            if (drop_mag)      n_drop_mag_pulses++;
            if (mag_force_pop) n_mag_force_pulses++;

            prev_out_valid     = out_valid;
            prev_drop_pos      = drop_pos;
            prev_drop_mag      = drop_mag;
            prev_mag_force_pop = mag_force_pop;
        end
    end

    // =================================================================
    // MAIN
    // =================================================================
    initial begin
        rst       = 1;
        pos_valid = 0; pos_tag = 0; pos_x = 0; pos_y = 0;
        mag_valid = 0; mag_tag = 0; mag_val = 0;

        n_out_valid_pulses = 0;
        n_drop_pos_pulses  = 0;
        n_drop_mag_pulses  = 0;
        n_mag_force_pulses = 0;
        n_pulse_violations = 0;
        n_mutex_violations = 0;
        n_unexpected_match = 0;
        n_match_field_err  = 0;
        tests_passed       = 0;
        tests_failed       = 0;

        repeat (20) @(posedge clk);
        rst = 0;
        repeat (10) @(posedge clk);

        $display("===========================================================");
        $display(" event_builder Unit Testbench");
        $display("===========================================================");
        $display("  TAG=%0d POS=%0d MAG=%0d DEPTH=%0d",
                 TAG_WIDTH, POS_WIDTH, MAG_WIDTH, DEPTH);
        $display("===========================================================");

        build_test_inorder();      run_test("in-order matched stream");
        repeat (20) @(posedge clk);

        build_test_pos_leads();    run_test("pos leads, mag follows");
        repeat (20) @(posedge clk);

        build_test_mag_leads();    run_test("mag leads, pos follows");
        repeat (20) @(posedge clk);

        build_test_stale_pos();    run_test("stale pos entries (drop_pos)");
        repeat (20) @(posedge clk);

        build_test_stale_mag();    run_test("stale mag entries (drop_mag)");
        repeat (20) @(posedge clk);

        build_and_run_test_overflow();
        repeat (20) @(posedge clk);

        // -------- Global summary --------
        $display("\n===========================================================");
        $display(" GLOBAL CHECKS");
        $display("===========================================================");
        $display("  Pulse-shape violations: %0d", n_pulse_violations);
        $display("  Mutex violations:       %0d", n_mutex_violations);
        $display("  Unexpected matches:     %0d", n_unexpected_match);
        $display("  Match field errors:     %0d", n_match_field_err);
        $display("  out_valid pulses:       %0d", n_out_valid_pulses);
        $display("  drop_pos pulses:        %0d", n_drop_pos_pulses);
        $display("  drop_mag pulses:        %0d", n_drop_mag_pulses);
        $display("  mag_force_pop pulses:   %0d", n_mag_force_pulses);

        $display("\n===========================================================");
        $display(" SUMMARY: %0d tests passed, %0d failed",
                 tests_passed, tests_failed);
        if (tests_failed == 0 &&
            n_pulse_violations == 0 &&
            n_mutex_violations == 0 &&
            n_unexpected_match == 0 &&
            n_match_field_err  == 0)
            $display(" ALL CHECKS PASSED");
        else
            $display(" *** FAILURES DETECTED ***");
        $display("===========================================================\n");

        $finish;
    end

    initial begin #10_000_000; $error("TIMEOUT"); $finish; end

endmodule
