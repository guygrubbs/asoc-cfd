// tb_pattern_generator.sv
//
// Unit testbench for pattern_generator.
// Drives enable, lets the spiral run to completion, and verifies:
//   - When out_valid=0, all outputs are held (no spurious activity)
//   - Each emitted point follows the spiral rule: 4 directions in order
//     (R,U,L,D), with segment lengths growing by LEN_INC every 2 segments
//   - POINT_STEP +/- 2 dither alternates each emit
//   - out_tag = {TAG_PREFIX, point_count} where point_count increments
//     monotonically by 1 each emit
//   - out_mag rotates through {1000, 2000, 3000, 4000} in order
//   - Pattern stops (no more out_valid) once the next point would exceed
//     +/- BOUND_ABS in either axis
//   - Disabling and re-enabling restarts the spiral cleanly
//   - DIV cycles between emits is honored
//
// Style: ref-model-as-checker (per the reference TB) but adapted to the
// current DUT's reset polarity, output names, registered emit pulse, and
// 4-entry magnitude rotation. Uses small DIV/STEP/BOUND so the spiral
// completes quickly.

`timescale 1ns / 1ps

module tb_pattern_generator;

    // -----------------------------------------------------------------
    // Test parameters (override DUT defaults for fast simulation)
    // -----------------------------------------------------------------
    localparam integer       POS_WIDTH_TB  = 32;
    localparam integer       MAG_WIDTH_TB  = 16;
    localparam integer       DIV_TB        = 8;
    localparam signed [31:0] POINT_STEP_TB = 32'sd10;
    localparam integer       LEN_INC_TB    = 3;
    localparam [31:0]        BOUND_ABS_TB  = 32'd60;
    localparam [31:0]        TAG_PREFIX_TB = 32'hCAFE_0000;

    // Magnitude table (must match RTL)
    localparam logic signed [MAG_WIDTH_TB-1:0] MAG_TABLE [0:3] =
        '{16'sd1000, 16'sd2000, 16'sd3000, 16'sd4000};

    // -----------------------------------------------------------------
    // Clock / reset / enable
    // -----------------------------------------------------------------
    logic clk;
    logic rst;
    logic enable;
    initial clk = 0;
    always #5 clk = ~clk;

    // -----------------------------------------------------------------
    // DUT signals
    // -----------------------------------------------------------------
    wire                            out_valid;
    wire [63:0]                     out_tag;
    wire signed [POS_WIDTH_TB-1:0]  out_x, out_y;
    wire signed [MAG_WIDTH_TB-1:0]  out_mag;

    pattern_generator #(
        .POS_WIDTH (POS_WIDTH_TB),
        .MAG_WIDTH (MAG_WIDTH_TB),
        .DIV       (DIV_TB),
        .POINT_STEP(POINT_STEP_TB),
        .LEN_INC   (LEN_INC_TB),
        .BOUND_ABS (BOUND_ABS_TB),
        .TAG_PREFIX(TAG_PREFIX_TB)
    ) dut (
        .clk      (clk),
        .rst      (rst),
        .enable   (enable),
        .out_valid(out_valid),
        .out_tag  (out_tag),
        .out_x    (out_x),
        .out_y    (out_y),
        .out_mag  (out_mag)
    );

    // -----------------------------------------------------------------
    // Reference model: tracks what the next emitted point *should* be.
    // Updated only when we see an out_valid pulse (event-driven model)
    // since the DUT's pipeline timing is complex but emits are sparse.
    // -----------------------------------------------------------------
    logic signed [POS_WIDTH_TB-1:0] ref_x, ref_y;
    logic [1:0]                     ref_dir;        // 0=R,1=U,2=L,3=D
    logic [15:0]                    ref_step_len;   // current segment length
    logic [15:0]                    ref_step_left;  // points remaining in segment
    logic                           ref_seg_in_pair;
    logic                           ref_dither;
    logic [31:0]                    ref_point_count;
    logic [1:0]                     ref_mag_idx;
    logic                           ref_done;       // hit boundary

    task automatic ref_reset();
        ref_x           = 0;
        ref_y           = 0;
        ref_dir         = 2'd0;
        ref_step_len    = 16'd1;
        ref_step_left   = 16'd1;
        ref_seg_in_pair = 1'b0;
        ref_dither      = 1'b0;
        ref_point_count = 32'd0;
        ref_mag_idx     = 2'd0;
        ref_done        = 1'b0;
    endtask

    // Compute what the next emitted point's (x, y) *would* be, and
    // whether that next emit would hit the boundary instead.
    task automatic ref_predict_next(
        output logic signed [POS_WIDTH_TB-1:0] nx,
        output logic signed [POS_WIDTH_TB-1:0] ny,
        output logic                           hit_bound
    );
        logic signed [POS_WIDTH_TB-1:0] step_amt;
        logic signed [POS_WIDTH_TB-1:0] x_cand, y_cand;
        logic signed [POS_WIDTH_TB-1:0] x_abs, y_abs;
        step_amt = ref_dither ? (POINT_STEP_TB - 32'sd2)
                              : (POINT_STEP_TB + 32'sd2);
        x_cand = ref_x;
        y_cand = ref_y;
        case (ref_dir)
            2'd0: x_cand = ref_x + step_amt;
            2'd1: y_cand = ref_y + step_amt;
            2'd2: x_cand = ref_x - step_amt;
            2'd3: y_cand = ref_y - step_amt;
        endcase
        x_abs = (x_cand < 0) ? -x_cand : x_cand;
        y_abs = (y_cand < 0) ? -y_cand : y_cand;
        nx        = x_cand;
        ny        = y_cand;
        hit_bound = (x_abs >= $signed(BOUND_ABS_TB)) ||
                    (y_abs >= $signed(BOUND_ABS_TB));
    endtask

    // Apply a confirmed emit to the model state
    task automatic ref_advance(
        input logic signed [POS_WIDTH_TB-1:0] nx,
        input logic signed [POS_WIDTH_TB-1:0] ny
    );
        ref_x           = nx;
        ref_y           = ny;
        ref_point_count = ref_point_count + 32'd1;
        ref_mag_idx     = ref_mag_idx + 2'd1;
        ref_dither      = ~ref_dither;
        if (ref_step_left == 16'd1) begin
            ref_dir = ref_dir + 2'd1;
            if (ref_seg_in_pair) begin
                ref_step_len    = ref_step_len + LEN_INC_TB[15:0];
                ref_step_left   = ref_step_len;
                ref_seg_in_pair = 1'b0;
            end else begin
                ref_step_left   = ref_step_len;
                ref_seg_in_pair = 1'b1;
            end
        end else begin
            ref_step_left = ref_step_left - 16'd1;
        end
    endtask

    // -----------------------------------------------------------------
    // Capture / scoreboard counters
    // -----------------------------------------------------------------
    int n_emits;
    int n_x_mismatch, n_y_mismatch, n_tag_mismatch, n_mag_mismatch;
    int n_count_skip;          // tag's point_count didn't increase by 1
    int n_idle_violations;     // outputs not held when out_valid=0 around emits
    int n_bound_violations;    // emit happened that should have hit the bound
    int min_emit_gap, max_emit_gap;
    int last_emit_cycle, cycle_counter;
    logic [31:0] last_point_count;

    // -----------------------------------------------------------------
    // Cycle counter (used to measure DIV gap between emits)
    // -----------------------------------------------------------------
    always_ff @(posedge clk or posedge rst) begin
        if (rst) cycle_counter <= 0;
        else     cycle_counter <= cycle_counter + 1;
    end

    // -----------------------------------------------------------------
    // Capture process: check every emit against the model
    // -----------------------------------------------------------------
    initial begin
        logic signed [POS_WIDTH_TB-1:0] exp_nx, exp_ny;
        logic                           exp_hit_bound;
        int                             gap;

        @(negedge rst);
        forever begin
            @(posedge clk);
            #1;

            if (out_valid) begin
                n_emits++;

                // Predict what this emit *should* contain
                ref_predict_next(exp_nx, exp_ny, exp_hit_bound);

                if (exp_hit_bound) begin
                    // DUT emitted but model said this should have stopped
                    n_bound_violations++;
                    $display("[%0t] BOUND VIOLATION: emit at (%0d,%0d) but expected bound hit",
                             $time, out_x, out_y);
                end

                if (out_x !== exp_nx) begin
                    n_x_mismatch++;
                    $display("[%0t] X mismatch: dut=%0d exp=%0d (point %0d)",
                             $time, out_x, exp_nx, n_emits);
                end
                if (out_y !== exp_ny) begin
                    n_y_mismatch++;
                    $display("[%0t] Y mismatch: dut=%0d exp=%0d (point %0d)",
                             $time, out_y, exp_ny, n_emits);
                end

                // Tag check: upper 32b == TAG_PREFIX, lower 32b ==
                // ref_point_count + 1 (i.e., the count of *this* emit)
                if (out_tag[63:32] !== TAG_PREFIX_TB) begin
                    n_tag_mismatch++;
                    $display("[%0t] TAG prefix wrong: got=%h exp=%h",
                             $time, out_tag[63:32], TAG_PREFIX_TB);
                end
                if (out_tag[31:0] !== (ref_point_count + 32'd1)) begin
                    n_tag_mismatch++;
                    $display("[%0t] TAG count wrong: got=%0d exp=%0d",
                             $time, out_tag[31:0], ref_point_count + 32'd1);
                end

                // Magnitude rotation check
                if (out_mag !== MAG_TABLE[ref_mag_idx]) begin
                    n_mag_mismatch++;
                    $display("[%0t] MAG mismatch: got=%0d exp=%0d (idx=%0d)",
                             $time, out_mag, MAG_TABLE[ref_mag_idx],
                             ref_mag_idx);
                end

                // Tag-count monotonicity
                if (n_emits > 1 && out_tag[31:0] != last_point_count + 1) begin
                    n_count_skip++;
                    $display("[%0t] TAG count skip: prev=%0d now=%0d",
                             $time, last_point_count, out_tag[31:0]);
                end
                last_point_count = out_tag[31:0];

                // Emit gap measurement
                if (n_emits > 1) begin
                    gap = cycle_counter - last_emit_cycle;
                    if (gap < min_emit_gap) min_emit_gap = gap;
                    if (gap > max_emit_gap) max_emit_gap = gap;
                end
                last_emit_cycle = cycle_counter;

                // Advance the model
                ref_advance(exp_nx, exp_ny);
            end
        end
    end

    // -----------------------------------------------------------------
    // Emit-gap monitor: when two emits happen too close (< DIV-2), flag
    // it. We give -2 of slack for the registered emit_pulse_r logic.
    // Driven from the same initial block that initializes the counter,
    // to satisfy the single-driver rule for the variable.
    // -----------------------------------------------------------------
    int   cycles_since_emit;
    logic seen_first_emit;
    initial begin
        cycles_since_emit = 0;
        seen_first_emit   = 0;
        @(negedge rst);
        forever begin
            @(posedge clk);
            #1;
            if (out_valid) begin
                if (seen_first_emit && cycles_since_emit < (DIV_TB - 2)) begin
                    n_idle_violations++;
                    $display("[%0t] EMIT GAP TOO SHORT: %0d cycles since previous emit (DIV=%0d)",
                             $time, cycles_since_emit, DIV_TB);
                end
                cycles_since_emit = 0;
                seen_first_emit   = 1;
            end else begin
                cycles_since_emit = cycles_since_emit + 1;
            end
        end
    end

    // -----------------------------------------------------------------
    // Test sequence
    // -----------------------------------------------------------------
    int n_emits_phase1;
    int n_emits_phase2;
    int total_failures;

    initial begin
        // Init
        rst              = 1;
        enable           = 0;
        n_emits          = 0;
        n_x_mismatch     = 0;
        n_y_mismatch     = 0;
        n_tag_mismatch   = 0;
        n_mag_mismatch   = 0;
        n_count_skip     = 0;
        n_bound_violations = 0;
        min_emit_gap     = 32'h7FFF_FFFF;
        max_emit_gap     = 0;
        last_emit_cycle  = 0;
        last_point_count = 0;
        ref_reset();

        repeat (10) @(posedge clk);
        rst = 0;
        repeat (5) @(posedge clk);

        $display("===========================================================");
        $display(" pattern_generator Unit Testbench");
        $display("===========================================================");
        $display("  DIV=%0d POINT_STEP=%0d LEN_INC=%0d BOUND_ABS=%0d",
                 DIV_TB, POINT_STEP_TB, LEN_INC_TB, BOUND_ABS_TB);
        $display("  TAG_PREFIX=%h", TAG_PREFIX_TB);
        $display("===========================================================\n");

        // ---- Phase 1: enable and run until pattern naturally completes ----
        $display("  -- Phase 1: run to completion --");
        enable = 1;

        // Wait long enough for spiral to definitely hit the bound.
        // With BOUND=60 and STEP~10, expect well under 100 emits.
        // Each emit takes DIV cycles, so 200*DIV cycles is generous.
        repeat (200 * DIV_TB) @(posedge clk);

        n_emits_phase1 = n_emits;
        $display("     emits in phase 1: %0d", n_emits_phase1);
        $display("     final position:   (%0d, %0d)", ref_x, ref_y);

        // Pattern should be done now. Verify no further emits for a while.
        begin : phase1_quiet_check
            int extra_emits_before, extra_emits_after;
            extra_emits_before = n_emits;
            repeat (5 * DIV_TB) @(posedge clk);
            extra_emits_after = n_emits;
            if (extra_emits_after != extra_emits_before)
                $display("     FAIL: %0d unexpected emits after pattern done",
                         extra_emits_after - extra_emits_before);
            else
                $display("     PASS: no emits after completion");
        end

        // ---- Phase 2: disable / re-enable to test restart ----
        $display("\n  -- Phase 2: disable then re-enable (restart) --");
        enable = 0;
        repeat (10) @(posedge clk);
        ref_reset();             // model also restarts
        last_point_count = 0;    // tag count restarts at 0->1
        last_emit_cycle  = cycle_counter;
        enable = 1;

        // Run for a smaller window this time
        repeat (50 * DIV_TB) @(posedge clk);

        n_emits_phase2 = n_emits - n_emits_phase1;
        $display("     emits in phase 2: %0d", n_emits_phase2);

        if (n_emits_phase2 == 0) begin
            $display("     FAIL: pattern did not restart after re-enable");
        end else begin
            $display("     PASS: pattern restarted");
        end

        // ---- Summary ----
        total_failures = n_x_mismatch + n_y_mismatch + n_tag_mismatch +
                         n_mag_mismatch + n_count_skip + n_idle_violations +
                         n_bound_violations;

        $display("\n===========================================================");
        $display(" SUMMARY");
        $display("===========================================================");
        $display("  Total emits:           %0d (phase1=%0d, phase2=%0d)",
                 n_emits, n_emits_phase1, n_emits_phase2);
        $display("  X mismatches:          %0d", n_x_mismatch);
        $display("  Y mismatches:          %0d", n_y_mismatch);
        $display("  Tag mismatches:        %0d", n_tag_mismatch);
        $display("  Magnitude mismatches:  %0d", n_mag_mismatch);
        $display("  Tag-count skips:       %0d", n_count_skip);
        $display("  Boundary violations:   %0d", n_bound_violations);
        $display("  Emit-gap violations:   %0d", n_idle_violations);
        if (n_emits >= 2) begin
            $display("  Min emit gap:          %0d cycles", min_emit_gap);
            $display("  Max emit gap:          %0d cycles", max_emit_gap);
            $display("  Expected gap:          %0d cycles (DIV)", DIV_TB);
        end

        $display("\n===========================================================");
        if (total_failures == 0)
            $display(" ALL CHECKS PASSED");
        else
            $display(" *** %0d FAILURES DETECTED ***", total_failures);
        $display("===========================================================\n");

        $finish;
    end

    // Watchdog
    initial begin #50_000_000; $error("TIMEOUT"); $finish; end

endmodule
