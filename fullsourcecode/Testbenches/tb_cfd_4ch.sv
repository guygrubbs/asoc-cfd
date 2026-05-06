// tb_cfd_4ch.sv
//
// Unit testbench for cfd_4ch.
//
// Drives synthetic Gaussian pulses on 4 channels with sample_valid framing
// the event boundaries. Verifies behavioral properties:
//   - event_valid pulses with 4 timestamps when all 4 channels detect a ZC
//   - event_fail pulses when threshold is too high to ever arm
//   - event_dropped pulses when a new event ends inside the FSM busy window
//   - Tag tracking: tag_out matches the right tag for each pulse type
//   - Within-event consistency: same pulse on all 4 channels -> timestamps
//     within a few samples of each other
//   - Across-event consistency: shifting one channel's pulse by N samples
//     shifts its timestamp by ~N
//   - Back-to-back: 4 normal events all produce event_valid
//
// We do NOT model the CFD pipeline bit-accurately. Absolute timestamp values
// depend on the 5-stage internal pipeline offset and aren't checked. Deltas
// and clustering are checked, which catches the more common bug classes.

`timescale 1ns / 1ps

// Behavioral model of the SmartGen 16x14 multiplier IP.
// Keep this only if you are not separately compiling the SmartGen multiplier.
// If ModelSim reports that mult_16x14_3p is already defined, remove this module.
module mult_16x14_3p (
    input  wire        Clock,
    input  wire [15:0] DataA,
    input  wire [13:0] DataB,
    output wire [29:0] Mult
);
    reg signed [29:0] stage1, stage2, stage3;
    wire signed [15:0] a_signed = $signed(DataA);
    wire signed [13:0] b_signed = $signed(DataB);

    always @(posedge Clock) begin
        stage1 <= a_signed * b_signed;
        stage2 <= stage1;
        stage3 <= stage2;
    end

    assign Mult = stage3;
endmodule


module tb_cfd_4ch;

    // -----------------------------------------------------------------
    // Parameters
    // -----------------------------------------------------------------
    localparam integer DELAY_RAM_AW      = 7;
    localparam integer MAX_EVENT_SAMPLES = 1024;
    localparam integer IDX_W             = 10;
    localparam integer FRAC_BITS         = 10;

    localparam [DELAY_RAM_AW-1:0] DELAY_VAL          = 38;
    localparam [13:0]             ATT_Q0_13          = 14'd7373;   // ~0.9
    localparam signed [15:0]      THRESHOLD_NORMAL   = 16'sd2000;
    localparam signed [15:0]      THRESHOLD_TOO_HIGH = 16'sd28000;
    localparam [7:0]              ZC_NEG             = 8'd3;

    localparam integer N_SAMPLES_MAX  = 320;
    localparam integer N_SAMPLES      = 300;
    localparam real    SIGMA_SAMPLES  = 12.0;
    localparam real    PEAK_AMPLITUDE = 8000.0;

    localparam integer CFD_LATENCY = 46;
    localparam integer GAP_CYCLES  = 2;

    // -----------------------------------------------------------------
    // Clock / reset
    // -----------------------------------------------------------------
    logic clk;
    logic rst;

    initial clk = 1'b0;
    always #5 clk = ~clk;

    // -----------------------------------------------------------------
    // DUT signals
    // -----------------------------------------------------------------
    logic                    sample_valid;
    logic signed [15:0]      x1_in, x2_in, y1_in, y2_in;
    logic [13:0]             att_q0_13;
    logic [DELAY_RAM_AW-1:0] delay_val;
    logic signed [15:0]      threshold;
    logic [7:0]              zc_neg_samples;
    logic [63:0]             tag_in;

    wire [IDX_W-1:0]     tpx1_int,  tpx2_int,  tpy1_int,  tpy2_int;
    wire [FRAC_BITS-1:0] tpx1_frac, tpx2_frac, tpy1_frac, tpy2_frac;
    wire                 event_valid;
    wire                 event_fail;
    wire                 event_dropped;
    wire [63:0]          tag_out;

    cfd_4ch #(
        .DELAY_RAM_AW      (DELAY_RAM_AW),
        .MAX_EVENT_SAMPLES (MAX_EVENT_SAMPLES),
        .IDX_W             (IDX_W),
        .FRAC_BITS         (FRAC_BITS)
    ) dut (
        .clk            (clk),
        .rst            (rst),
        .sample_valid   (sample_valid),

        .x1_in          (x1_in),
        .x2_in          (x2_in),
        .y1_in          (y1_in),
        .y2_in          (y2_in),

        .att_q0_13      (att_q0_13),
        .delay_val      (delay_val),
        .threshold      (threshold),
        .zc_neg_samples (zc_neg_samples),
        .tag_in         (tag_in),

        .tpx1_int       (tpx1_int),
        .tpx1_frac      (tpx1_frac),
        .tpx2_int       (tpx2_int),
        .tpx2_frac      (tpx2_frac),
        .tpy1_int       (tpy1_int),
        .tpy1_frac      (tpy1_frac),
        .tpy2_int       (tpy2_int),
        .tpy2_frac      (tpy2_frac),

        .event_valid    (event_valid),
        .event_fail     (event_fail),
        .event_dropped  (event_dropped),
        .tag_out        (tag_out)
    );

    // -----------------------------------------------------------------
    // Per-event sample buffers
    // -----------------------------------------------------------------
    logic signed [15:0] ev_x1 [0:N_SAMPLES_MAX-1];
    logic signed [15:0] ev_x2 [0:N_SAMPLES_MAX-1];
    logic signed [15:0] ev_y1 [0:N_SAMPLES_MAX-1];
    logic signed [15:0] ev_y2 [0:N_SAMPLES_MAX-1];

    // -----------------------------------------------------------------
    // Captured DUT outputs
    // -----------------------------------------------------------------
    logic [63:0]          cap_valid_tag;
    logic [IDX_W-1:0]     cap_ts_int  [0:3];
    logic [FRAC_BITS-1:0] cap_ts_frac [0:3];
    logic [63:0]          cap_fail_tag;
    logic [63:0]          cap_dropped_tag;

    int n_valid_total;
    int n_fail_total;
    int n_dropped_total;

    integer cap_i;

    // Important:
    // These variables are driven only here. Do not also initialize them
    // in the initial block, or always_ff single-driver errors will occur.
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            cap_valid_tag   <= 64'd0;
            cap_fail_tag    <= 64'd0;
            cap_dropped_tag <= 64'd0;

            n_valid_total   <= 0;
            n_fail_total    <= 0;
            n_dropped_total <= 0;

            for (cap_i = 0; cap_i < 4; cap_i = cap_i + 1) begin
                cap_ts_int [cap_i] <= {IDX_W{1'b0}};
                cap_ts_frac[cap_i] <= {FRAC_BITS{1'b0}};
            end
        end else begin
            if (event_valid) begin
                cap_valid_tag  <= tag_out;

                cap_ts_int [0] <= tpx1_int;
                cap_ts_frac[0] <= tpx1_frac;

                cap_ts_int [1] <= tpx2_int;
                cap_ts_frac[1] <= tpx2_frac;

                cap_ts_int [2] <= tpy1_int;
                cap_ts_frac[2] <= tpy1_frac;

                cap_ts_int [3] <= tpy2_int;
                cap_ts_frac[3] <= tpy2_frac;

                n_valid_total  <= n_valid_total + 1;
            end

            if (event_fail) begin
                cap_fail_tag <= tag_out;
                n_fail_total <= n_fail_total + 1;
            end

            if (event_dropped) begin
                cap_dropped_tag <= tag_out;
                n_dropped_total <= n_dropped_total + 1;
            end
        end
    end

    // -----------------------------------------------------------------
    // Pulse generator: Gaussian centered at n_peak
    // -----------------------------------------------------------------
    task automatic gen_pulse(
        input  real n_peak,
        input  real sigma_samples,
        input  real amplitude,
        input  int  n_samples,
        output logic signed [15:0] buf_out [0:N_SAMPLES_MAX-1]
    );
        real val;
        real dn;
        int  q;

        begin
            for (int n = 0; n < N_SAMPLES_MAX; n++) begin
                buf_out[n] = 16'sd0;
            end

            for (int n = 0; n < n_samples; n++) begin
                dn  = real'(n) - n_peak;
                val = amplitude * $exp(-(dn * dn) /
                                      (2.0 * sigma_samples * sigma_samples));

                q = int'(val + 0.5);

                if (q >  32767) q =  32767;
                if (q < -32768) q = -32768;

                buf_out[n] = q[15:0];
            end
        end
    endtask

    // -----------------------------------------------------------------
    // Drive a full event with possibly different per-channel peaks
    // -----------------------------------------------------------------
    task automatic drive_event(
        input logic [63:0] tag,
        input real         n_peak_x1,
        input real         n_peak_x2,
        input real         n_peak_y1,
        input real         n_peak_y2,
        input real         amplitude,
        input int          n_samples
    );
        begin
            gen_pulse(n_peak_x1, SIGMA_SAMPLES, amplitude, n_samples, ev_x1);
            gen_pulse(n_peak_x2, SIGMA_SAMPLES, amplitude, n_samples, ev_x2);
            gen_pulse(n_peak_y1, SIGMA_SAMPLES, amplitude, n_samples, ev_y1);
            gen_pulse(n_peak_y2, SIGMA_SAMPLES, amplitude, n_samples, ev_y2);

            @(posedge clk);
            tag_in <= tag;

            for (int n = 0; n < n_samples; n++) begin
                x1_in        <= ev_x1[n];
                x2_in        <= ev_x2[n];
                y1_in        <= ev_y1[n];
                y2_in        <= ev_y2[n];
                sample_valid <= 1'b1;
                @(posedge clk);
            end

            x1_in        <= 16'sd0;
            x2_in        <= 16'sd0;
            y1_in        <= 16'sd0;
            y2_in        <= 16'sd0;
            sample_valid <= 1'b0;
        end
    endtask

    task automatic drive_event_uniform(
        input logic [63:0] tag,
        input real         n_peak,
        input real         amplitude,
        input int          n_samples
    );
        begin
            drive_event(tag,
                        n_peak, n_peak, n_peak, n_peak,
                        amplitude,
                        n_samples);
        end
    endtask

    // For the dropped-event test: a tiny zero-content event whose end
    // will land inside the FSM busy window.
    task automatic drive_short_event(
        input logic [63:0] tag,
        input int          len
    );
        begin
            @(posedge clk);
            tag_in       <= tag;
            x1_in        <= 16'sd0;
            x2_in        <= 16'sd0;
            y1_in        <= 16'sd0;
            y2_in        <= 16'sd0;
            sample_valid <= 1'b1;

            repeat (len) @(posedge clk);

            sample_valid <= 1'b0;
        end
    endtask

    // -----------------------------------------------------------------
    // Wait helpers
    // -----------------------------------------------------------------
    task automatic wait_for_valid_count(
        input int target_count,
        input int timeout_cycles
    );
        int k;
        begin
            k = 0;

            while ((n_valid_total < target_count) && (k < timeout_cycles)) begin
                @(posedge clk);
                #1;
                k++;
            end
        end
    endtask

    task automatic wait_for_fail_count(
        input int target_count,
        input int timeout_cycles
    );
        int k;
        begin
            k = 0;

            while ((n_fail_total < target_count) && (k < timeout_cycles)) begin
                @(posedge clk);
                #1;
                k++;
            end
        end
    endtask

    task automatic wait_for_dropped_count(
        input int target_count,
        input int timeout_cycles
    );
        int k;
        begin
            k = 0;

            while ((n_dropped_total < target_count) && (k < timeout_cycles)) begin
                @(posedge clk);
                #1;
                k++;
            end
        end
    endtask

    // -----------------------------------------------------------------
    // Test cases
    // -----------------------------------------------------------------
    int tests_passed;
    int tests_failed;

    task automatic test_basic_event();
        int prev_valid;
        int min_ts;
        int max_ts;
        int spread;

        begin
            $display("\n  -- Test 1: basic uniform event --");

            threshold  = THRESHOLD_NORMAL;
            prev_valid = n_valid_total;

            drive_event_uniform(64'hCFD0_0000_0000_1001,
                                150.0,
                                PEAK_AMPLITUDE,
                                N_SAMPLES);

            wait_for_valid_count(prev_valid + 1, CFD_LATENCY + 100);

            if ((n_valid_total != prev_valid + 1) ||
                (cap_valid_tag !== 64'hCFD0_0000_0000_1001)) begin
                $display("     FAIL: did not get event_valid with correct tag (got %0d pulses, tag=%h)",
                         n_valid_total - prev_valid,
                         cap_valid_tag);
                tests_failed++;
                return;
            end

            min_ts = cap_ts_int[0];
            max_ts = cap_ts_int[0];

            for (int c = 1; c < 4; c++) begin
                if (cap_ts_int[c] < min_ts) min_ts = cap_ts_int[c];
                if (cap_ts_int[c] > max_ts) max_ts = cap_ts_int[c];
            end

            spread = max_ts - min_ts;

            $display("     ts_int  = [%0d, %0d, %0d, %0d]",
                     cap_ts_int[0],
                     cap_ts_int[1],
                     cap_ts_int[2],
                     cap_ts_int[3]);

            $display("     ts_frac = [%0d, %0d, %0d, %0d] / 1024",
                     cap_ts_frac[0],
                     cap_ts_frac[1],
                     cap_ts_frac[2],
                     cap_ts_frac[3]);

            $display("     spread  = %0d samples", spread);

            if (spread <= 2) begin
                $display("     PASS");
                tests_passed++;
            end else begin
                $display("     FAIL: timestamps too spread (got %0d, expected <=2)",
                         spread);
                tests_failed++;
            end
        end
    endtask

    task automatic test_threshold_too_high();
        int prev_fail;

        begin
            $display("\n  -- Test 2: threshold too high (event_fail) --");

            threshold = THRESHOLD_TOO_HIGH;
            prev_fail = n_fail_total;

            drive_event_uniform(64'hCFD0_0000_0000_2002,
                                150.0,
                                PEAK_AMPLITUDE,
                                N_SAMPLES);

            wait_for_fail_count(prev_fail + 1, CFD_LATENCY + 100);

            threshold = THRESHOLD_NORMAL;

            if ((n_fail_total == prev_fail + 1) &&
                (cap_fail_tag === 64'hCFD0_0000_0000_2002)) begin
                $display("     PASS: event_fail fired with correct tag");
                tests_passed++;
            end else begin
                $display("     FAIL: expected event_fail with tag CFD0_..._2002 (got %0d pulses, tag=%h)",
                         n_fail_total - prev_fail,
                         cap_fail_tag);
                tests_failed++;
            end
        end
    endtask

    task automatic test_time_shifted();
        int prev_valid;
        int dx;
        int dy;

        begin
            $display("\n  -- Test 3: x2/y2 shifted by +5 samples --");

            threshold  = THRESHOLD_NORMAL;
            prev_valid = n_valid_total;

            drive_event(64'hCFD0_0000_0000_3003,
                        150.0,
                        155.0,
                        150.0,
                        155.0,
                        PEAK_AMPLITUDE,
                        N_SAMPLES);

            wait_for_valid_count(prev_valid + 1, CFD_LATENCY + 100);

            if ((n_valid_total != prev_valid + 1) ||
                (cap_valid_tag !== 64'hCFD0_0000_0000_3003)) begin
                $display("     FAIL: did not get event_valid for shifted event");
                tests_failed++;
                return;
            end

            dx = cap_ts_int[1] - cap_ts_int[0];
            dy = cap_ts_int[3] - cap_ts_int[2];

            $display("     ts_int  = [%0d, %0d, %0d, %0d]",
                     cap_ts_int[0],
                     cap_ts_int[1],
                     cap_ts_int[2],
                     cap_ts_int[3]);

            $display("     dx = %0d, dy = %0d (expected ~5)", dx, dy);

            if ((dx >= 4) && (dx <= 6) &&
                (dy >= 4) && (dy <= 6)) begin
                $display("     PASS");
                tests_passed++;
            end else begin
                $display("     FAIL: shift mismatch");
                tests_failed++;
            end
        end
    endtask

    task automatic test_back_to_back();
        int prev_valid;
        int got;

        begin
            $display("\n  -- Test 4: 4 back-to-back events --");

            threshold  = THRESHOLD_NORMAL;
            prev_valid = n_valid_total;

            for (int i = 0; i < 4; i++) begin
                drive_event_uniform({32'hCFD0_4040, 32'd0 + i + 1},
                                    150.0,
                                    PEAK_AMPLITUDE,
                                    N_SAMPLES);

                repeat (GAP_CYCLES) @(posedge clk);
            end

            wait_for_valid_count(prev_valid + 4, CFD_LATENCY + 200);

            got = n_valid_total - prev_valid;

            if ((got == 4) &&
                (cap_valid_tag === {32'hCFD0_4040, 32'd4})) begin
                $display("     PASS: got 4 event_valid pulses, last tag matches");
                tests_passed++;
            end else begin
                $display("     FAIL: expected 4 event_valid pulses (got %0d), last tag %h",
                         got,
                         cap_valid_tag);
                tests_failed++;
            end
        end
    endtask

    task automatic test_event_dropped();
        int   prev_valid;
        int   prev_dropped;
        logic dropped_ok;
        logic valid_ok;

        begin
            $display("\n  -- Test 5: event dropped while FSM busy --");

            threshold    = THRESHOLD_NORMAL;
            prev_valid   = n_valid_total;
            prev_dropped = n_dropped_total;

            drive_event_uniform(64'hCFD0_0000_0000_5050,
                                150.0,
                                PEAK_AMPLITUDE,
                                N_SAMPLES);

            repeat (5) @(posedge clk);

            drive_short_event(64'hCFD0_0000_0000_5051, 8);

            wait_for_dropped_count(prev_dropped + 1, CFD_LATENCY + 100);
            wait_for_valid_count(prev_valid + 1, CFD_LATENCY + 150);

            dropped_ok = ((n_dropped_total - prev_dropped) >= 1) &&
                         (cap_dropped_tag === 64'hCFD0_0000_0000_5051);

            valid_ok = ((n_valid_total - prev_valid) >= 1) &&
                       (cap_valid_tag === 64'hCFD0_0000_0000_5050);

            if (dropped_ok) begin
                $display("     PASS: event_dropped fired with short-event tag");
            end else begin
                $display("     FAIL: expected event_dropped with tag CFD0_..._5051 (got %0d, tag=%h)",
                         n_dropped_total - prev_dropped,
                         cap_dropped_tag);
            end

            if (valid_ok) begin
                $display("     PASS: original event still completed (event_valid with original tag)");
            end else begin
                $display("     FAIL: expected event_valid with tag CFD0_..._5050 (got %0d, tag=%h)",
                         n_valid_total - prev_valid,
                         cap_valid_tag);
            end

            if (dropped_ok && valid_ok) begin
                tests_passed++;
            end else begin
                tests_failed++;
            end
        end
    endtask

    // -----------------------------------------------------------------
    // Main
    // -----------------------------------------------------------------
    initial begin
        rst            = 1'b1;
        sample_valid   = 1'b0;

        x1_in = 16'sd0;
        x2_in = 16'sd0;
        y1_in = 16'sd0;
        y2_in = 16'sd0;

        att_q0_13      = ATT_Q0_13;
        delay_val      = DELAY_VAL;
        threshold      = THRESHOLD_NORMAL;
        zc_neg_samples = ZC_NEG;
        tag_in         = 64'd0;

        tests_passed = 0;
        tests_failed = 0;

        repeat (20) @(posedge clk);
        rst = 1'b0;
        repeat (10) @(posedge clk);

        $display("===========================================================");
        $display(" cfd_4ch Unit Testbench");
        $display("===========================================================");
        $display("  DELAY_VAL=%0d  ATT_Q0_13=%0d (~%.4f)",
                 DELAY_VAL,
                 ATT_Q0_13,
                 real'(ATT_Q0_13) / 8192.0);
        $display("  THRESHOLD=%0d  ZC_NEG=%0d",
                 THRESHOLD_NORMAL,
                 ZC_NEG);
        $display("  Pulse: amp=%.0f sigma=%.1f n_samples=%0d",
                 PEAK_AMPLITUDE,
                 SIGMA_SAMPLES,
                 N_SAMPLES);
        $display("  Latency: %0d cycles", CFD_LATENCY);
        $display("===========================================================");

        test_basic_event();
        repeat (CFD_LATENCY + 20) @(posedge clk);

        test_threshold_too_high();
        repeat (CFD_LATENCY + 20) @(posedge clk);

        test_time_shifted();
        repeat (CFD_LATENCY + 20) @(posedge clk);

        test_back_to_back();
        repeat (CFD_LATENCY + 20) @(posedge clk);

        test_event_dropped();
        repeat (CFD_LATENCY + 20) @(posedge clk);

        $display("\n===========================================================");
        $display(" SUMMARY: %0d tests passed, %0d failed",
                 tests_passed,
                 tests_failed);

        if (tests_failed == 0) begin
            $display(" ALL CHECKS PASSED");
        end else begin
            $display(" *** FAILURES DETECTED ***");
        end

        $display("===========================================================");
        $display("\n  Total event_valid pulses:   %0d", n_valid_total);
        $display("  Total event_fail pulses:    %0d", n_fail_total);
        $display("  Total event_dropped pulses: %0d", n_dropped_total);
        $display("");

        $finish;
    end

    initial begin
        #50_000_000;
        $error("TIMEOUT");
        $finish;
    end

endmodule
