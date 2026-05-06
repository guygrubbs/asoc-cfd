// tb_peak_detector_4ch.sv
//
// Unit testbench for peak_detector_4ch.
// Drives bursts of samples on 4 signed ADC channels with a 1-cycle gap of
// in_valid=0 between events. The DUT should:
//   - Track the per-channel peak (max signed value) over each burst.
//   - On the falling edge of in_valid, emit a 1-cycle out_valid pulse with
//     out_sum = (peak0+peak1+peak2+peak3) arithmetically right-shifted by 3
//     (Q12.3 -> Q15.0), and out_timestamp = the in_timestamp latched from
//     the first sample of the burst.
//
// Coverage:
//   - Single-sample bursts
//   - Long bursts (peak placement scattered by random seed)
//   - All-zero, all-PEAK_MIN, all-PEAK_MAX bursts
//   - All-negative bursts (peak is least-negative)
//   - Mixed-sign random bursts of varying lengths
//   - Back-to-back events with 1-cycle gap

`timescale 1ns / 1ps

module tb_peak_detector_4ch;

    // -----------------------------------------------------------------
    // Parameters
    // -----------------------------------------------------------------
    localparam integer ADC_WIDTH      = 16;
    localparam integer TS_WIDTH       = 64;
    localparam integer OUT_WIDTH      = 16;
    localparam integer FRAC_BITS      = 3;
    localparam integer PEAK_SUM_WIDTH = ADC_WIDTH + 2;  // 18

    localparam integer N_DIRECTED    = 6;
    localparam integer N_RANDOM      = 14;
    localparam integer N_EVENTS      = N_DIRECTED + N_RANDOM;
    localparam integer GAP_CYCLES    = 1;
    localparam integer MAX_BURST_LEN = 64;

    localparam logic signed [ADC_WIDTH-1:0] PEAK_MIN_VAL = 16'sh8000; // -32768

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
    logic                        in_valid;
    logic signed [ADC_WIDTH-1:0] adc0, adc1, adc2, adc3;
    logic [TS_WIDTH-1:0]         in_timestamp;

    wire                         out_valid;
    wire signed [OUT_WIDTH-1:0]  out_sum;
    wire [TS_WIDTH-1:0]          out_timestamp;

    peak_detector_4ch #(
        .ADC_WIDTH(ADC_WIDTH),
        .TS_WIDTH (TS_WIDTH),
        .OUT_WIDTH(OUT_WIDTH)
    ) dut (
        .clk          (clk),
        .rst          (rst),
        .in_valid     (in_valid),
        .adc0         (adc0),
        .adc1         (adc1),
        .adc2         (adc2),
        .adc3         (adc3),
        .in_timestamp (in_timestamp),
        .out_valid    (out_valid),
        .out_sum      (out_sum),
        .out_timestamp(out_timestamp)
    );

    // -----------------------------------------------------------------
    // PRNG
    // -----------------------------------------------------------------
    longint unsigned prng_state;
    function automatic longint unsigned xorshift64();
        longint unsigned s;
        s = prng_state;
        s = s ^ (s << 13);
        s = s ^ (s >> 7);
        s = s ^ (s << 17);
        prng_state = s;
        return s;
    endfunction

    function automatic int rand_in_range(input int lo, input int hi);
        longint unsigned span;
        longint unsigned r;
        span = longint'(hi - lo + 1);
        r    = xorshift64();
        return lo + int'(r % span);
    endfunction

    // -----------------------------------------------------------------
    // Per-event record
    // -----------------------------------------------------------------
    typedef struct {
        logic [TS_WIDTH-1:0]         tag;
        int                          length;
        int                          range_lo;
        int                          range_hi;
        // Computed expected values (model)
        logic signed [ADC_WIDTH-1:0] peak0, peak1, peak2, peak3;
        logic signed [OUT_WIDTH-1:0] exp_sum;
        // Captured DUT outputs
        logic                        got_result;
        logic signed [OUT_WIDTH-1:0] dut_sum;
        logic [TS_WIDTH-1:0]         dut_tag;
    } ev_rec_t;

    ev_rec_t ev [0:N_EVENTS-1];

    // Sample buffers for the event currently being driven
    logic signed [ADC_WIDTH-1:0] s0 [0:MAX_BURST_LEN-1];
    logic signed [ADC_WIDTH-1:0] s1 [0:MAX_BURST_LEN-1];
    logic signed [ADC_WIDTH-1:0] s2 [0:MAX_BURST_LEN-1];
    logic signed [ADC_WIDTH-1:0] s3 [0:MAX_BURST_LEN-1];

    logic drive_done;
    int   n_valid_pulses;

    // -----------------------------------------------------------------
    // Helpers
    // -----------------------------------------------------------------
    function automatic int find_ev(input logic [TS_WIDTH-1:0] t);
        int idx;
        if (t[63:32] != 32'hF00D_0000) return -1;
        idx = int'(t[31:0]) - 1;
        if (idx < 0 || idx >= N_EVENTS) return -1;
        return idx;
    endfunction

    // Bit-accurate model of the DUT's peak_sum_truncated: 18-bit signed
    // sum, then {s[17], s[17:3]} for OUT_WIDTH=16.
    function automatic logic signed [OUT_WIDTH-1:0] expected_sum_from_peaks(
        input logic signed [ADC_WIDTH-1:0] p0,
        input logic signed [ADC_WIDTH-1:0] p1,
        input logic signed [ADC_WIDTH-1:0] p2,
        input logic signed [ADC_WIDTH-1:0] p3
    );
        logic signed [PEAK_SUM_WIDTH-1:0] s;
        s = p0 + p1 + p2 + p3;
        return {s[PEAK_SUM_WIDTH-1], s[PEAK_SUM_WIDTH-1:FRAC_BITS]};
    endfunction

    task automatic gen_event_samples(input int idx);
        logic signed [ADC_WIDTH-1:0] p0, p1, p2, p3;
        p0 = PEAK_MIN_VAL; p1 = PEAK_MIN_VAL;
        p2 = PEAK_MIN_VAL; p3 = PEAK_MIN_VAL;
        for (int k = 0; k < ev[idx].length; k++) begin
            s0[k] = rand_in_range(ev[idx].range_lo, ev[idx].range_hi);
            s1[k] = rand_in_range(ev[idx].range_lo, ev[idx].range_hi);
            s2[k] = rand_in_range(ev[idx].range_lo, ev[idx].range_hi);
            s3[k] = rand_in_range(ev[idx].range_lo, ev[idx].range_hi);
            if (s0[k] > p0) p0 = s0[k];
            if (s1[k] > p1) p1 = s1[k];
            if (s2[k] > p2) p2 = s2[k];
            if (s3[k] > p3) p3 = s3[k];
        end
        ev[idx].peak0   = p0;
        ev[idx].peak1   = p1;
        ev[idx].peak2   = p2;
        ev[idx].peak3   = p3;
        ev[idx].exp_sum = expected_sum_from_peaks(p0, p1, p2, p3);
    endtask

    task automatic set_uniform_samples(input int idx,
                                        input logic signed [ADC_WIDTH-1:0] v);
        for (int k = 0; k < ev[idx].length; k++) begin
            s0[k] = v; s1[k] = v; s2[k] = v; s3[k] = v;
        end
        ev[idx].peak0   = v;
        ev[idx].peak1   = v;
        ev[idx].peak2   = v;
        ev[idx].peak3   = v;
        ev[idx].exp_sum = expected_sum_from_peaks(v, v, v, v);
    endtask

    // Caller must already be aligned to a posedge clk on entry.
    task automatic drive_event(input int idx);
        for (int k = 0; k < ev[idx].length; k++) begin
            adc0         <= s0[k];
            adc1         <= s1[k];
            adc2         <= s2[k];
            adc3         <= s3[k];
            in_valid     <= 1'b1;
            in_timestamp <= ev[idx].tag;
            @(posedge clk);
        end
        in_valid <= 1'b0;
        adc0 <= '0; adc1 <= '0; adc2 <= '0; adc3 <= '0;
        repeat (GAP_CYCLES) @(posedge clk);
    endtask

    // =================================================================
    // DRIVE PROCESS
    // =================================================================
    initial begin
        drive_done = 0;
        rst = 1;
        in_valid = 0;
        adc0 = 0; adc1 = 0; adc2 = 0; adc3 = 0;
        in_timestamp = 0;
        prng_state = 64'hFEED_FACE_CAFE_BABE;
        n_valid_pulses = 0;
        for (int i = 0; i < N_EVENTS; i++) ev[i].got_result = 0;

        repeat (20) @(posedge clk);
        rst = 0;
        repeat (10) @(posedge clk);

        $display("===========================================================");
        $display(" peak_detector_4ch Unit Testbench");
        $display("===========================================================");
        $display("  ADC_WIDTH=%0d  TS_WIDTH=%0d  OUT_WIDTH=%0d  FRAC_BITS=%0d",
                 ADC_WIDTH, TS_WIDTH, OUT_WIDTH, FRAC_BITS);
        $display("  Events: %0d directed + %0d random = %0d total",
                 N_DIRECTED, N_RANDOM, N_EVENTS);
        $display("  Gap between bursts: %0d cycle", GAP_CYCLES);
        $display("===========================================================\n");

        // --- Directed events ---

        // 0: single-sample burst
        ev[0].tag       = {32'hF00D_0000, 32'd1};
        ev[0].length    = 1;
        ev[0].range_lo  = -1000; ev[0].range_hi = 1000;
        gen_event_samples(0);
        drive_event(0);

        // 1: all zeros
        ev[1].tag    = {32'hF00D_0000, 32'd2};
        ev[1].length = 10;
        set_uniform_samples(1, 16'sd0);
        drive_event(1);

        // 2: all PEAK_MIN -> peak should remain at PEAK_MIN
        ev[2].tag    = {32'hF00D_0000, 32'd3};
        ev[2].length = 8;
        set_uniform_samples(2, 16'sh8000);
        drive_event(2);

        // 3: all PEAK_MAX -> sum near +16383
        ev[3].tag    = {32'hF00D_0000, 32'd4};
        ev[3].length = 6;
        set_uniform_samples(3, 16'sh7FFF);
        drive_event(3);

        // 4: all-negative range (peak is least-negative)
        ev[4].tag       = {32'hF00D_0000, 32'd5};
        ev[4].length    = 20;
        ev[4].range_lo  = -100; ev[4].range_hi = -1;
        gen_event_samples(4);
        drive_event(4);

        // 5: long mid-range mixed-sign
        ev[5].tag       = {32'hF00D_0000, 32'd6};
        ev[5].length    = 50;
        ev[5].range_lo  = -5000; ev[5].range_hi = 5000;
        gen_event_samples(5);
        drive_event(5);

        // --- Random events ---
        for (int i = N_DIRECTED; i < N_EVENTS; i++) begin
            ev[i].tag    = {32'hF00D_0000, 32'(i+1)};
            ev[i].length = 1 + rand_in_range(0, MAX_BURST_LEN - 1);
            case (i % 4)
                0: begin ev[i].range_lo = -1000;  ev[i].range_hi =  1000;  end
                1: begin ev[i].range_lo = -10000; ev[i].range_hi =  10000; end
                2: begin ev[i].range_lo = -32768; ev[i].range_hi =  32767; end
                3: begin ev[i].range_lo = -200;   ev[i].range_hi =  -1;    end
            endcase
            gen_event_samples(i);
            drive_event(i);
        end

        $display("  Drive done at t=%0t", $time);
        repeat (50) @(posedge clk);
        drive_done = 1;
    end

    // =================================================================
    // CAPTURE PROCESS
    // =================================================================
    initial begin
        int idx;
        @(negedge rst);
        @(posedge clk);
        forever begin
            @(posedge clk);
            if (out_valid) begin
                idx = find_ev(out_timestamp);
                if (idx >= 0 && !ev[idx].got_result) begin
                    ev[idx].got_result = 1;
                    ev[idx].dut_sum    = out_sum;
                    ev[idx].dut_tag    = out_timestamp;
                end
                n_valid_pulses++;
            end
            if (drive_done) break;
        end
    end

    // =================================================================
    // ANALYSIS PROCESS
    // =================================================================
    initial begin
        int pc, fc;

        wait (drive_done);
        repeat (10) @(posedge clk);

        pc = 0; fc = 0;

        $display("\n===========================================================");
        $display(" PER-EVENT RESULTS (failures only)");
        $display("===========================================================");

        for (int i = 0; i < N_EVENTS; i++) begin
            if (!ev[i].got_result) begin
                fc++;
                $display("  ev%0d: NO out_valid pulse received (length=%0d)",
                         i, ev[i].length);
            end else if (ev[i].dut_tag !== ev[i].tag) begin
                fc++;
                $display("  ev%0d: tag mismatch got=%h expected=%h",
                         i, ev[i].dut_tag, ev[i].tag);
            end else if (ev[i].dut_sum !== ev[i].exp_sum) begin
                fc++;
                $display("  ev%0d: sum mismatch got=%0d expected=%0d  peaks=(%0d,%0d,%0d,%0d) length=%0d",
                         i, ev[i].dut_sum, ev[i].exp_sum,
                         ev[i].peak0, ev[i].peak1, ev[i].peak2, ev[i].peak3,
                         ev[i].length);
            end else begin
                pc++;
            end
        end

        if (n_valid_pulses != N_EVENTS) begin
            fc++;
            $display("  WARNING: out_valid pulse count mismatch (got=%0d expected=%0d)",
                     n_valid_pulses, N_EVENTS);
        end

        $display("\n===========================================================");
        $display(" CHECKS: %0d passed, %0d failed", pc, fc);
        if (fc == 0) $display(" ALL CHECKS PASSED");
        else         $display(" *** FAILURES DETECTED ***");
        $display("===========================================================");

        $display("\n===========================================================");
        $display(" EVENT SUMMARY");
        $display("===========================================================");
        $display("  Total events:      %0d", N_EVENTS);
        $display("  out_valid pulses:  %0d", n_valid_pulses);

        $display("\n  Sim time: %0t", $time);
        $display("===========================================================\n");
        $finish;
    end

    initial begin #5_000_000; $error("TIMEOUT"); $finish; end

endmodule
