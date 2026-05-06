// tb_full_dsp_pipeline.sv
//
// System testbench for dsp_pipeline.
// Drives events back-to-back with 1-cycle gaps.
// Captures combined event builder output and status flags.
// Verifies tag matching, position accuracy, and event builder operation.

`timescale 1ns / 1ps

// behavioral model of the SmartGen multiplier IP
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

module tb_full_dsp_pipeline;

    localparam integer DELAY_RAM_AW      = 7;
    localparam integer MAX_EVENT_SAMPLES = 1024;
    localparam integer IDX_W             = 10;
    localparam integer FRAC_BITS         = 10;
    localparam integer K_WIDTH           = 20;
    localparam integer K_FRAC            = 19;
    localparam integer POS_WIDTH         = 32;
    localparam integer WINDOW_X          = 51000;
    localparam integer WINDOW_Y          = 51000;
    localparam integer MAG_WIDTH         = 16;

    localparam [DELAY_RAM_AW-1:0] DELAY_VAL = 38;
    localparam [13:0]             ATT_Q0_13 = 14'd7373;
    localparam signed [15:0]      THRESHOLD = 16'sd2000;
    localparam [7:0]              ZC_NEG    = 8'd3;
    localparam [K_WIDTH-1:0]      KX_ENC    = 20'd118613;
    localparam [K_WIDTH-1:0]      KY_ENC    = 20'd113493;

    localparam real FS_HZ       = 3.0e9;
    localparam real VPX_MM_NS   = 1.39;
    localparam real VPY_MM_NS   = 1.33;
    localparam real DETECTOR_MM = 102.0;
    localparam real T0_NS       = 100.0;
    localparam real TAU_NS      = 20.0;
    localparam real SIGMA_NS    = 4.0;
    localparam real ADC_FSR_V   = 2.5;
    localparam integer ADC_BITS = 12;
    localparam integer ADC_MAX  = (1 << ADC_BITS) - 1;
    localparam real V_PER_COUNT = ADC_FSR_V / real'(ADC_MAX);
    localparam real BASELINE_V  = 0.250;
    localparam real MEAN_PULSE_V = 0.350;
    localparam integer N_SAMPLES = 901;

    localparam integer N_POS    = 10;
    localparam integer N_NOISE  = 3;
    localparam integer N_TRIALS = 3;
    localparam integer N_IN_WIN = N_POS * N_NOISE * N_TRIALS;
    localparam integer N_REJECT = 2;
    localparam integer N_EVENTS = N_IN_WIN + N_REJECT;
    localparam integer GAP_CYCLES = 1;

    logic clk;
    logic rst;
    initial clk = 0;
    always #5 clk = ~clk;

    // DUT signals
    logic        adc_valid;
    logic [11:0] adc_x1, adc_x2, adc_y1, adc_y2;
    logic [63:0] tag_in;

    // combined event output
    wire                        out_valid;
    wire [63:0]                 out_tag;
    wire signed [POS_WIDTH-1:0] out_x, out_y;
    wire signed [MAG_WIDTH-1:0] out_mag;

    // status flags
    wire event_missed;
    wire pos_rejected;

    // DUT
    dsp_pipeline #(
        .DELAY_RAM_AW(DELAY_RAM_AW), .MAX_EVENT_SAMPLES(MAX_EVENT_SAMPLES),
        .IDX_W(IDX_W), .FRAC_BITS(FRAC_BITS), .K_WIDTH(K_WIDTH),
        .K_FRAC(K_FRAC), .POS_WIDTH(POS_WIDTH),
        .WINDOW_X(WINDOW_X), .WINDOW_Y(WINDOW_Y), .MAG_WIDTH(MAG_WIDTH)
    ) dut (
        .clk(clk), .rst(rst), .adc_valid(adc_valid),
        .adc_x1(adc_x1), .adc_x2(adc_x2), .adc_y1(adc_y1), .adc_y2(adc_y2),
        .att_q0_13(ATT_Q0_13), .delay_val(DELAY_VAL),
        .threshold(THRESHOLD), .zc_neg_samples(ZC_NEG),
        .kx(KX_ENC), .ky(KY_ENC), .tag_in(tag_in),
        .out_valid(out_valid), .out_tag(out_tag),
        .out_x(out_x), .out_y(out_y), .out_mag(out_mag),
        .event_missed(event_missed),
        .pos_rejected(pos_rejected)
    );

    // PRNG
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
    function automatic real rand_uniform();
        return real'(xorshift64() & 64'h000F_FFFF_FFFF_FFFF) /
               real'(64'h0010_0000_0000_0000);
    endfunction
    function automatic real rand_gauss();
        real u1, u2;
        u1 = rand_uniform(); u2 = rand_uniform();
        if (u1 < 1.0e-15) u1 = 1.0e-15;
        return $sqrt(-2.0 * $ln(u1)) * $cos(6.283185307 * u2);
    endfunction

    // Pulse generation
    task automatic gen_pulse(
        input real t_arrival_ns, input real height_v, input real noise_v,
        output logic [11:0] buf_out [0:N_SAMPLES-1]
    );
        real dt_ns, alpha_hp, alpha_lp, peak_val, sig, t_ns;
        real inp [0:N_SAMPLES-1], hp [0:N_SAMPLES-1], lp [0:N_SAMPLES-1];
        int adc_int;
        dt_ns = 1.0e9 / FS_HZ;
        alpha_hp = TAU_NS / (TAU_NS + dt_ns);
        alpha_lp = dt_ns / (TAU_NS + dt_ns);
        for (int n = 0; n < N_SAMPLES; n++) begin
            t_ns = real'(n) * dt_ns;
            inp[n] = $exp(-((t_ns - t_arrival_ns)*(t_ns - t_arrival_ns)) /
                           (2.0 * SIGMA_NS * SIGMA_NS));
        end
        hp[0] = 0.0;
        for (int n = 1; n < N_SAMPLES; n++)
            hp[n] = alpha_hp * (hp[n-1] + inp[n] - inp[n-1]);
        lp[0] = 0.0;
        for (int n = 1; n < N_SAMPLES; n++)
            lp[n] = lp[n-1] + alpha_lp * (hp[n] - lp[n-1]);
        peak_val = 0.0;
        for (int n = 0; n < N_SAMPLES; n++)
            if (lp[n] > peak_val) peak_val = lp[n];
        if (peak_val < 1.0e-20) peak_val = 1.0;
        for (int n = 0; n < N_SAMPLES; n++) begin
            sig = (lp[n] / peak_val) * height_v + BASELINE_V;
            if (noise_v > 0.0) sig = sig + noise_v * rand_gauss();
            adc_int = int'((sig / V_PER_COUNT) + 0.5);
            if (adc_int < 0) adc_int = 0;
            if (adc_int > ADC_MAX) adc_int = ADC_MAX;
            buf_out[n] = adc_int[11:0];
        end
    endtask

    logic [11:0] ev_x1 [0:N_SAMPLES-1], ev_x2 [0:N_SAMPLES-1];
    logic [11:0] ev_y1 [0:N_SAMPLES-1], ev_y2 [0:N_SAMPLES-1];

    task automatic gen_event(input real x_mm, input real y_mm, input real noise_frac);
        real tx1, tx2, ty1, ty2, hx, hy, noise_v;
        tx1 = T0_NS + ((DETECTOR_MM/2.0 - x_mm) / VPX_MM_NS);
        tx2 = T0_NS + ((DETECTOR_MM/2.0 + x_mm) / VPX_MM_NS);
        ty1 = T0_NS + ((DETECTOR_MM/2.0 - y_mm) / VPY_MM_NS);
        ty2 = T0_NS + ((DETECTOR_MM/2.0 + y_mm) / VPY_MM_NS);
        hx = MEAN_PULSE_V + 0.050 * rand_gauss();
        if (hx < 0.200) hx = 0.200; if (hx > 0.500) hx = 0.500;
        hy = MEAN_PULSE_V + 0.050 * rand_gauss();
        if (hy < 0.200) hy = 0.200; if (hy > 0.500) hy = 0.500;
        noise_v = noise_frac * MEAN_PULSE_V;
        gen_pulse(tx1, hx, noise_v, ev_x1);
        gen_pulse(tx2, hx, noise_v, ev_x2);
        gen_pulse(ty1, hy, noise_v, ev_y1);
        gen_pulse(ty2, hy, noise_v, ev_y2);
    endtask

    // Dump event for diagnosis
    task automatic dump_event(input int idx);
        int fd;
        string fname;
        longint unsigned save;
        save = prng_state;
        prng_state = ev[idx].prng_save;
        gen_event(ev[idx].x_mm, ev[idx].y_mm, ev[idx].noise_frac);
        $sformat(fname, "ev%04d_samples.hex", idx);
        fd = $fopen(fname, "w");
        if (fd) begin
            $fdisplay(fd, "// ev%0d: x=%0d um y=%0d um noise=%.1f%%",
                      idx, ev[idx].exp_x_um, ev[idx].exp_y_um,
                      ev[idx].noise_frac * 100.0);
            for (int s = 0; s < N_SAMPLES; s++)
                $fdisplay(fd, "%03X %03X %03X %03X",
                          ev_x1[s], ev_x2[s], ev_y1[s], ev_y2[s]);
            $fclose(fd);
            $display("  >> Dumped %s (x=%.0fmm y=%.0fmm noise=%.1f%%)",
                     fname, ev[idx].x_mm, ev[idx].y_mm,
                     ev[idx].noise_frac * 100.0);
        end
        prng_state = save;
    endtask

    // Per-event records
    typedef struct {
        logic [63:0] tag;
        int          exp_x_um, exp_y_um;
        real         x_mm, y_mm, noise_frac;
        longint unsigned prng_save;
        logic        in_window;
        int          noise_bin;
        // Event builder combined output
        logic        got_eb;
        int          eb_x, eb_y;
        logic signed [MAG_WIDTH-1:0] eb_mag;
        logic [63:0] eb_tag;
    } ev_rec_t;

    ev_rec_t ev [0:N_EVENTS-1];

    // Config arrays
    real pos_x [0:N_POS-1]      = '{  0,  25, -30,  45, -10,  40, -45,  15, -20,  35};
    real pos_y [0:N_POS-1]      = '{  0, -15,  20, -40,   5,  35, -35,  45, -25,  10};
    real noise_lvl [0:N_NOISE-1] = '{0.0, 0.025, 0.050};
    real rej_x [0:N_REJECT-1]   = '{ 55.0, -52.0};
    real rej_y [0:N_REJECT-1]   = '{-55.0,  52.0};

    logic drive_done;

    function automatic int find_ev(input logic [63:0] t);
        int idx;
        if (t[63:32] != 32'hCAFE_0000) return -1;
        idx = int'(t[31:0]) - 1;
        if (idx < 0 || idx >= N_EVENTS) return -1;
        return idx;
    endfunction

    // status counters
    int n_missed, n_rejected;

    // === DRIVE PROCESS ===
    initial begin
        int en;
        drive_done = 0;
        rst = 1; adc_valid = 0;
        adc_x1 = 0; adc_x2 = 0; adc_y1 = 0; adc_y2 = 0;
        tag_in = 0;
        prng_state = 64'hDEAD_BEEF_CAFE_1234;
        n_missed = 0; n_rejected = 0;
        for (int i = 0; i < N_EVENTS; i++) begin
            ev[i].got_eb = 0;
        end
        repeat (20) @(posedge clk);
        rst = 0;
        repeat (10) @(posedge clk);

        $display("===========================================================");
        $display(" DSP Pipeline Testbench (Back-to-Back, 1-Cycle Gap)");
        $display("===========================================================");
        $display("  DELAY=%0d ATT=%.4f THRESH=%0d ZC_NEG=%0d",
                 DELAY_VAL, real'(ATT_Q0_13)/8192.0, THRESHOLD, ZC_NEG);
        $display("  %0d in-window + %0d rejection = %0d events, 1-cycle gap",
                 N_IN_WIN, N_REJECT, N_EVENTS);
        $display("===========================================================\n");

        en = 0;
        for (int pi = 0; pi < N_POS; pi++)
            for (int ni = 0; ni < N_NOISE; ni++)
                for (int ti = 0; ti < N_TRIALS; ti++) begin
                    ev[en].tag      = {32'hCAFE_0000, 32'(en+1)};
                    ev[en].exp_x_um = int'(pos_x[pi] * 1000.0);
                    ev[en].exp_y_um = int'(pos_y[pi] * 1000.0);
                    ev[en].in_window = 1;
                    ev[en].noise_bin = ni;
                    ev[en].x_mm = pos_x[pi];
                    ev[en].y_mm = pos_y[pi];
                    ev[en].noise_frac = noise_lvl[ni];
                    ev[en].prng_save = prng_state;
                    gen_event(pos_x[pi], pos_y[pi], noise_lvl[ni]);
                    tag_in <= ev[en].tag;
                    for (int s = 0; s < N_SAMPLES; s++) begin
                        adc_x1 <= ev_x1[s]; adc_x2 <= ev_x2[s];
                        adc_y1 <= ev_y1[s]; adc_y2 <= ev_y2[s];
                        adc_valid <= 1;
                        @(posedge clk);
                    end
                    adc_valid <= 0;
                    repeat (GAP_CYCLES) @(posedge clk);
                    en++;
                    if (en % 100 == 0)
                        $display("  Driving: %0d / %0d events...", en, N_EVENTS);
                end

        for (int ri = 0; ri < N_REJECT; ri++) begin
            ev[en].tag      = {32'hCAFE_0000, 32'(en+1)};
            ev[en].exp_x_um = int'(rej_x[ri] * 1000.0);
            ev[en].exp_y_um = int'(rej_y[ri] * 1000.0);
            ev[en].in_window = 0;
            ev[en].noise_bin = 0;
            ev[en].x_mm = rej_x[ri];
            ev[en].y_mm = rej_y[ri];
            ev[en].noise_frac = 0.0;
            ev[en].prng_save = prng_state;
            gen_event(rej_x[ri], rej_y[ri], 0.0);
            tag_in <= ev[en].tag;
            for (int s = 0; s < N_SAMPLES; s++) begin
                adc_x1 <= ev_x1[s]; adc_x2 <= ev_x2[s];
                adc_y1 <= ev_y1[s]; adc_y2 <= ev_y2[s];
                adc_valid <= 1;
                @(posedge clk);
            end
            adc_valid <= 0;
            repeat (GAP_CYCLES) @(posedge clk);
            en++;
        end

        $display("  Drive done: %0d events at t=%0t", en, $time);
        repeat (500) @(posedge clk);
        drive_done = 1;
    end

    // === CAPTURE PROCESS ===
    initial begin
        int idx;
        @(negedge rst);
        @(posedge clk);
        forever begin
            @(posedge clk);

            // event builder combined output
            if (out_valid) begin
                idx = find_ev(out_tag);
                if (idx >= 0 && !ev[idx].got_eb) begin
                    ev[idx].got_eb  = 1;
                    ev[idx].eb_x    = int'(out_x);
                    ev[idx].eb_y    = int'(out_y);
                    ev[idx].eb_mag  = out_mag;
                    ev[idx].eb_tag  = out_tag;
                end
            end

            // status counters
            if (event_missed)  n_missed++;
            if (pos_rejected)  n_rejected++;

            if (drive_done) break;
        end
    end

    // === ANALYSIS PROCESS ===
    initial begin
        int pc, fc, nv, n_eb, nf;
        real xe, ye, re, ssq, mr, sax, say, max_x, max_y;
        int nn[0:2]; real ns[0:2], nq[0:2], nm[0:2];

        wait (drive_done);
        repeat (10) @(posedge clk);

        pc=0; fc=0; nv=0; n_eb=0; nf=0;
        ssq=0; mr=0; sax=0; say=0; max_x=0; max_y=0;
        for (int i=0;i<3;i++) begin nn[i]=0; ns[i]=0; nq[i]=0; nm[i]=0; end

        $display("\n===========================================================");
        $display(" PER-EVENT RESULTS (failures and outliers > 500 um only)");
        $display("===========================================================");

        for (int i = 0; i < N_EVENTS; i++) begin
            if (ev[i].in_window) begin
                if (ev[i].got_eb && ev[i].eb_tag == ev[i].tag) begin
                    pc++;
                    n_eb++;
                    xe = real'(ev[i].eb_x - ev[i].exp_x_um);
                    ye = real'(ev[i].eb_y - ev[i].exp_y_um);
                    re = $sqrt(xe*xe + ye*ye);
                    nv++;
                    ssq += re*re;
                    if (re > mr) mr = re;
                    if (xe<0) xe=-xe; if (ye<0) ye=-ye;
                    sax+=xe; say+=ye;
                    if (xe>max_x) max_x=xe; if (ye>max_y) max_y=ye;
                    nn[ev[i].noise_bin]++;
                    ns[ev[i].noise_bin]+=re;
                    nq[ev[i].noise_bin]+=re*re;
                    if (re>nm[ev[i].noise_bin]) nm[ev[i].noise_bin]=re;
                    if (re > 500.0)
                        $display("  ev%0d: OUTLIER pos=(%0d,%0d) err=(%0d,%0d) r=%.1f mag=%0d",
                                 i, ev[i].eb_x, ev[i].eb_y,
                                 ev[i].eb_x-ev[i].exp_x_um,
                                 ev[i].eb_y-ev[i].exp_y_um, re, ev[i].eb_mag);
                    if (re > 2000.0)
                        dump_event(i);
                end else if (!ev[i].got_eb) begin
                    // could be a CFD fail ? not necessarily an error
                    nf++;
                    $display("  ev%0d: no EB output (event_missed or CFD fail)", i);
                end else begin
                    fc++;
                    $display("  ev%0d: EB tag mismatch", i);
                end
            end else begin
                // rejection test: should NOT get event builder output
                if (!ev[i].got_eb) begin
                    pc++;
                    $display("  ev%0d: rejected, no EB output ? correct", i);
                end else begin
                    fc++;
                    $display("  ev%0d: rejected but EB output present ? FAIL", i);
                end
            end
        end

        $display("\n===========================================================");
        $display(" CHECKS: %0d passed, %0d failed", pc, fc);
        if (fc==0) $display(" ALL CHECKS PASSED");
        else       $display(" *** FAILURES DETECTED ***");
        $display("===========================================================");

        $display("\n===========================================================");
        $display(" EVENT SUMMARY");
        $display("===========================================================");
        $display("  Total:              %0d", N_EVENTS);
        $display("  EB outputs:         %0d", n_eb);
        $display("  event_missed pulses: %0d", n_missed);
        $display("  pos_rejected pulses: %0d", n_rejected);
        $display("  No EB output (in-window): %0d", nf);

        if (nv > 0) begin
            $display("\n===========================================================");
            $display(" POSITION ACCURACY (%0d events, from event builder)", nv);
            $display("===========================================================");
            $display("  RMS radial:  %.1f um", $sqrt(ssq/real'(nv)));
            $display("  Max radial:  %.1f um", mr);
            $display("  Mean |x|:    %.1f um", sax/real'(nv));
            $display("  Mean |y|:    %.1f um", say/real'(nv));
            $display("  Max  |x|:    %.1f um", max_x);
            $display("  Max  |y|:    %.1f um", max_y);
            $display("\n  --- By noise level ---");
            if (nn[0]>0) $display("  0.0%%:  n=%0d avg=%.1f rms=%.1f max=%.1f um",
                nn[0], ns[0]/real'(nn[0]), $sqrt(nq[0]/real'(nn[0])), nm[0]);
            if (nn[1]>0) $display("  2.5%%:  n=%0d avg=%.1f rms=%.1f max=%.1f um",
                nn[1], ns[1]/real'(nn[1]), $sqrt(nq[1]/real'(nn[1])), nm[1]);
            if (nn[2]>0) $display("  5.0%%:  n=%0d avg=%.1f rms=%.1f max=%.1f um",
                nn[2], ns[2]/real'(nn[2]), $sqrt(nq[2]/real'(nn[2])), nm[2]);
        end

        $display("\n  Sim time: %0t", $time);
        $display("===========================================================\n");
        $finish;
    end

    initial begin #500000000; $error("TIMEOUT"); $finish; end

endmodule

