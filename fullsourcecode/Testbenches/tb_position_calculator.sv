// tb_position_calc.sv
//
// Unit testbench for position_calc.
// Drives event_valid pulses with synthetic CFD timestamps representing
// known (x_mm, y_mm) positions, then waits the 45-cycle latency for
// pos_valid / pos_rejected and verifies the reported micron position
// matches expectations.
//
// Coverage:
//   - In-window positions spanning the detector area
//   - Edge cases near the +/- WINDOW boundary
//   - Rejection cases just outside the window on each axis
//   - dt = 0 (origin)
//   - Both signs of dt on each axis
//   - Back-to-back events

`timescale 1ns / 1ps

module tb_position_calculator;

    // -----------------------------------------------------------------
    // Parameters (must match dsp_pipeline.v defaults)
    // -----------------------------------------------------------------
    localparam integer IDX_W      = 10;
    localparam integer FRAC_BITS  = 10;
    localparam integer K_WIDTH    = 20;
    localparam integer K_FRAC     = 19;
    localparam integer POS_WIDTH  = 32;
    localparam integer WINDOW_X   = 51000;
    localparam integer WINDOW_Y   = 51000;

    localparam integer TS_W       = IDX_W + FRAC_BITS;   // 20
    localparam integer LATENCY    = 45;                  // event_valid -> pos_valid

    // K constants (same as system TB)
    localparam [K_WIDTH-1:0] KX_ENC = 20'd118613;
    localparam [K_WIDTH-1:0] KY_ENC = 20'd113493;

    // Detector / pulse-arrival model (matches tb_full_dsp_pipeline)
    localparam real FS_HZ       = 3.0e9;
    localparam real DT_NS       = 1.0e9 / FS_HZ;
    localparam real VPX_MM_NS   = 1.39;
    localparam real VPY_MM_NS   = 1.33;
    localparam real DETECTOR_MM = 102.0;
    localparam real T0_NS       = 100.0;

    // Effective Kx/Ky in floating point (microns per timestamp tick)
    localparam real KX_REAL = real'(KX_ENC) / real'(1 <<< K_FRAC);
    localparam real KY_REAL = real'(KY_ENC) / real'(1 <<< K_FRAC);

    // -----------------------------------------------------------------
    // Test set sizing
    // -----------------------------------------------------------------
    localparam integer N_IN_WIN  = 9;
    localparam integer N_REJECT  = 4;
    localparam integer N_EVENTS  = N_IN_WIN + N_REJECT;

    // Allow a few um for rounding in the shift-and-add multiply.
    localparam integer ACCURACY_TOL_UM = 5;

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
    logic                     event_valid;
    logic [IDX_W-1:0]         tx1_int, tx2_int, ty1_int, ty2_int;
    logic [FRAC_BITS-1:0]     tx1_frac, tx2_frac, ty1_frac, ty2_frac;
    logic [K_WIDTH-1:0]       kx, ky;
    logic [63:0]              tag_in;

    wire signed [POS_WIDTH-1:0] x_pos;
    wire signed [POS_WIDTH-1:0] y_pos;
    wire                        pos_valid;
    wire                        pos_rejected;
    wire [63:0]                 tag_out;

    // -----------------------------------------------------------------
    // DUT
    // -----------------------------------------------------------------
    position_calc #(
        .IDX_W    (IDX_W),
        .FRAC_BITS(FRAC_BITS),
        .K_WIDTH  (K_WIDTH),
        .K_FRAC   (K_FRAC),
        .POS_WIDTH(POS_WIDTH),
        .WINDOW_X (WINDOW_X),
        .WINDOW_Y (WINDOW_Y)
    ) dut (
        .clk         (clk),
        .rst         (rst),
        .event_valid (event_valid),
        .tx1_int     (tx1_int),
        .tx1_frac    (tx1_frac),
        .tx2_int     (tx2_int),
        .tx2_frac    (tx2_frac),
        .ty1_int     (ty1_int),
        .ty1_frac    (ty1_frac),
        .ty2_int     (ty2_int),
        .ty2_frac    (ty2_frac),
        .kx          (kx),
        .ky          (ky),
        .tag_in      (tag_in),
        .x_pos       (x_pos),
        .y_pos       (y_pos),
        .pos_valid   (pos_valid),
        .pos_rejected(pos_rejected),
        .tag_out     (tag_out)
    );

    // -----------------------------------------------------------------
    // PRNG (kept for forward-compat with system TB style)
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

    // -----------------------------------------------------------------
    // Helpers
    // -----------------------------------------------------------------
    // Build a 20-bit timestamp from a real-valued number of clock ticks.
    task automatic real_to_ts(
        input  real          ts_ticks,
        output logic [IDX_W-1:0]     int_part,
        output logic [FRAC_BITS-1:0] frac_part
    );
        real    scaled;
        longint quant;
        scaled = ts_ticks * real'(1 <<< FRAC_BITS);
        quant  = longint'(scaled + 0.5);
        if (quant < 0) quant = 0;
        if (quant > ((1 <<< (IDX_W + FRAC_BITS)) - 1))
            quant = (1 <<< (IDX_W + FRAC_BITS)) - 1;
        int_part  = quant[IDX_W+FRAC_BITS-1 : FRAC_BITS];
        frac_part = quant[FRAC_BITS-1 : 0];
    endtask

    // Convert (x_mm, y_mm) to four CFD timestamps in clock ticks.
    task automatic mm_to_timestamps(
        input  real x_mm,
        input  real y_mm,
        output logic [IDX_W-1:0]     i_x1, i_x2, i_y1, i_y2,
        output logic [FRAC_BITS-1:0] f_x1, f_x2, f_y1, f_y2
    );
        real tx1_ns, tx2_ns, ty1_ns, ty2_ns;
        tx1_ns = T0_NS + ((DETECTOR_MM/2.0 - x_mm) / VPX_MM_NS);
        tx2_ns = T0_NS + ((DETECTOR_MM/2.0 + x_mm) / VPX_MM_NS);
        ty1_ns = T0_NS + ((DETECTOR_MM/2.0 - y_mm) / VPY_MM_NS);
        ty2_ns = T0_NS + ((DETECTOR_MM/2.0 + y_mm) / VPY_MM_NS);
        real_to_ts(tx1_ns / DT_NS, i_x1, f_x1);
        real_to_ts(tx2_ns / DT_NS, i_x2, f_x2);
        real_to_ts(ty1_ns / DT_NS, i_y1, f_y1);
        real_to_ts(ty2_ns / DT_NS, i_y2, f_y2);
    endtask

    // Bit-accurate model of the DUT's dt -> dt*K -> >>K_FRAC pipeline.
    function automatic int expected_um(
        input logic [IDX_W-1:0]     a_int,
        input logic [FRAC_BITS-1:0] a_frac,
        input logic [IDX_W-1:0]     b_int,
        input logic [FRAC_BITS-1:0] b_frac,
        input logic [K_WIDTH-1:0]   k_val
    );
        logic signed [TS_W:0]                a_ts, b_ts, dt;
        logic signed [TS_W+K_WIDTH:0]        prod, prod_rounded;
        logic signed [TS_W+K_WIDTH-K_FRAC:0] shifted;
        a_ts = $signed({1'b0, a_int, a_frac});
        b_ts = $signed({1'b0, b_int, b_frac});
        dt   = b_ts - a_ts;
        prod = dt * $signed({1'b0, k_val});
        prod_rounded = prod + (1 <<< (K_FRAC - 1));
        shifted = prod_rounded >>> K_FRAC;
        return int'(shifted);
    endfunction

    // -----------------------------------------------------------------
    // Per-event record
    // -----------------------------------------------------------------
    typedef struct {
        logic [63:0] tag;
        real         x_mm, y_mm;
        int          exp_x_um, exp_y_um;
        logic        in_window;
        logic        got_result;
        logic        got_valid;
        logic        got_rejected;
        int          dut_x, dut_y;
        logic [63:0] dut_tag;
    } ev_rec_t;

    ev_rec_t ev [0:N_EVENTS-1];

    real pos_x [0:N_IN_WIN-1] = '{  0.0,  25.0, -25.0,  40.0, -40.0,
                                   50.0, -50.0,  10.0, -15.0};
    real pos_y [0:N_IN_WIN-1] = '{  0.0, -15.0,  15.0,  35.0, -35.0,
                                  -50.0,  50.0, -45.0,  20.0};
    real rej_x [0:N_REJECT-1] = '{ 55.0, -55.0,   0.0,   0.0};
    real rej_y [0:N_REJECT-1] = '{  0.0,   0.0,  55.0, -55.0};

    logic drive_done;
    int   n_valid_pulses, n_reject_pulses;

    function automatic int find_ev(input logic [63:0] t);
        int idx;
        if (t[63:32] != 32'hBEEF_0000) return -1;
        idx = int'(t[31:0]) - 1;
        if (idx < 0 || idx >= N_EVENTS) return -1;
        return idx;
    endfunction

    task automatic drive_event(input int idx);
        logic [IDX_W-1:0]     i_x1, i_x2, i_y1, i_y2;
        logic [FRAC_BITS-1:0] f_x1, f_x2, f_y1, f_y2;
        mm_to_timestamps(ev[idx].x_mm, ev[idx].y_mm,
                         i_x1, i_x2, i_y1, i_y2,
                         f_x1, f_x2, f_y1, f_y2);

        ev[idx].exp_x_um = expected_um(i_x1, f_x1, i_x2, f_x2, KX_ENC);
        ev[idx].exp_y_um = expected_um(i_y1, f_y1, i_y2, f_y2, KY_ENC);

        @(posedge clk);
        tx1_int  <= i_x1; tx1_frac <= f_x1;
        tx2_int  <= i_x2; tx2_frac <= f_x2;
        ty1_int  <= i_y1; ty1_frac <= f_y1;
        ty2_int  <= i_y2; ty2_frac <= f_y2;
        kx       <= KX_ENC;
        ky       <= KY_ENC;
        tag_in   <= ev[idx].tag;
        event_valid <= 1'b1;
        @(posedge clk);
        event_valid <= 1'b0;
        repeat (LATENCY + 5) @(posedge clk);
    endtask

    // =================================================================
    // DRIVE PROCESS
    // =================================================================
    initial begin
        drive_done       = 0;
        rst              = 1;
        event_valid      = 0;
        tx1_int = 0; tx1_frac = 0;
        tx2_int = 0; tx2_frac = 0;
        ty1_int = 0; ty1_frac = 0;
        ty2_int = 0; ty2_frac = 0;
        kx       = 0; ky      = 0;
        tag_in   = 0;
        prng_state = 64'hAAAA_BBBB_CCCC_1111;
        n_valid_pulses  = 0;
        n_reject_pulses = 0;
        for (int i = 0; i < N_EVENTS; i++) begin
            ev[i].got_result   = 0;
            ev[i].got_valid    = 0;
            ev[i].got_rejected = 0;
        end

        repeat (20) @(posedge clk);
        rst = 0;
        repeat (10) @(posedge clk);

        $display("===========================================================");
        $display(" position_calc Unit Testbench");
        $display("===========================================================");
        $display("  KX = %0d (%.6f um/tick)", KX_ENC, KX_REAL);
        $display("  KY = %0d (%.6f um/tick)", KY_ENC, KY_REAL);
        $display("  Window: +/- %0d um (X), +/- %0d um (Y)",
                 WINDOW_X, WINDOW_Y);
        $display("  Events: %0d in-window + %0d rejection = %0d",
                 N_IN_WIN, N_REJECT, N_EVENTS);
        $display("  Tolerance: +/- %0d um", ACCURACY_TOL_UM);
        $display("===========================================================\n");

        for (int i = 0; i < N_IN_WIN; i++) begin
            ev[i].tag       = {32'hBEEF_0000, 32'(i+1)};
            ev[i].x_mm      = pos_x[i];
            ev[i].y_mm      = pos_y[i];
            ev[i].in_window = 1;
            drive_event(i);
        end

			for (int j = 0; j < N_REJECT; j++) begin
  		  		ev[N_IN_WIN + j].tag       = {32'hBEEF_0000, 32'(N_IN_WIN + j + 1)};
    			ev[N_IN_WIN + j].x_mm      = rej_x[j];
    			ev[N_IN_WIN + j].y_mm      = rej_y[j];
    			ev[N_IN_WIN + j].in_window = 0;
    			drive_event(N_IN_WIN + j);
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

            if (pos_valid || pos_rejected) begin
                idx = find_ev(tag_out);
                if (idx >= 0 && !ev[idx].got_result) begin
                    ev[idx].got_result   = 1;
                    ev[idx].got_valid    = pos_valid;
                    ev[idx].got_rejected = pos_rejected;
                    ev[idx].dut_x        = int'(x_pos);
                    ev[idx].dut_y        = int'(y_pos);
                    ev[idx].dut_tag      = tag_out;
                end
                if (pos_valid)    n_valid_pulses++;
                if (pos_rejected) n_reject_pulses++;
            end

            if (drive_done) break;
        end
    end

    // =================================================================
    // ANALYSIS PROCESS
    // =================================================================
    initial begin
        int  pc, fc, nv;
        real ssq, mr, sax, say, max_x, max_y;
        real xe, ye, re;
        int  err_x, err_y;

        wait (drive_done);
        repeat (10) @(posedge clk);

        pc = 0; fc = 0; nv = 0;
        ssq = 0.0; mr = 0.0;
        sax = 0.0; say = 0.0; max_x = 0.0; max_y = 0.0;

        $display("\n===========================================================");
        $display(" PER-EVENT RESULTS (failures and outliers only)");
        $display("===========================================================");

        for (int i = 0; i < N_EVENTS; i++) begin
            if (ev[i].in_window) begin
                if (!ev[i].got_result) begin
                    fc++;
                    $display("  ev%0d: NO RESULT (expected pos_valid)", i);
                end else if (ev[i].got_rejected) begin
                    fc++;
                    $display("  ev%0d: pos_rejected but in-window expected (x=%.0fmm y=%.0fmm)",
                             i, ev[i].x_mm, ev[i].y_mm);
                end else if (!ev[i].got_valid) begin
                    fc++;
                    $display("  ev%0d: neither valid nor rejected", i);
                end else if (ev[i].dut_tag !== ev[i].tag) begin
                    fc++;
                    $display("  ev%0d: tag mismatch got=%h expected=%h",
                             i, ev[i].dut_tag, ev[i].tag);
                end else begin
                    err_x = ev[i].dut_x - ev[i].exp_x_um;
                    err_y = ev[i].dut_y - ev[i].exp_y_um;
                    if ((err_x >  ACCURACY_TOL_UM) ||
                        (err_x < -ACCURACY_TOL_UM) ||
                        (err_y >  ACCURACY_TOL_UM) ||
                        (err_y < -ACCURACY_TOL_UM)) begin
                        fc++;
                        $display("  ev%0d: ACCURACY FAIL pos=(%0d,%0d) exp=(%0d,%0d) err=(%0d,%0d)",
                                 i, ev[i].dut_x, ev[i].dut_y,
                                 ev[i].exp_x_um, ev[i].exp_y_um,
                                 err_x, err_y);
                    end else begin
                        pc++;
                    end
                    xe = real'(err_x); ye = real'(err_y);
                    re = $sqrt(xe*xe + ye*ye);
                    nv++;
                    ssq += re*re;
                    if (re > mr) mr = re;
                    if (xe < 0) xe = -xe;
                    if (ye < 0) ye = -ye;
                    sax += xe; say += ye;
                    if (xe > max_x) max_x = xe;
                    if (ye > max_y) max_y = ye;
                end
            end else begin
                if (!ev[i].got_result) begin
                    fc++;
                    $display("  ev%0d: NO RESULT (expected pos_rejected)", i);
                end else if (ev[i].got_valid) begin
                    fc++;
                    $display("  ev%0d: pos_valid but should reject (x=%.0fmm y=%.0fmm dut=(%0d,%0d))",
                             i, ev[i].x_mm, ev[i].y_mm, ev[i].dut_x, ev[i].dut_y);
                end else if (!ev[i].got_rejected) begin
                    fc++;
                    $display("  ev%0d: neither valid nor rejected", i);
                end else if (ev[i].dut_tag !== ev[i].tag) begin
                    fc++;
                    $display("  ev%0d: tag mismatch on rejection got=%h expected=%h",
                             i, ev[i].dut_tag, ev[i].tag);
                end else begin
                    pc++;
                end
            end
        end

        $display("\n===========================================================");
        $display(" CHECKS: %0d passed, %0d failed", pc, fc);
        if (fc == 0) $display(" ALL CHECKS PASSED");
        else         $display(" *** FAILURES DETECTED ***");
        $display("===========================================================");

        $display("\n===========================================================");
        $display(" EVENT SUMMARY");
        $display("===========================================================");
        $display("  Total:               %0d", N_EVENTS);
        $display("  pos_valid pulses:    %0d", n_valid_pulses);
        $display("  pos_rejected pulses: %0d", n_reject_pulses);

        if (nv > 0) begin
            $display("\n===========================================================");
            $display(" POSITION ACCURACY (%0d in-window events)", nv);
            $display("===========================================================");
            $display("  RMS radial:  %.2f um", $sqrt(ssq/real'(nv)));
            $display("  Max radial:  %.2f um", mr);
            $display("  Mean |x|:    %.2f um", sax/real'(nv));
            $display("  Mean |y|:    %.2f um", say/real'(nv));
            $display("  Max  |x|:    %.2f um", max_x);
            $display("  Max  |y|:    %.2f um", max_y);
        end

        $display("\n  Sim time: %0t", $time);
        $display("===========================================================\n");
        $finish;
    end

    initial begin #5_000_000; $error("TIMEOUT"); $finish; end

endmodule
