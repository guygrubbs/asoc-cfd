///////////////////////////////////////////////////////////////////////////////////////////////////
// Company: SwRI/VT
//
// File: pulse_extender.v
// File history:
//
//
// Description:
//
// Stretches a short input pulse into a fixed-width output hold signal.
//
// On a detected rising edge of i_trig, o_hold is asserted and a down-counter is loaded
// with HOLD_CYCLES - 1. o_hold stays high until the counter reaches zero, at which point
// it is deasserted. Re-triggering during an active hold reloads the counter, extending
// the pulse rather than stacking it.
//
// Useful for making brief events (e.g. event_valid strobes) visible on slower
// downstream consumers such as LEDs, scope triggers, or status registers.
//
// Targeted device: <Family::ProASIC3E> <Die::A3PE1500> <Package::208 PQFP>
// Author: VT MDE S26-23
//
///////////////////////////////////////////////////////////////////////////////////////////////////

`timescale 1ns/1ps

module pulse_extender
#(
    parameter integer HOLD_CYCLES = 40000000   // hold duration in clock cycles
)
(
    input  wire i_clk,
    input  wire i_rst,
    input  wire i_trig,    // pulse input (any width >= 1 cycle)
    output reg  o_hold     // stretched output, high for HOLD_CYCLES after a rising edge
);

    // ceil(log2(value)); used to size the down-counter
    function integer clog2;
        input integer value;
        integer v;
        begin
            v = value - 1;
            clog2 = 0;
            while (v > 0) begin
                v = v >> 1;
                clog2 = clog2 + 1;
            end
        end
    endfunction

    // calculated bit lengths
    localparam integer CNT_W = clog2(HOLD_CYCLES + 1);   // width needed to hold HOLD_CYCLES

    reg [CNT_W-1:0] r_count;    // remaining cycles in current hold
    reg             r_trig_d;   // i_trig delayed by 1 cycle, for edge detection

    // rising-edge detect on i_trig
    wire w_rise;
    assign w_rise = i_trig & ~r_trig_d;

    always @(posedge i_clk or posedge i_rst) begin
        if (i_rst) begin
            // reset values to 0 on reset
            r_count  <= {CNT_W{1'b0}};
            r_trig_d <= 1'b0;
            o_hold   <= 1'b0;
        end else begin
            // delay line for edge detection
            r_trig_d <= i_trig;

            if (w_rise) begin
                // new rising edge: (re)load the counter and assert the hold
                r_count <= HOLD_CYCLES - 1;
                o_hold  <= 1'b1;
            end else if (o_hold) begin
                // count down while hold is active; release when counter expires
                if (r_count == 0) begin
                    o_hold <= 1'b0;
                end else begin
                    r_count <= r_count - 1'b1;
                end
            end
        end
    end

endmodule