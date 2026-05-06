///////////////////////////////////////////////////////////////////////////////////////////////////
// Company: SwRI/VT
//
// File: uart_config_fsm.v
// File history:
//
//
// Description:
//
// FSM that accepts a 151-bit packed word (already assembled by the UART module) and
// latches it into individual DSP-pipeline configuration registers.
//
// Packed bit layout (bit 150 is MSB, received first):
//
//  Bits        Width  Field
//  ---------   -----  --------------------------------------------------
//  [150]         1    start_stop     - acquisition gate (1 = run)
//  [149]         1    sel            - mode/source select
//  [148:135]    14    Attenuation    - Q0.13 signed
//  [134:128]     7    Delay          - Unsigned integer (1-127)
//  [127:112]    16    Threshold      - SQ12.3 signed
//  [111:104]     8    ZC Neg Samples - Unsigned integer
//  [103: 84]    20    Kx             - UQ1.19 unsigned
//  [ 83: 64]    20    Ky             - UQ1.19 unsigned
//  [ 63:  0]    64    Timestamp      - 64-bit unsigned
//  ---------   -----  --------------------------------------------------
//                151 bits total
//
// Operation:
//   1. S_IDLE   - waits for rx_valid (indicates the 151-bit word is ready)
//   2. S_LATCH  - slices the packed word into individual registers
//   3. S_DONE   - asserts cfg_valid for one cycle, then returns to S_IDLE
//
// Latency: 2 cycles from rx_valid to cfg_valid.
//
// Targeted device: <Family::ProASIC3E> <Die::A3PE1500> <Package::208 PQFP>
// Author: VT MDE S26-23
//
///////////////////////////////////////////////////////////////////////////////////////////////////

`timescale 1ns / 1ps

module uart_config_fsm (
    input  wire          clk,
    input  wire          rst,

    // packed input from UART assembler, valid on a pulse of rx_valid
    input  wire [150:0]  rx_packed,
    input  wire          rx_valid,        // single-cycle pulse: rx_packed is valid

    // DSP parameter outputs, all updated together on cfg_valid
    output reg  [13:0]   attenuation,     // Q0.13 signed
    output reg  [ 6:0]   delay,           // unsigned, 1-127
    output reg  [15:0]   threshold,       // SQ12.3 signed
    output reg  [ 7:0]   zc_neg_samples,  // unsigned
    output reg  [19:0]   kx,              // UQ1.19 unsigned
    output reg  [19:0]   ky,              // UQ1.19 unsigned
    output reg  [63:0]   timestamp,       // 64-bit unsigned
    output reg           sel,             // mode/source select
    output reg           start_stop,      // acquisition gate (drives `acquire` downstream)
    output reg           cfg_valid        // one-cycle strobe: outputs above are fresh
);

    // FSM states
    localparam [1:0] S_IDLE  = 2'b00,   // waiting for a new packed word
                     S_LATCH = 2'b01,   // slicing rx_packed into config registers
                     S_DONE  = 2'b10;   // asserting cfg_valid for one cycle

    reg [1:0] state, state_next;


    // state register
    always @(posedge clk or posedge rst) begin
        if (rst)
            state <= S_IDLE;
        else
            state <= state_next;
    end


    // next-state logic: linear walk through IDLE -> LATCH -> DONE -> IDLE
    always @(*) begin
        state_next = state;
        case (state)
            S_IDLE:  if (rx_valid) state_next = S_LATCH;
            S_LATCH: state_next = S_DONE;
            S_DONE:  state_next = S_IDLE;
            default: state_next = S_IDLE;
        endcase
    end


    // output / datapath logic
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            // reset values to 0 on reset
            sel            <= 1'b0;
            start_stop     <= 1'b0;
            attenuation    <= 14'd0;
            delay          <=  7'd0;
            threshold      <= 16'd0;
            zc_neg_samples <=  8'd0;
            kx             <= 20'd0;
            ky             <= 20'd0;
            timestamp      <= 64'd0;
            cfg_valid      <=  1'b0;
        end else begin
            // default: cfg_valid is a one-cycle strobe, so deassert every cycle
            // and let S_DONE pulse it high
            cfg_valid <= 1'b0;

            case (state)
                S_LATCH: begin
                    // slice the packed word into named config registers
                    // (see bit layout table in the file header)
                    start_stop     <= rx_packed[150];
                    sel            <= rx_packed[149];
                    attenuation    <= rx_packed[148:135];
                    delay          <= rx_packed[134:128];
                    threshold      <= rx_packed[127:112];
                    zc_neg_samples <= rx_packed[111:104];
                    kx             <= rx_packed[103: 84];
                    ky             <= rx_packed[ 83: 64];
                    timestamp      <= rx_packed[ 63:  0];
                end

                S_DONE: begin
                    // pulse cfg_valid for one cycle so the downstream pipeline
                    // sees all outputs update atomically
                    cfg_valid <= 1'b1;
                end

                default: ;  // S_IDLE - hold previously latched values
            endcase
        end
    end

endmodule