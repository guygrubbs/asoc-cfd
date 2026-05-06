///////////////////////////////////////////////////////////////////////////////////////////////////
// Company: SwRI/VT
//
// File: uart_packet_parser.v
// File history:
//
//
// Description:
//
// Parses framed UART packets from an upstream byte stream and writes the unpacked
// samples into the event_buffer. Drives o_event_commit on a clean end-of-packet so
// the buffer can begin replaying samples to the DSP pipeline.
//
// Wire protocol:
//
//   [HEADER 0xAA] [CNT_HI] [CNT_LO] [SAMPLE_0] [SAMPLE_1] ... [SAMPLE_N-1] [FOOTER 0x55]
//
//   - 16-bit sample count is sent big-endian (high byte first).
//   - N must be in [MIN_SAMPLES, MAX_SAMPLES]; out-of-range counts raise o_proto_err.
//   - Each sample is 8 bytes covering 4 channels (x1, x2, y1, y2) at 12 bits/channel.
//     Per channel, the high nibble is sent first in the lower 4 bits of one byte,
//     followed by the full low byte:
//
//        Byte 0: 0000 | x1[11:8]      Byte 4: 0000 | y1[11:8]
//        Byte 1: x1[7:0]              Byte 5: y1[7:0]
//        Byte 2: 0000 | x2[11:8]      Byte 6: 0000 | y2[11:8]
//        Byte 3: x2[7:0]              Byte 7: y2[7:0]
//
//   - On byte 7 of each sample the assembled 12-bit values are written to the
//     event_buffer with o_wr_en, addressed by the running r_samples_written index.
//   - After the last sample, the footer byte is checked. A correct footer pulses
//     o_event_commit; a mismatch pulses o_proto_err. Either path returns to S_IDLE.
//
// Targeted device: <Family::ProASIC3E> <Die::A3PE1500> <Package::208 PQFP>
// Author: VT MDE S26-23
//
///////////////////////////////////////////////////////////////////////////////////////////////////


`timescale 1ns/1ps

module uart_packet_parser #(
    parameter integer MAX_SAMPLES = 1024,   // upper bound on samples per packet
    parameter integer MIN_SAMPLES = 1       // lower bound on samples per packet
)(
    input  wire        i_clk,
    input  wire        i_rst,

    // byte stream from UART receiver, valid on pulses of i_rx_dv
    input  wire [7:0]  i_rx_byte,
    input  wire        i_rx_dv,

    // status outputs
    output reg         o_busy,        // high while a packet is in flight
    output reg         o_proto_err,   // one-cycle pulse on header/footer/length errors

    // write side to event_buffer
    output reg         o_wr_en,
    output reg [15:0]  o_wr_addr,
    output reg [11:0]  o_wr_x1,
    output reg [11:0]  o_wr_x2,
    output reg [11:0]  o_wr_y1,
    output reg [11:0]  o_wr_y2,

    // event metadata to event_buffer
    output reg [15:0]  o_sample_count,
    output reg         o_event_commit  // one-cycle pulse: packet received cleanly
);

    // packet framing bytes
    localparam [7:0] HEADER_BYTE = 8'hAA;
    localparam [7:0] FOOTER_BYTE = 8'h55;

    // FSM states
    localparam [2:0]
        S_IDLE        = 3'd0,   // waiting for HEADER_BYTE
        S_CNT_HI      = 3'd1,   // capturing high byte of sample count
        S_CNT_LO      = 3'd2,   // capturing low byte of sample count and validating range
        S_RX_SAMPLE   = 3'd3,   // streaming 8-byte samples into the event_buffer
        S_WAIT_FOOTER = 3'd4;   // expecting FOOTER_BYTE, then commit or flag error

    reg [2:0]  r_state;
    reg [15:0] r_sample_count;       // total samples expected in this packet
    reg [15:0] r_samples_written;    // number of complete samples written so far
    reg [2:0]  r_byte_idx;           // 0..7: position within the current 8-byte sample

    // staging registers for each channel; the 12-bit value is reassembled
    // on byte 7 and written in a single cycle to the event_buffer
    reg [3:0]  r_x1_hi, r_x2_hi, r_y1_hi, r_y2_hi;
    reg [7:0]  r_x1_lo, r_x2_lo, r_y1_lo;

    // candidate sample count combining the previously latched high byte with
    // the incoming low byte; used for range-check before committing to r_sample_count
    wire [15:0] w_count_candidate = {o_sample_count[15:8], i_rx_byte};

    always @(posedge i_clk or posedge i_rst) begin
        if (i_rst) begin
            // reset values to 0 on reset
            r_state           <= S_IDLE;
            r_sample_count    <= 16'd0;
            r_samples_written <= 16'd0;
            r_byte_idx        <= 3'd0;

            r_x1_hi <= 4'd0; r_x2_hi <= 4'd0; r_y1_hi <= 4'd0; r_y2_hi <= 4'd0;
            r_x1_lo <= 8'd0; r_x2_lo <= 8'd0; r_y1_lo <= 8'd0;

            o_busy         <= 1'b0;
            o_proto_err    <= 1'b0;
            o_wr_en        <= 1'b0;
            o_wr_addr      <= 16'd0;
            o_wr_x1        <= 12'd0;
            o_wr_x2        <= 12'd0;
            o_wr_y1        <= 12'd0;
            o_wr_y2        <= 12'd0;
            o_sample_count <= 16'd0;
            o_event_commit <= 1'b0;
        end else begin
            // default: one-cycle strobes deassert each clock unless reasserted below
            o_proto_err    <= 1'b0;
            o_wr_en        <= 1'b0;
            o_event_commit <= 1'b0;

            case (r_state)
                S_IDLE: begin
                    // wait for a valid header byte to begin a new packet
                    o_busy <= 1'b0;

                    if (i_rx_dv && (i_rx_byte == HEADER_BYTE)) begin
                        o_busy            <= 1'b1;
                        o_sample_count    <= 16'd0;
                        r_sample_count    <= 16'd0;
                        r_samples_written <= 16'd0;
                        r_byte_idx        <= 3'd0;
                        r_state           <= S_CNT_HI;
                    end
                end

                S_CNT_HI: begin
                    // latch high byte of the sample count
                    if (i_rx_dv) begin
                        o_sample_count[15:8] <= i_rx_byte;
                        r_state              <= S_CNT_LO;
                    end
                end

                S_CNT_LO: begin
                    // latch low byte and validate the assembled count is in range
                    if (i_rx_dv) begin
                        o_sample_count[7:0] <= i_rx_byte;
                        r_sample_count      <= w_count_candidate;
                        r_samples_written   <= 16'd0;
                        r_byte_idx          <= 3'd0;

                        if ((w_count_candidate < MIN_SAMPLES[15:0]) ||
                            (w_count_candidate > MAX_SAMPLES[15:0])) begin
                            // out-of-range count: raise error and abort the packet
                            o_proto_err <= 1'b1;
                            o_busy      <= 1'b0;
                            r_state     <= S_IDLE;
                        end else begin
                            r_state <= S_RX_SAMPLE;
                        end
                    end
                end

                S_RX_SAMPLE: begin
                    // walk through 8 bytes per sample, capturing high nibble then low byte
                    // for x1, x2, y1, y2. On byte 7, assemble all four 12-bit values
                    // and write them to the event_buffer in a single cycle.
                    if (i_rx_dv) begin
                        case (r_byte_idx)
                            3'd0: begin
                                r_x1_hi    <= i_rx_byte[3:0];
                                r_byte_idx <= 3'd1;
                            end

                            3'd1: begin
                                r_x1_lo    <= i_rx_byte;
                                r_byte_idx <= 3'd2;
                            end

                            3'd2: begin
                                r_x2_hi    <= i_rx_byte[3:0];
                                r_byte_idx <= 3'd3;
                            end

                            3'd3: begin
                                r_x2_lo    <= i_rx_byte;
                                r_byte_idx <= 3'd4;
                            end

                            3'd4: begin
                                r_y1_hi    <= i_rx_byte[3:0];
                                r_byte_idx <= 3'd5;
                            end

                            3'd5: begin
                                r_y1_lo    <= i_rx_byte;
                                r_byte_idx <= 3'd6;
                            end

                            3'd6: begin
                                r_y2_hi    <= i_rx_byte[3:0];
                                r_byte_idx <= 3'd7;
                            end

                            3'd7: begin
                                // last byte of the sample: commit the assembled
                                // 12-bit values to the event_buffer
                                o_wr_en   <= 1'b1;
                                o_wr_addr <= r_samples_written;
                                o_wr_x1   <= {r_x1_hi, r_x1_lo};
                                o_wr_x2   <= {r_x2_hi, r_x2_lo};
                                o_wr_y1   <= {r_y1_hi, r_y1_lo};
                                o_wr_y2   <= {r_y2_hi, i_rx_byte};

                                // last sample of the packet -> wait for footer
                                if (r_samples_written == (r_sample_count - 16'd1)) begin
                                    r_byte_idx <= 3'd0;
                                    r_state    <= S_WAIT_FOOTER;
                                end else begin
                                    // more samples remain -> reset byte index, advance addr
                                    r_samples_written <= r_samples_written + 16'd1;
                                    r_byte_idx        <= 3'd0;
                                end
                            end

                            default: begin
                                r_byte_idx <= 3'd0;
                            end
                        endcase
                    end
                end

                S_WAIT_FOOTER: begin
                    // expect FOOTER_BYTE; commit on match, raise proto_err on mismatch
                    if (i_rx_dv) begin
                        o_busy <= 1'b0;

                        if (i_rx_byte == FOOTER_BYTE) begin
                            o_event_commit <= 1'b1;
                        end else begin
                            o_proto_err <= 1'b1;
                        end

                        r_state <= S_IDLE;
                    end
                end

                default: begin
                    r_state <= S_IDLE;
                    o_busy  <= 1'b0;
                end
            endcase
        end
    end

endmodule