// =============================================================================
// pixel_coord.v  (NEW)  -  VGA scan-position counter
// Existing vga_timing.v does not export its h/v counters, so this module
// rebuilds the pixel position from the signals it DOES export:
//   active (1 = visible pixel, aligned with frame-buffer data) and vs.
//   x = 0..639, y = 0..479 (VGA coordinates, aligned with rd_data / active)
//   frame_start = 1-clk pulse at the falling edge of VS (once per frame)
// =============================================================================
module pixel_coord (
    input  wire       clk,         // 25 MHz VGA clock
    input  wire       active,
    input  wire       vs,          // active-low vertical sync
    output reg  [9:0] x = 10'd0,
    output reg  [9:0] y = 10'd0,
    output wire       frame_start
);
    reg act_d = 1'b0;
    reg vs_d  = 1'b1;

    assign frame_start = vs_d & ~vs;

    always @(posedge clk) begin
        act_d <= active;
        vs_d  <= vs;

        // x: 0 on the first visible pixel of a line, +1 per visible pixel
        if (!active) x <= 10'd0;
        else         x <= x + 1'b1;

        // y: +1 when a visible line ends, back to 0 at the start of VS
        if (frame_start)           y <= 10'd0;
        else if (act_d & ~active)  y <= y + 1'b1;
    end
endmodule
