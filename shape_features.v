// =============================================================================
// shape_features.v  (NEW)  -  one-pass blob statistics, 320x240 source space
//
// Runs on the VGA read stream (every source pixel is shown 2x2, so only the
// even-x / even-y copy is sampled -> each source pixel is counted exactly once).
//   * ignores a BORDER-pixel frame edge (camera edge artefacts)
//   * a pixel only counts if it is the 3rd (or later) object pixel in a row
//     run  -> kills 1-2 pixel speckle;  +2 is added at run start to compensate
//   * a row only counts if it has >= MIN_ROW object pixels -> kills vertical
//     speckle
// At frame_start (VS falling edge) the accumulators are copied to the outputs,
// `done` pulses for one clock, and the accumulators restart.
//
// Automatic polarity: the background is whatever touches the edges of the
// picture.  The raw `bin` pixels in a RING-pixel band just inside the ROI are
// counted; if more than 5/8 of that band is "object", the polarity is wrong
// and `flip` toggles (the top level XORs it into shape_binarize.invert).
// After a flip the band is < 3/8 object, so it cannot flip straight back.
//
// v3: ring_o (the band count of the finished frame) is also exported, the
// classifier uses it to reject a blob that is really the background.
//
// Corner occupancy: the bounding box of the PREVIOUS frame (the xmin..ymax
// outputs) is split into a 4x4 grid; the raw object pixels in each of the four
// corner cells are counted.  At frame_start the counts (c_tl..c_br) and the
// cell size in pixels (c_area) are output for shape_classifier:
//   square / rectangle ... all four corners ~100 % full
//   triangle ............. two neighbouring corners ~75 %, the other two ~0 %
//   circle / ellipse ..... all four corners ~31 %
//   square at 45 deg ..... all four corners ~0 %
// Being an area measure it still works on small (distant) objects.
// =============================================================================
module shape_features #(
    parameter BORDER  = 8,
    parameter MIN_ROW = 4,
    parameter RING    = 8
)(
    input  wire        clk,         // 25 MHz VGA clock
    input  wire        frame_start,
    input  wire        active,
    input  wire [9:0]  x,
    input  wire [9:0]  y,
    input  wire        bin,         // aligned with x/y/active
    input  wire        auto_pol,    // 1 = let `flip` follow the edge band

    output reg  [16:0] area  = 17'd0,
    output reg  [8:0]  xmin  = 9'd0,
    output reg  [8:0]  xmax  = 9'd0,
    output reg  [7:0]  ymin  = 8'd0,
    output reg  [7:0]  ymax  = 8'd0,
    output reg         done  = 1'b0,
    output reg         flip  = 1'b0,
    output reg  [15:0] ring_o = 16'd0,   // object pixels in the edge band (v3: used to reject "background")
    // corner cells of the previous frame's bounding box
    output reg  [12:0] c_tl  = 13'd0,
    output reg  [12:0] c_tr  = 13'd0,
    output reg  [12:0] c_bl  = 13'd0,
    output reg  [12:0] c_br  = 13'd0,
    output reg  [12:0] c_area = 13'd0
);
    localparam ROI_W   = 320 - 2*BORDER;
    localparam ROI_H   = 240 - 2*BORDER;
    localparam RING_N  = ROI_W*ROI_H - (ROI_W - 2*RING)*(ROI_H - 2*RING);
    localparam FLIP_AT = RING_N * 5 / 8;

    wire [8:0] xs = x[9:1];
    wire [7:0] ys = y[8:1];

    wire sample = active & ~x[0] & ~y[0];
    wire in_roi = (xs >= BORDER) && (xs <= 319 - BORDER) &&
                  (ys >= BORDER) && (ys <= 239 - BORDER);
    wire in_ring = in_roi && ((xs < BORDER + RING) || (xs > 319 - BORDER - RING) ||
                              (ys < BORDER + RING) || (ys > 239 - BORDER - RING));

    reg [15:0] ring_cnt = 16'd0;

    // --- corner cells (quarter of the previous bounding box in each axis) -----
    wire        pv_ok = (xmax >= xmin) && (ymax >= ymin);
    wire [9:0]  pw    = {1'b0, xmax} - {1'b0, xmin} + 10'd1;
    wire [8:0]  ph    = {1'b0, ymax} - {1'b0, ymin} + 9'd1;
    wire [7:0]  cw    = pw[9:2];
    wire [6:0]  ch    = ph[8:2];
    wire in_pbox = pv_ok && (xs >= xmin) && (xs <= xmax) && (ys >= ymin) && (ys <= ymax);
    wire c_l = ({1'b0, xs} <  {1'b0, xmin} + cw);
    wire c_r = ({1'b0, xs} >  {1'b0, xmax} - cw);
    wire c_t = ({1'b0, ys} <  {1'b0, ymin} + ch);
    wire c_b = ({1'b0, ys} >  {1'b0, ymax} - ch);
    wire c_px = sample & in_roi & bin & in_pbox;

    reg [12:0] k_tl = 13'd0, k_tr = 13'd0, k_bl = 13'd0, k_br = 13'd0;

    reg act_d = 1'b0;
    always @(posedge clk) act_d <= active;
    wire row_end = act_d & ~active & ~y[0];       // end of a sampled line

    // --- run filter ----------------------------------------------------------
    reg [1:0] hist   = 2'b00;                     // previous two ROI/bin samples
    reg       qual_d = 1'b0;
    wire      qual   = sample & in_roi & bin & hist[0] & hist[1];
    wire      run_start = qual & ~qual_d;

    // --- per-row accumulators --------------------------------------------------
    reg [9:0] row_cnt  = 10'd0;
    reg [8:0] row_xmin = 9'h1FF;
    reg [8:0] row_xmax = 9'h000;

    // --- per-frame accumulators ----------------------------------------------
    reg [16:0] a_area = 17'd0;
    reg [8:0]  a_xmin = 9'h1FF;
    reg [8:0]  a_xmax = 9'h000;
    reg [7:0]  a_ymin = 8'hFF;
    reg [7:0]  a_ymax = 8'h00;

    always @(posedge clk) begin
        done <= 1'b0;

        if (!active) begin
            hist   <= 2'b00;
            qual_d <= 1'b0;
        end else if (sample) begin
            hist   <= {hist[0], bin & in_roi};
            qual_d <= qual;
        end

        if (frame_start) begin
            area <= a_area;  xmin <= a_xmin;  xmax <= a_xmax;
            ymin <= a_ymin;  ymax <= a_ymax;  done <= 1'b1;
            a_area <= 17'd0; a_xmin <= 9'h1FF; a_xmax <= 9'h000;
            a_ymin <= 8'hFF; a_ymax <= 8'h00;
            row_cnt <= 10'd0; row_xmin <= 9'h1FF; row_xmax <= 9'h000;
            if (!auto_pol)                flip <= 1'b0;
            else if (ring_cnt > FLIP_AT)  flip <= ~flip;
            ring_o  <= ring_cnt;
            ring_cnt <= 16'd0;
            c_tl <= k_tl;  c_tr <= k_tr;  c_bl <= k_bl;  c_br <= k_br;
            c_area <= pv_ok ? cw * ch : 13'd0;
            k_tl <= 13'd0; k_tr <= 13'd0; k_bl <= 13'd0; k_br <= 13'd0;
        end else begin
            if (sample && in_ring && bin) ring_cnt <= ring_cnt + 1'b1;
            if (c_px && c_t && c_l) k_tl <= k_tl + 1'b1;
            if (c_px && c_t && c_r) k_tr <= k_tr + 1'b1;
            if (c_px && c_b && c_l) k_bl <= k_bl + 1'b1;
            if (c_px && c_b && c_r) k_br <= k_br + 1'b1;
            if (qual) begin
                row_cnt <= row_cnt + (run_start ? 10'd3 : 10'd1);
                if ((xs - 9'd2) < row_xmin) row_xmin <= xs - 9'd2;
                if (xs > row_xmax)          row_xmax <= xs;
            end
            if (row_end) begin
                if (row_cnt >= MIN_ROW) begin
                    a_area <= a_area + row_cnt;
                    if (row_xmin < a_xmin) a_xmin <= row_xmin;
                    if (row_xmax > a_xmax) a_xmax <= row_xmax;
                    if (ys < a_ymin)       a_ymin <= ys;
                    if (ys > a_ymax)       a_ymax <= ys;
                end
                row_cnt <= 10'd0; row_xmin <= 9'h1FF; row_xmax <= 9'h000;
            end
        end
    end
endmodule
