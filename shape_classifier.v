// =============================================================================
// shape_classifier.v  (v3)  -  tilt-proof, cut-off-proof shape decision
//
// MAIN METHOD (objects larger than SMALL_DIM source pixels)
//   shape_orient / shape_harm deliver the edge-direction harmonics
//   R2 = |C2|/n , R4 = |C4|/n , R6 = |C6|/n   (n = number of edge pixels):
//
//        square / rectangle ... R4 high (edge directions repeat every 90 deg)
//        triangle ........... R6 high (repeat every 60 deg)
//        ellipse ............ R2 clearly above 0     circle ... all low
//        square vs rectangle: R2 of the edge lengths  (|w-h|/(w+h) for a full rectangle)
//
//   WHOLE OBJECT VISIBLE (not cut by the picture edge), R in 1/32 units
//        moment invariant A^6 < 125*det (shape_kmom) .. TRIANGLE  (any triangle:
//                                                       equilateral, right-angled, scalene)
//        R4 >= 20/32 ............................... SQUARE (R2 < 3/32) or RECTANGLE
//        R2 >= 5/32 ................................ ELLIPSE, otherwise CIRCLE
//   CUT OFF by the picture edge ("clipped"): only the directions of the visible
//   edges are known, so
//        R4 >= 22/32 ............................... SQUARE (R2 < 7/32) or RECTANGLE
//        3 strong direction peaks (shape_peaks, >= 23/32 of all edge pixels, not
//        bunched together) or R6 >= 10/32 .......... TRIANGLE
//        else R2 >= 8/32 ........................... ELLIPSE, otherwise CIRCLE
//   These numbers do not change when the object is rotated.  When the length ratio
//   of the sides can not be seen (clipped), the SQUARE/RECTANGLE and
//   CIRCLE/ELLIPSE answer of the last un-clipped frame (mem) is kept.
//   The tilt is arg(C4)/4 (square, rectangle), the strongest-direction peak
//   (triangle, shape_peaks) or arg(C2)/2 (ellipse); it is sent back to shape_orient
//   as (tc, ts) so the next frame's bounding box is measured in the tilted frame.
//
// SMALL OBJECTS (both box sides < SMALL_DIM): the v2 method (fill ratio +
//   corner occupancy, upright / 45 deg only) is used, because a 5x5 edge
//   window is too coarse below about 25 pixels.
//
// BACKGROUND REJECTION: a blob is "background" (not an object) when it touches
//   three or four sides of the measuring area, or when more than 3/8 of the
//   edge band is object.  A single object that is merely cut off by one or
//   two picture edges is accepted (v2 rejected it).
//
//   shape codes:  0 NONE   1 CIRCLE   2 SQUARE   3 RECTANGLE
//                 4 TRIANGLE   5 ELLIPSE
//   The result only changes after the same raw answer was seen on AGREE+1
//   consecutive frames.
// =============================================================================
module shape_classifier #(
    parameter MIN_AREA  = 120,
    parameter MIN_DIM   = 10,
    parameter SMALL_DIM = 26,
    parameter NMIN      = 40,
    parameter AGREE     = 2,
    parameter BORDER    = 8,
    parameter RING_BG   = 3072        // 3/8 of the edge band (8192 pixels)
)(
    input  wire        clk,
    input  wire        h_done,                    // harmonics are valid
    input  wire        force_axis,                // 1 = never tilt the box
    // blob data of the finished frame (shape_features)
    input  wire [16:0] area,
    input  wire [8:0]  xmin, xmax,
    input  wire [7:0]  ymin, ymax,
    input  wire [12:0] c_tl, c_tr, c_bl, c_br,
    input  wire [12:0] c_area,
    input  wire [15:0] ring_o,
    // harmonics (shape_harm)
    input  wire [17:0] hn,
    input  wire [27:0] m2, m4, m6,
    input  wire signed [15:0] a2, a4, a6,
    // triangle tests: moment invariant (shape_kmom) and direction peaks (shape_peaks)
    input  wire        k_tri,
    input  wire [16:0] pk_p3,
    input  wire [4:0]  pk_d12, pk_d13,
    input  wire signed [15:0] pk_alpha,

    output reg  [2:0]  shape   = 3'd0,
    output reg         found   = 1'b0,
    output reg         clipped = 1'b0,            // object is cut off by the picture edge
    output reg  [8:0]  bx0 = 9'd0, bx1 = 9'd0,
    output reg  [7:0]  by0 = 8'd0, by1 = 8'd0,
    output reg  signed [7:0] tc = 8'sd127,        // tilt for the next frame (cos, sin x127)
    output reg  signed [7:0] ts = 8'sd0
);
    localparam [2:0] NONE=0, CIRCLE=1, SQUARE=2, RECT=3, TRI=4, ELLIPSE=5;
    localparam [31:0] K127 = 32'd209;             // 127 * CORDIC gain 1.6468

    // ------------------------------------------------------------- blob geometry
    wire [9:0]  w = {1'b0, xmax} - {1'b0, xmin} + 10'd1;
    wire [8:0]  h = {1'b0, ymax} - {1'b0, ymin} + 9'd1;
    wire [19:0] bbarea = w * h;
    wire valid_box = (xmax >= xmin) && (ymax >= ymin);

    wire t_l = (xmin <= BORDER);
    wire t_r = (xmax >= 319 - BORDER);
    wire t_t = (ymin <= BORDER);
    wire t_b = (ymax >= 239 - BORDER);
    wire [2:0] sides = t_l + t_r + t_t + t_b;
    wire is_clipped  = (sides != 3'd0);
    wire bg_like     = (sides >= 3'd3) || (ring_o > RING_BG);

    wire is_found = valid_box && !bg_like &&
                    (area >= MIN_AREA) && (w >= MIN_DIM) && (h >= MIN_DIM);
    wire is_small    = !is_clipped && (w < SMALL_DIM) && (h < SMALL_DIM);

    // ----------------------------------------------- v2 method (small objects)
    wire [31:0] a128   = {area, 7'b0};
    wire [31:0] bb83   = bbarea * 32'd83;
    wire [31:0] bb111  = bbarea * 32'd111;
    wire lt_tri  = a128 < bb83;
    wire lt_circ = a128 < bb111;

    wire [12:0] w8 = {w, 3'b0}, h8 = {1'b0, h, 3'b0};
    wire [12:0] w9 = w8 + w,    h9 = h8 + h;
    wire aspect_sq = (w8 <= h9) && (h8 <= w9);

    wire [16:0] ca9   = c_area * 17'd9;
    wire        c_ok  = (c_area >= 13'd4);
    wire f_tl = c_ok && ({c_tl, 4'b0} > ca9);
    wire f_tr = c_ok && ({c_tr, 4'b0} > ca9);
    wire f_bl = c_ok && ({c_bl, 4'b0} > ca9);
    wire f_br = c_ok && ({c_br, 4'b0} > ca9);
    wire e_tl = c_ok && ({c_tl, 3'b0} < c_area);
    wire e_tr = c_ok && ({c_tr, 3'b0} < c_area);
    wire e_bl = c_ok && ({c_bl, 3'b0} < c_area);
    wire e_br = c_ok && ({c_br, 3'b0} < c_area);
    wire [2:0] n_full  = f_tl + f_tr + f_bl + f_br;
    wire       all_empty = e_tl & e_tr & e_bl & e_br;
    wire two_adj = (n_full == 3'd2) &&
                   ((f_tl & f_tr) | (f_bl & f_br) | (f_tl & f_bl) | (f_tr & f_br));

    reg [2:0] raw_old;
    always @(*) begin
        if (n_full >= 3'd3)                        raw_old = aspect_sq ? SQUARE : RECT;
        else if (two_adj)                          raw_old = TRI;
        else if (all_empty && lt_tri && aspect_sq) raw_old = SQUARE;
        else if (lt_tri)                           raw_old = TRI;
        else if (lt_circ)                          raw_old = aspect_sq ? CIRCLE : ELLIPSE;
        else                                       raw_old = aspect_sq ? SQUARE : RECT;
    end

    // ------------------------------------- main method: pipelined comparisons
    // p1 : nk = n * 209
    // p2 : flags
    // p3 : decision, debounce, outputs
    reg p1 = 1'b0, p2 = 1'b0, p3 = 1'b0;
    reg [31:0] nk = 32'd0;
    reg f4_20 = 1'b0, f4_22 = 1'b0, f6_10 = 1'b0;
    reg f2_3 = 1'b0, f2_5 = 1'b0, f2_7 = 1'b0, f2_8 = 1'b0;
    reg pk3 = 1'b0, pk_ok = 1'b0;

    wire [39:0] m2x = {m2, 5'b0}, m4x = {m4, 5'b0}, m6x = {m6, 5'b0};
    wire [39:0] nk40 = {8'b0, nk};
    wire [22:0] p3x  = {pk_p3, 5'b0};
    wire [22:0] n23  = {5'b0, hn} * 23;

    always @(posedge clk) begin
        p1 <= h_done;
        p2 <= p1;
        p3 <= p2;
        if (h_done) nk <= {14'b0, hn} * K127;
        if (p1) begin
            f4_20 <= (m4x >= nk40 * 40'd20);
            f4_22 <= (m4x >= nk40 * 40'd22);
            f6_10 <= (m6x >= nk40 * 40'd10);
            f2_3  <= (m2x >= nk40 * 40'd3);
            f2_5  <= (m2x >= nk40 * 40'd5);
            f2_7  <= (m2x >= nk40 * 40'd7);
            f2_8  <= (m2x >= nk40 * 40'd8);
            pk3   <= (p3x >= n23);
            pk_ok <= !((pk_d12 <= 5'd4) && (pk_d13 <= 5'd7));
        end
    end

    reg [2:0] raw_new;
    always @(*) begin
        if (!is_clipped) begin
            // whole object visible
            if (k_tri)       raw_new = TRI;
            else if (f4_20)  raw_new = f2_3 ? RECT : SQUARE;
            else             raw_new = f2_5 ? ELLIPSE : CIRCLE;
        end else begin
            // cut off by the picture edge: directions of the visible edges only
            if (f4_22)              raw_new = f2_7 ? RECT : SQUARE;
            else if (pk3 && pk_ok)  raw_new = TRI;
            else if (f6_10)         raw_new = TRI;
            else                    raw_new = f2_8 ? ELLIPSE : CIRCLE;
        end
    end

    // memory of the last un-clipped answer (square vs rectangle, circle vs ellipse)
    reg [2:0] mem  = NONE;
    reg [4:0] lost = 5'd0;
    wire use_new = is_found && !is_small && (hn >= NMIN);
    reg [2:0] raw;
    always @(*) begin
        if (!is_found)        raw = NONE;
        else if (!use_new)    raw = raw_old;
        else begin
            raw = raw_new;
            if (is_clipped) begin
                if ((raw_new == SQUARE || raw_new == RECT) && (mem == SQUARE || mem == RECT))
                    raw = mem;
                if ((raw_new == CIRCLE || raw_new == ELLIPSE) && (mem == CIRCLE || mem == ELLIPSE))
                    raw = mem;
            end
        end
    end

    // tilt of the raw answer (turns, 65536 = 360 deg, |alpha| <= 45 deg)
    wire signed [15:0] al_rect = a4 >>> 2;
    wire signed [15:0] al_tri  = pk_alpha;                        // from shape_peaks
    wire signed [15:0] a2h     = a2 >>> 1;                      // a2 / 2  (+-90 deg)
    wire signed [15:0] al_ell  = (a2h >  16'sd8192) ? a2h - 16'sd16384 :
                                 (a2h < -16'sd8192) ? a2h + 16'sd16384 : a2h;
    reg signed [15:0] alpha;
    always @(*) begin
        alpha = 16'sd0;
        if (use_new) begin
            case (raw)
                SQUARE, RECT: alpha = al_rect;
                TRI:          alpha = al_tri;
                ELLIPSE:      alpha = al_ell;
                default:      alpha = 16'sd0;
            endcase
        end
        if (force_axis) alpha = 16'sd0;
    end
    wire [15:0] aabs = alpha[15] ? -alpha : alpha;
    wire [21:0] idx_f = aabs * 6'd45;
    wire [6:0]  idx   = (idx_f[21:12] > 10'd90) ? 7'd90 : idx_f[18:12];
    wire signed [7:0] lc, ls;
    shape_tilt_lut u_tilt (.idx(idx), .c(lc), .s(ls));
    wire signed [7:0] tc_n = lc;
    wire signed [7:0] ts_n = alpha[15] ? -ls : ls;

    reg [2:0] last_raw = 3'd0;
    reg [1:0] cnt      = 2'd0;

    always @(posedge clk) begin
        if (p3) begin
            // memory handling
            if (!is_found) begin
                if (lost != 5'd31) lost <= lost + 5'd1;
                if (lost >= 5'd15) mem <= NONE;
            end else begin
                lost <= 5'd0;
                if (use_new && !is_clipped) mem <= raw_new;
            end
            // debounce
            last_raw <= raw;
            if (raw == last_raw) begin
                if (cnt != 2'd3) cnt <= cnt + 2'd1;
                if (cnt >= AGREE-1) begin
                    shape   <= raw;
                    found   <= is_found;
                    clipped <= is_found & is_clipped;
                    bx0 <= xmin; bx1 <= xmax; by0 <= ymin; by1 <= ymax;
                    tc <= is_found ? tc_n : 8'sd127;
                    ts <= is_found ? ts_n : 8'sd0;
                end
            end else begin
                cnt <= 2'd0;
            end
        end
    end
endmodule
