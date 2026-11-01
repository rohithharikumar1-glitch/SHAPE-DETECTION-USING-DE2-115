// =============================================================================
// shape_orient.v  (NEW in v3)  -  edge-direction histogram + tilted bounding box
//
// Works on the same 320x240 source-pixel stream as shape_features (every source
// pixel is shown 2x2, only the even-x / even-y copy is used).
//
// 1) 5x5 window on the black/white mask (4 line buffers of 1 bit)
//        gx = sum_r S[r] * sum_c D[c] * m      S = 1 2 2 2 1   D = -2 -1 0 1 2
//        gy = the same turned by 90 degrees
//    A pixel is an EDGE pixel when |gx| + |gy| >= T_GRAD.  Its edge-normal
//    direction (modulo 180 degrees) is quantised into 32 bins and counted in a
//    histogram.  Only windows that lie completely inside the measuring area
//    (BORDER pixels in from the picture edge) are used, so the artificial
//    "cut" where an object leaves the picture never produces an edge.
//    The histogram is what makes the detector rotation- and cut-off-proof:
//        square / rectangle ... edge directions repeat every 90 deg
//        triangle ........... edge directions repeat every 60 deg
//        circle / ellipse ... all directions (ellipse: two preferred ones)
//
// 2) Tilted bounding box.  For every "solid" object pixel (3x3 majority) the
//    coordinates are projected on a rotated axis pair
//        u =  x*c + y*s        v = -x*s + y*c        (c, s = cos, sin of the tilt, x127)
//    and min / max of u and v are tracked.  (c, s) come from the classifier
//    (tilt measured on the previous frame).  The snapshot at frame_start
//    contains the box together with the (c, s) it was measured with.
//
// At frame_start: histogram -> hsnap (hist_ready pulses), box -> ob_*, all
// accumulators restart.
// =============================================================================
module shape_orient #(
    parameter BORDER = 8,
    parameter T_GRAD = 12
)(
    input  wire        clk,            // 25 MHz VGA clock
    input  wire        frame_start,
    input  wire        active,
    input  wire [9:0]  x,
    input  wire [9:0]  y,
    input  wire        bin,            // aligned with x / y / active
    input  wire signed [7:0] tc,       // cos(tilt) * 127
    input  wire signed [7:0] ts,       // sin(tilt) * 127
    // gate for the moment sums: previous frame's box (source pixels), +-GATE_M
    input  wire        gate_en,
    input  wire [8:0]  gx0, gx1,
    input  wire [7:0]  gy0, gy1,

    output reg  [415:0] hsnap = 416'd0,   // 32 bins x 13 bit
    output reg          hist_ready = 1'b0,
    // tilted bounding box of the previous frame
    output reg  signed [19:0] ob_umin = 20'sd0,
    output reg  signed [19:0] ob_umax = 20'sd0,
    output reg  signed [19:0] ob_vmin = 20'sd0,
    output reg  signed [19:0] ob_vmax = 20'sd0,
    output reg  signed [7:0]  ob_c = 8'sd127,
    output reg  signed [7:0]  ob_s = 8'sd0,
    output reg          ob_valid = 1'b0,
    // moment sums of the solid pixels (snapshot together with the histogram)
    output reg  [16:0]  mo_a   = 17'd0,
    output reg  [24:0]  mo_sx  = 25'd0, mo_sy = 25'd0,
    output reg  [35:0]  mo_sxx = 36'd0, mo_syy = 36'd0, mo_sxy = 36'd0
);
    localparam GATE_M = 8;
    // ---------------------------------------------------------------- sampling
    wire [8:0] xs = x[9:1];
    wire [7:0] ys = y[8:1];
    wire sample = active & ~x[0] & ~y[0];
    wire in_roi = (xs >= BORDER) && (xs <= 319 - BORDER) &&
                  (ys >= BORDER) && (ys <= 239 - BORDER);
    wire pin = bin & in_roi;

    // ------------------------------------------------------------ line buffers
    reg [319:0] lb1 = 320'd0, lb2 = 320'd0, lb3 = 320'd0, lb4 = 320'd0;
    reg [4:0]   col0 = 5'd0, col1 = 5'd0, col2 = 5'd0, col3 = 5'd0, col4 = 5'd0;
    reg [8:0]   cx_r = 9'd0;
    reg [7:0]   cy_r = 8'd0;
    reg         wv   = 1'b0;

    always @(posedge clk) begin
        wv <= sample;
        if (sample) begin
            lb1  <= {lb1[318:0], pin};
            lb2  <= {lb2[318:0], lb1[319]};
            lb3  <= {lb3[318:0], lb2[319]};
            lb4  <= {lb4[318:0], lb3[319]};
            col4 <= col3;
            col3 <= col2;
            col2 <= col1;
            col1 <= col0;
            col0 <= {pin, lb1[319], lb2[319], lb3[319], lb4[319]};
            cx_r <= xs;
            cy_r <= ys;
        end
    end

    // ---------------------------------------------------------------- gradient
    // bit 4 = current row (y), bit 0 = four rows up;  col0 = newest = right-most
    function [3:0] csum;                    // 1 2 2 2 1 weighting of one column
        input [4:0] c;
        csum = c[4] + c[0] + {c[3], 1'b0} + {c[2], 1'b0} + {c[1], 1'b0};
    endfunction
    wire [3:0] cs0 = csum(col0), cs1 = csum(col1), cs3 = csum(col3), cs4 = csum(col4);

    function [3:0] rsum;                    // 1 2 2 2 1 weighting of one row (bit b)
        input [4:0] c0, c1, c2, c3, c4;     // the five columns
        input integer b;
        rsum = c0[b] + c4[b] + {c1[b], 1'b0} + {c2[b], 1'b0} + {c3[b], 1'b0};
    endfunction
    wire [3:0] rs4 = rsum(col0, col1, col2, col3, col4, 4);
    wire [3:0] rs3 = rsum(col0, col1, col2, col3, col4, 3);
    wire [3:0] rs1 = rsum(col0, col1, col2, col3, col4, 1);
    wire [3:0] rs0 = rsum(col0, col1, col2, col3, col4, 0);

    wire signed [6:0] gx = $signed({3'b000, cs0, 1'b0}) + $signed({3'b000, cs1})
                         - $signed({3'b000, cs3})       - $signed({3'b000, cs4, 1'b0});
    wire signed [6:0] gy = $signed({3'b000, rs4, 1'b0}) + $signed({3'b000, rs3})
                         - $signed({3'b000, rs1})       - $signed({3'b000, rs0, 1'b0});

    wire [6:0] agx = gx[6] ? -gx : gx;
    wire [6:0] agy = gy[6] ? -gy : gy;
    wire       edge_px = ({1'b0, agx} + {1'b0, agy}) >= T_GRAD;

    // sign normalisation: gy_n >= 0 (and gx_n > 0 when gy_n == 0)
    wire flip = gy[6] | ((gy == 7'sd0) & gx[6]);
    wire signed [6:0] gxn = flip ? -gx : gx;
    wire signed [6:0] gyn = flip ? -gy : gy;
    wire [10:0] lut_a = {gyn[4:0], 6'd24 + gxn[5:0]};
    wire [4:0]  lut_b;
    shape_bin_lut u_bin_lut (.a(lut_a), .b(lut_b));

    // centre pixel of the window
    wire [9:0] cxx = {1'b0, cx_r} - 10'd2;
    wire [9:0] cyy = {2'b00, cy_r} - 10'd2;
    wire h_ok = (cx_r >= BORDER + 4) && (cx_r <= 319 - BORDER) &&
                (cy_r >= BORDER + 4) && (cy_r <= 239 - BORDER);
    wire c_roi = (cx_r >= BORDER + 2) && (cx_r <= 321 - BORDER) &&
                 (cy_r >= BORDER + 2) && (cy_r <= 241 - BORDER);

    // 3x3 majority around the centre (rows 3..1, columns 1..3)
    wire [3:0] n3 = col1[3] + col1[2] + col1[1] + col2[3] + col2[2] + col2[1] +
                    col3[3] + col3[2] + col3[1];
    wire solid = (n3 >= 4'd5) & col2[2];

    // tilted coordinates of the centre pixel
    wire signed [19:0] u_new = $signed({10'b0, cxx}) * tc + $signed({10'b0, cyy}) * ts;
    wire signed [19:0] v_new = $signed({10'b0, cyy}) * tc - $signed({10'b0, cxx}) * ts;

    // ------------------------------------------------------------- stage 2 regs
    // gate test (signed arithmetic, 11 bit)
    wire signed [10:0] sxx_ = $signed({1'b0, cxx});
    wire signed [10:0] syy_ = $signed({1'b0, cyy});
    wire g_ok = !gate_en ||
                ((sxx_ >= $signed({2'b0, gx0}) - GATE_M) && (sxx_ <= $signed({2'b0, gx1}) + GATE_M) &&
                 (syy_ >= $signed({3'b0, gy0}) - GATE_M) && (syy_ <= $signed({3'b0, gy1}) + GATE_M));

    reg        h_v = 1'b0;
    reg        q_v = 1'b0;
    reg [8:0]  q_x = 9'd0;
    reg [7:0]  q_y = 8'd0;
    reg [4:0]  h_b = 5'd0;
    reg        o_v = 1'b0;
    reg signed [19:0] o_u = 20'sd0, o_w = 20'sd0;

    always @(posedge clk) begin
        h_v <= wv & h_ok & edge_px;
        h_b <= lut_b;
        o_v <= wv & c_roi & solid;
        q_v <= wv & c_roi & solid & g_ok;
        q_x <= cxx[8:0];
        q_y <= cyy[7:0];
        o_u <= u_new;
        o_w <= v_new;
    end

    // ---------------------------------------------------------------- histogram
    // 32 counters of 13 bit, kept in one flat register vector (bin b = hc[b*13 +: 13])
    reg [415:0] hc = 416'd0;

    // tilted box accumulators
    reg signed [19:0] a_umin = 20'sh7FFFF, a_umax = 20'sh80000;
    reg signed [19:0] a_vmin = 20'sh7FFFF, a_vmax = 20'sh80000;
    reg               a_any  = 1'b0;

    // moment pipeline: stage 3 = products, stage 4 = accumulate
    reg        r_v = 1'b0;
    reg [8:0]  r_x = 9'd0;
    reg [7:0]  r_y = 8'd0;
    reg [17:0] r_xx = 18'd0, r_xy = 18'd0;   // xy needs 17 bit
    reg [15:0] r_yy = 16'd0;
    always @(posedge clk) begin
        r_v  <= q_v;
        r_x  <= q_x;  r_y <= q_y;
        r_xx <= q_x * q_x;
        r_yy <= q_y * q_y;
        r_xy <= q_x * q_y;
    end
    reg [16:0] ac_a = 17'd0;
    reg [24:0] ac_sx = 25'd0, ac_sy = 25'd0;
    reg [35:0] ac_sxx = 36'd0, ac_syy = 36'd0, ac_sxy = 36'd0;

    wire [8:0]  hsel = {4'b0, h_b} * 9'd13;               // bit position of the selected counter
    wire [12:0] hcur = hc[hsel +: 13];

    always @(posedge clk) begin
        hist_ready <= 1'b0;
        if (frame_start) begin
            hsnap      <= hc;
            hc         <= 416'd0;
            hist_ready <= 1'b1;
            ob_umin <= a_umin; ob_umax <= a_umax;
            ob_vmin <= a_vmin; ob_vmax <= a_vmax;
            ob_valid <= a_any;
            ob_c <= tc;  ob_s <= ts;
            a_umin <= 20'sh7FFFF; a_umax <= 20'sh80000;
            a_vmin <= 20'sh7FFFF; a_vmax <= 20'sh80000;
            a_any <= 1'b0;
            mo_a <= ac_a; mo_sx <= ac_sx; mo_sy <= ac_sy;
            mo_sxx <= ac_sxx; mo_syy <= ac_syy; mo_sxy <= ac_sxy;
            ac_a <= 17'd0; ac_sx <= 25'd0; ac_sy <= 25'd0;
            ac_sxx <= 36'd0; ac_syy <= 36'd0; ac_sxy <= 36'd0;
        end else begin
            if (r_v) begin
                ac_a   <= ac_a + 17'd1;
                ac_sx  <= ac_sx + r_x;
                ac_sy  <= ac_sy + r_y;
                ac_sxx <= ac_sxx + r_xx;
                ac_syy <= ac_syy + r_yy;
                ac_sxy <= ac_sxy + r_xy;
            end
            if (h_v && hcur != 13'h1FFF) hc[hsel +: 13] <= hcur + 13'd1;
            if (o_v) begin
                a_any <= 1'b1;
                if (o_u < a_umin) a_umin <= o_u;
                if (o_u > a_umax) a_umax <= o_u;
                if (o_w < a_vmin) a_vmin <= o_w;
                if (o_w > a_vmax) a_vmax <= o_w;
            end
        end
    end
endmodule
