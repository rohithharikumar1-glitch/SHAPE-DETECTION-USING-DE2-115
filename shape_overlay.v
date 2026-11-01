// =============================================================================
// shape_overlay.v  (v3)  -  draws the detection result on the VGA picture
//   show_mask : replace picture by the black/white object mask
//   show_box  : outline of the (tilted) bounding box
//                 green  = whole object inside the picture
//                 orange = object is cut off by the picture edge
//   show_icon : 64x64 panel (top-left) with a drawing of the detected shape
//
// The tilted box is given in the rotated frame the classifier chose:
//        u =  x*c + y*s       v = -x*s + y*c        (c, s = cos, sin x127)
//        umin..umax , vmin..vmax measured on source pixels (shape_orient).
// For every VGA pixel u, v are computed and compared with the limits - no
// line drawing is needed, it works for any tilt.  c = 127, s = 0 gives the
// normal upright box.
//
// The module has TWO pipeline stages; `active_out` is `active` delayed by two
// clocks and the top level delays HS / VS by the same amount.
// =============================================================================
module shape_overlay (
    input  wire        clk,
    input  wire [11:0] pix_in,
    input  wire        active,
    input  wire [9:0]  x,
    input  wire [9:0]  y,
    input  wire        bin,
    input  wire        show_mask,
    input  wire        show_box,
    input  wire        show_icon,
    input  wire        found,
    input  wire        clipped,
    input  wire [2:0]  shape,
    // tilted box (shape_orient snapshot)
    input  wire        ob_valid,
    input  wire signed [19:0] ob_umin, ob_umax, ob_vmin, ob_vmax,
    input  wire signed [7:0]  ob_c, ob_s,
    output reg  [11:0] pix_out    = 12'h000,
    output reg         active_out = 1'b0
);
    // ------------------------------------------------------------- stage 1
    reg [11:0] pix1 = 12'h000;
    reg        act1 = 1'b0, bin1 = 1'b0;
    reg [9:0]  x1 = 10'd0, y1 = 10'd0;
    reg signed [21:0] u1 = 22'sd0, v1 = 22'sd0;

    wire signed [10:0] xs_ = {1'b0, x};
    wire signed [10:0] ys_ = {1'b0, y};

    always @(posedge clk) begin
        pix1 <= pix_in;  act1 <= active;  bin1 <= bin;
        x1 <= x;         y1 <= y;
        u1 <= xs_ * ob_c + ys_ * ob_s;
        v1 <= ys_ * ob_c - xs_ * ob_s;
    end

    // ------------------------------------------------------------- stage 2
    // outer limits (VGA pixels are twice the source pixels, units 1/127 pixel)
    // ($signed is needed: a concatenation is unsigned and would not sign-extend)
    wire signed [22:0] uo0 = $signed({ob_umin, 1'b0}) - 23'sd127;
    wire signed [22:0] uo1 = $signed({ob_umax, 1'b0}) + 23'sd254;
    wire signed [22:0] vo0 = $signed({ob_vmin, 1'b0}) - 23'sd254;
    wire signed [22:0] vo1 = $signed({ob_vmax, 1'b0}) + 23'sd254;
    wire signed [22:0] u2  = u1;
    wire signed [22:0] v2  = v1;
    wire in_outer = (u2 >= uo0) && (u2 <= uo1) && (v2 >= vo0) && (v2 <= vo1);
    wire in_inner = (u2 >= uo0 + 23'sd254) && (u2 <= uo1 - 23'sd254) &&
                    (v2 >= vo0 + 23'sd254) && (v2 <= vo1 - 23'sd254);
    wire box_px = show_box & found & ob_valid & in_outer & ~in_inner;

    // ---- icon panel ---------------------------------------------------------
    wire        in_icon = (x1 >= 10'd8) && (x1 < 10'd72) && (y1 >= 10'd8) && (y1 < 10'd72);
    wire [5:0]  u  = x1 - 10'd8;
    wire [5:0]  v  = y1 - 10'd8;
    wire signed [7:0] du = $signed({2'b00, u}) - 8'sd32;
    wire signed [7:0] dv = $signed({2'b00, v}) - 8'sd32;
    wire [6:0]  adu = du[7] ? -du : du;
    wire [6:0]  adv = dv[7] ? -dv : dv;
    wire [13:0] du2 = adu * adu;
    wire [13:0] dv2 = adv * adv;

    wire s_circle = (du2 + dv2) <= 14'd576;                       // r = 24
    wire s_square = (u >= 12) && (u <= 51) && (v >= 12) && (v <= 51);
    wire s_rect   = (u >= 6)  && (u <= 57) && (v >= 18) && (v <= 45);
    wire [5:0] vt = v - 6'd10;
    wire s_tri    = (v >= 10) && (v <= 54) && (adu <= {1'b0, vt[5:1]});
    wire [24:0] ell = (du2 * 12'd256) + (dv2 * 12'd784);
    wire s_ell    = ell <= 25'd200704;                            // a=28, b=16

    reg icon_on;
    always @(*) begin
        case (shape)
            3'd1:    icon_on = s_circle;
            3'd2:    icon_on = s_square;
            3'd3:    icon_on = s_rect;
            3'd4:    icon_on = s_tri;
            3'd5:    icon_on = s_ell;
            default: icon_on = 1'b0;
        endcase
    end
    wire icon_border = in_icon && ((u == 0) || (u == 63) || (v == 0) || (v == 63));

    reg [11:0] pix_c;
    always @(*) begin
        pix_c = pix1;
        if (show_mask) pix_c = bin1 ? 12'hFFF : 12'h000;
        if (box_px)    pix_c = clipped ? 12'hF80 : 12'h0F0;      // orange / green
        if (show_icon && in_icon) begin
            if (icon_border)  pix_c = found ? 12'h0F0 : 12'hF00; // green / red frame
            else if (icon_on) pix_c = 12'hFF0;                   // yellow shape
            else              pix_c = 12'h222;                   // dark panel
        end
        if (!act1) pix_c = 12'h000;
    end

    always @(posedge clk) begin
        pix_out    <= pix_c;
        active_out <= act1;
    end
endmodule
