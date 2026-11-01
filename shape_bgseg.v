// =============================================================================
// shape_bgseg.v  (NEW in v4)  -  object / background decision for ANY background
//
//   The old binarizer compared the grey value with a fixed threshold and needed
//   the right polarity (dark object on light ground or the reverse), so it only
//   worked for one kind of lighting / background.  Now the background colour is
//   MEASURED every frame from the band along the picture edge (BORDER .. BORDER+8):
//
//   1. Most frequent colour.  Three 16-bin histograms (R, G, B nibble of the 12 bit
//      pixel); the peak (3-bin window) is the background colour g0.  The most
//      frequent colour is used, not the average, so an object that touches one or
//      two picture edges does not disturb it.
//   2. Lighting gradient.  Edge pixels that are close to g0 (|dR|+|dG|+|dB| <= 12
//      nibble steps - object pixels are left out) are averaged in 8 zones along
//      the top/bottom edge (X[0..7]) and 6 zones along the left/right edge
//      (Y[0..5]) and overall (G).  The background colour at pixel (x,y) is
//            ref(x,y) = X[zone x] + Y[zone y] - G
//      so a light that gets darker to one side (or to the top / bottom) is followed.
//   3. A pixel is OBJECT when |R-refR| + |G-refG| + |B-refB| >= TH,
//      TH = 4 + sens/2 nibble steps,  sens = SW[7:4]  (0 = most sensitive).
//   Black, white, grey or coloured background, dark / light / coloured object,
//   bright or dim room: all the same, no polarity switch.
//   The values measured in frame n are used from frame n+1 on (invisible); after
//   power-up or a sudden change of light it needs about 2 frames to settle.
// =============================================================================
module shape_bgseg #(
    parameter BORDER = 8,
    parameter RING   = 8,
    parameter GATE   = 12
)(
    input  wire        clk,            // 25 MHz VGA clock
    input  wire        frame_start,
    input  wire        active,
    input  wire [9:0]  x,
    input  wire [9:0]  y,
    input  wire [11:0] rgb,            // camera pixel (frame buffer output), aligned with x/y
    input  wire [3:0]  sens,
    output wire        bin,            // 1 = object pixel
    output reg  [3:0]  ref_r = 4'd0, ref_g = 4'd0, ref_b = 4'd0     // g0 (debug)
);
    wire [8:0] xs = x[9:1];
    wire [7:0] ys = y[8:1];
    wire sample  = active & ~x[0] & ~y[0];
    wire in_roi  = (xs >= BORDER) && (xs <= 319 - BORDER) &&
                   (ys >= BORDER) && (ys <= 239 - BORDER);
    wire in_tb   = in_roi && ((ys < BORDER + RING) || (ys > 239 - BORDER - RING));   // top / bottom band
    wire in_lr   = in_roi && ((xs < BORDER + RING) || (xs > 319 - BORDER - RING));   // left / right band
    wire upd     = sample & (in_tb | in_lr);

    wire [3:0] pr = rgb[11:8], pg = rgb[7:4], pb = rgb[3:0];

    function [3:0] ad;
        input [3:0] a, b;
        ad = (a > b) ? a - b : b - a;
    endfunction

    // zone numbers
    wire [8:0]  xo  = (xs < BORDER) ? 9'd0 : (xs > 319 - BORDER) ? 9'd303 : xs - BORDER;
    wire [7:0]  yo  = (ys < BORDER) ? 8'd0 : (ys > 239 - BORDER) ? 8'd223 : ys - BORDER;
    wire [12:0] xz_ = xo * 13'd27;
    wire [12:0] yz_ = {5'b0, yo} * 13'd27;
    wire [2:0]  zx  = xz_[12:10];                  // 0..7
    wire [2:0]  zy  = yz_[12:10];                  // 0..5

    // ---------------------------------------------------------------- histograms
    reg [207:0] hr = 208'd0, hg = 208'd0, hb = 208'd0;
    wire [7:0] ir = {4'b0, pr} * 8'd13;
    wire [7:0] ig = {4'b0, pg} * 8'd13;
    wire [7:0] ib = {4'b0, pb} * 8'd13;
    wire [12:0] cr = hr[ir +: 13];
    wire [12:0] cg = hg[ig +: 13];
    wire [12:0] cb = hb[ib +: 13];

    // ------------------------------------------------- gated zone accumulators
    wire near = ({2'b0, ad(pr, ref_r)} + {2'b0, ad(pg, ref_g)} + {2'b0, ad(pb, ref_b)}) <= GATE;
    reg [335:0] xsum = 336'd0;                     // 8 zones x 3 ch x 14 bit
    reg [79:0]  xcnt = 80'd0;                      // 8 zones x 10 bit
    reg [251:0] ysum = 252'd0;                     // 6 zones x 3 ch x 14 bit
    reg [59:0]  ycnt = 60'd0;                      // 6 zones x 10 bit
    reg [62:0]  tsum = 63'd0;                      // 3 ch x 21 bit  (total)
    reg [13:0]  tcnt = 14'd0;

    wire [8:0]  xi_s = {3'b0, zx} * 9'd3 * 9'd14;  // bit offset of zone zx, ch 0
    wire [8:0]  yi_s = {3'b0, zy} * 9'd3 * 9'd14;
    wire [6:0]  xi_c = {4'b0, zx} * 7'd10;
    wire [5:0]  yi_c = {3'b0, zy} * 6'd10;
    wire [9:0]  xc_cur = xcnt[xi_c +: 10];
    wire [9:0]  yc_cur = ycnt[yi_c +: 10];
    wire [13:0] xr_cur = xsum[xi_s +: 14],       xg_cur = xsum[xi_s + 14 +: 14],  xb_cur = xsum[xi_s + 28 +: 14];
    wire [13:0] yr_cur = ysum[yi_s +: 14],       yg_cur = ysum[yi_s + 14 +: 14],  yb_cur = ysum[yi_s + 28 +: 14];

    // ------------------------------------------------ results used by the pixels
    reg [191:0] xv = 192'd0;                       // 8 zones x 3 ch x 8 bit  (x16 scale)
    reg [143:0] yv = 144'd0;                       // 6 zones x 3 ch x 8 bit
    reg [23:0]  gv = 24'd0;                        // 3 ch x 8 bit

    // ---------------------------------------------------------------- the FSM
    localparam [2:0] F_IDLE = 3'd0, F_SCAN = 3'd1, F_LOAD = 3'd2, F_DIV = 3'd3, F_PUT = 3'd4, F_CLR = 3'd5;
    reg [2:0]  fs_st = F_IDLE;
    reg [4:0]  sb_i  = 5'd0;
    reg [14:0] best_r = 15'd0, best_g = 15'd0, best_b = 15'd0;
    reg [3:0]  idx_r = 4'd0, idx_g = 4'd0, idx_b = 4'd0;
    reg        trust = 1'b0;

    function [14:0] wsum;                          // counter b-1 + b + b+1
        input [207:0] v;
        input [3:0]   b;
        reg [12:0] l, m, r;
        begin
            m = v[b*13 +: 13];
            l = (b == 4'd0)  ? 13'd0 : v[(b - 4'd1)*13 +: 13];
            r = (b == 4'd15) ? 13'd0 : v[(b + 4'd1)*13 +: 13];
            wsum = {2'b0, l} + {2'b0, m} + {2'b0, r};
        end
    endfunction
    wire [3:0]  bi  = sb_i[3:0];
    wire [14:0] wr_ = wsum(hr, bi), wg_ = wsum(hg, bi), wb_ = wsum(hb, bi);

    // divider job: phase 0 = X zones, 1 = Y zones, 2 = overall
    reg [1:0]  ph = 2'd0;
    reg [2:0]  zi = 3'd0;
    reg [1:0]  ch = 2'd0;
    reg [23:0] dnd = 24'd0, quo = 24'd0;
    reg [24:0] rem = 25'd0;
    reg [13:0] dsr = 14'd0;
    reg [4:0]  dcnt = 5'd0;
    reg        d_ok = 1'b0;
    wire [24:0] rem_s = {rem[23:0], dnd[23]};
    wire        ge_d  = (rem_s >= {11'b0, dsr});
    wire [3:0]  g0_ch = (ch == 2'd0) ? ref_r : (ch == 2'd1) ? ref_g : ref_b;

    // numerator / denominator selection for the job
    reg [20:0] n_sel;
    reg [13:0] d_sel;
    wire [8:0] xs_off = ({6'b0, zi} * 9'd3 + {7'b0, ch}) * 9'd14;
    wire [8:0] ys_off = xs_off;
    wire [6:0] xc_off = {4'b0, zi} * 7'd10;
    wire [5:0] yc_off = {3'b0, zi} * 6'd10;
    wire [5:0] ts_off = {4'b0, ch} * 6'd21;
    always @(*) begin
        case (ph)
            2'd0:    begin n_sel = {7'b0, xsum[xs_off +: 14]}; d_sel = {4'b0, xcnt[xc_off +: 10]}; end
            2'd1:    begin n_sel = {7'b0, ysum[ys_off +: 14]}; d_sel = {4'b0, ycnt[yc_off +: 10]}; end
            default: begin n_sel = tsum[ts_off +: 21];         d_sel = tcnt; end
        endcase
    end
    wire [13:0] min_n = (ph == 2'd2) ? 14'd256 : 14'd16;

    wire [7:0] q8 = (quo > 24'd255) ? 8'd255 : quo[7:0];
    wire [7:0] fb = {g0_ch, 4'b0};                 // fallback = most frequent colour
    wire [7:0] res = d_ok ? q8 : fb;
    wire [8:0] xo_i = ({6'b0, zi} * 9'd3 + {7'b0, ch}) * 9'd8;
    wire [8:0] yo_i = xo_i;
    wire [4:0] go_i = {3'b0, ch} * 5'd8;

    always @(posedge clk) begin
        if (frame_start && fs_st == F_IDLE) begin
            fs_st <= F_SCAN; sb_i <= 5'd0;
            best_r <= 15'd0; best_g <= 15'd0; best_b <= 15'd0;
        end else begin
            case (fs_st)
            F_IDLE: begin
                if (upd) begin
                    if (cr != 13'h1FFF) hr[ir +: 13] <= cr + 13'd1;
                    if (cg != 13'h1FFF) hg[ig +: 13] <= cg + 13'd1;
                    if (cb != 13'h1FFF) hb[ib +: 13] <= cb + 13'd1;
                    if (near) begin
                        tsum[0  +: 21] <= tsum[0  +: 21] + pr;
                        tsum[21 +: 21] <= tsum[21 +: 21] + pg;
                        tsum[42 +: 21] <= tsum[42 +: 21] + pb;
                        tcnt <= tcnt + 14'd1;
                        if (in_tb) begin
                            xsum[xi_s      +: 14] <= xr_cur + pr;
                            xsum[xi_s + 14 +: 14] <= xg_cur + pg;
                            xsum[xi_s + 28 +: 14] <= xb_cur + pb;
                            xcnt[xi_c +: 10]      <= xc_cur + 10'd1;
                        end else if (in_lr) begin
                            ysum[yi_s      +: 14] <= yr_cur + pr;
                            ysum[yi_s + 14 +: 14] <= yg_cur + pg;
                            ysum[yi_s + 28 +: 14] <= yb_cur + pb;
                            ycnt[yi_c +: 10]      <= yc_cur + 10'd1;
                        end
                    end
                end
            end
            F_SCAN: begin
                if (sb_i == 5'd16) begin
                    trust <= (best_g >= 15'd256);
                    if (best_g >= 15'd256) begin ref_r <= idx_r; ref_g <= idx_g; ref_b <= idx_b; end
                    fs_st <= F_LOAD; ph <= 2'd0; zi <= 3'd0; ch <= 2'd0;
                end else begin
                    if (wr_ > best_r) begin best_r <= wr_; idx_r <= bi; end
                    if (wg_ > best_g) begin best_g <= wg_; idx_g <= bi; end
                    if (wb_ > best_b) begin best_b <= wb_; idx_b <= bi; end
                    sb_i <= sb_i + 5'd1;
                end
            end
            F_LOAD: begin
                dnd  <= {n_sel[19:0], 4'b0};                        // mean in x16 scale
                dsr  <= d_sel;
                d_ok <= (d_sel >= min_n);
                rem  <= 25'd0; quo <= 24'd0; dcnt <= 5'd0;
                fs_st <= F_DIV;
            end
            F_DIV: begin
                dnd <= {dnd[22:0], 1'b0};
                if (ge_d) begin rem <= rem_s - {11'b0, dsr}; quo <= {quo[22:0], 1'b1}; end
                else      begin rem <= rem_s;                 quo <= {quo[22:0], 1'b0}; end
                dcnt <= dcnt + 5'd1;
                if (dcnt == 5'd23) fs_st <= F_PUT;
            end
            F_PUT: begin
                case (ph)
                    2'd0: xv[xo_i +: 8] <= trust ? res : fb;
                    2'd1: yv[yo_i +: 8] <= trust ? res : fb;
                    default: gv[go_i +: 8] <= trust ? res : fb;
                endcase
                fs_st <= F_LOAD;
                if (ch != 2'd2) ch <= ch + 2'd1;
                else begin
                    ch <= 2'd0;
                    if (ph == 2'd2) fs_st <= F_CLR;
                    else if (ph == 2'd0 && zi == 3'd7) begin ph <= 2'd1; zi <= 3'd0; end
                    else if (ph == 2'd1 && zi == 3'd5) begin ph <= 2'd2; zi <= 3'd0; end
                    else zi <= zi + 3'd1;
                end
            end
            F_CLR: begin
                hr <= 208'd0; hg <= 208'd0; hb <= 208'd0;
                xsum <= 336'd0; xcnt <= 80'd0; ysum <= 252'd0; ycnt <= 60'd0;
                tsum <= 63'd0; tcnt <= 14'd0;
                fs_st <= F_IDLE;
            end
            default: fs_st <= F_IDLE;
            endcase
        end
    end

    // --------------------------------------- reference colour at the current pixel
    reg [7:0] rx_r = 8'd0, rx_g = 8'd0, rx_b = 8'd0;
    reg [7:0] ry_r = 8'd0, ry_g = 8'd0, ry_b = 8'd0;
    reg [7:0] rc_r = 8'd0, rc_g = 8'd0, rc_b = 8'd0;
    wire [8:0] xv_i = {6'b0, zx} * 9'd24;
    wire [8:0] yv_i = {6'b0, zy} * 9'd24;

    function [7:0] clamp;
        input signed [10:0] v;
        clamp = v[10] ? 8'd0 : (v > 11'sd255) ? 8'd255 : v[7:0];
    endfunction
    always @(posedge clk) begin
        rx_r <= xv[xv_i +: 8];  rx_g <= xv[xv_i + 8 +: 8];  rx_b <= xv[xv_i + 16 +: 8];
        ry_r <= yv[yv_i +: 8];  ry_g <= yv[yv_i + 8 +: 8];  ry_b <= yv[yv_i + 16 +: 8];
        rc_r <= clamp($signed({3'b0, rx_r}) + $signed({3'b0, ry_r}) - $signed({3'b0, gv[7:0]}));
        rc_g <= clamp($signed({3'b0, rx_g}) + $signed({3'b0, ry_g}) - $signed({3'b0, gv[15:8]}));
        rc_b <= clamp($signed({3'b0, rx_b}) + $signed({3'b0, ry_b}) - $signed({3'b0, gv[23:16]}));
    end

    // ---- object decision (combinational, same timing as the old binarizer) -------
    function [7:0] ad8;
        input [7:0] a, b;
        ad8 = (a > b) ? a - b : b - a;
    endfunction
    wire [9:0] d_sum = {2'b0, ad8({pr, 4'b0}, rc_r)} + {2'b0, ad8({pg, 4'b0}, rc_g)} + {2'b0, ad8({pb, 4'b0}, rc_b)};
    wire [9:0] thr   = {2'b0, 4'd4 + {1'b0, sens[3:1]}, 4'b0};
    assign bin = (d_sum >= thr);
endmodule
