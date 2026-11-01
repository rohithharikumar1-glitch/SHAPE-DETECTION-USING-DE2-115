// =============================================================================
// shape_peaks.v  (NEW in v3)  -  strongest edge directions of the histogram
//
// Finds the three strongest direction peaks (3-bin window, the winner and its two
// neighbours are removed before the next search) of the 32-bin histogram.
//   p3        = mass of the three peaks      (a triangle has 3 strong directions,
//                                             even when it is cut by the picture edge)
//   d12, d13  = circular bin distance between peak 1 and peaks 2 / 3
//   tri_alpha = tilt for the box of a triangle: peak direction, wrapped to +-45 deg,
//               refined by the slope between the two bins next to the peak
//               (65536 = 360 deg, +-8192 = +-45 deg)
// Bit-exact with ref2.find_peaks / peak_info / tri_tilt.   About 130 clocks.
// =============================================================================
module shape_peaks (
    input  wire         clk,
    input  wire         start,
    input  wire [415:0] hsnap,
    output reg  [16:0]  p3 = 17'd0,
    output reg  [4:0]   d12 = 5'd0, d13 = 5'd0,
    output reg  signed [15:0] tri_alpha = 16'sd0,
    output reg          done = 1'b0
);
    localparam [2:0] S_IDLE=0, S_SCAN=1, S_UPD=2, S_SEL=3, S_DIV=4, S_FIN=5;

    reg [2:0]   st = S_IDLE;
    reg [415:0] hw = 416'd0;
    reg [1:0]   r  = 2'd0;
    reg [5:0]   b  = 6'd0;
    reg [14:0]  best = 15'd0;
    reg [4:0]   bp   = 5'd0;

    reg [4:0]  pb0 = 0, pb1 = 0, pb2 = 0;
    reg [14:0] pm0 = 0, pm1 = 0, pm2 = 0;
    reg signed [14:0] pd0 = 0, pd1 = 0, pd2 = 0;

    function [12:0] hv;
        input [415:0] v;
        input [4:0]   i;
        hv = v[i*13 +: 13];
    endfunction

    wire [4:0] bm1 = b[4:0] - 5'd1;
    wire [4:0] bp1 = b[4:0] + 5'd1;
    wire [14:0] wsum = {2'b0, hv(hw, bm1)} + {2'b0, hv(hw, b[4:0])} + {2'b0, hv(hw, bp1)};

    wire [4:0] pm1_i = bp - 5'd1;
    wire [4:0] pp1_i = bp + 5'd1;
    wire signed [14:0] dnum = $signed({2'b0, hv(hw, pp1_i)}) - $signed({2'b0, hv(hw, pm1_i)});

    // ---- selection of the tilt peak -------------------------------------------
    function signed [5:0] tbf;               // ((bp+8) mod 16) - 8
        input [4:0] p;
        reg [4:0] t;
        begin t = (p + 5'd8) & 5'd15; tbf = $signed({1'b0, t}) - 6'sd8; end
    endfunction
    function [5:0] absf;
        input signed [5:0] v;
        absf = v[5] ? -v : v;
    endfunction
    function [4:0] cd;                       // circular distance (mod 32)
        input [4:0] x, y;
        reg [4:0] d;
        begin d = (x > y) ? x - y : y - x; cd = (d > 5'd16) ? 5'd0 - d : d; end
    endfunction

    wire [14:0] mmax = (pm0 >= pm1 && pm0 >= pm2) ? pm0 : (pm1 >= pm2 ? pm1 : pm2);
    wire [17:0] mx5  = {3'b0, mmax} * 5;
    wire q0 = ({3'b0, pm0} * 8) >= mx5;
    wire q1 = ({3'b0, pm1} * 8) >= mx5;
    wire q2 = ({3'b0, pm2} * 8) >= mx5;
    wire signed [5:0] t0 = tbf(pb0), t1 = tbf(pb1), t2 = tbf(pb2);
    reg signed [5:0]  sel_tb;
    reg [14:0]        sel_m;
    reg signed [14:0] sel_d;
    reg               have;
    always @(*) begin
        sel_tb = 6'sd0; sel_m = 15'd0; sel_d = 15'sd0; have = 1'b0;
        // first qualifying peak, then replace on strictly smaller |tb|
        if (q0) begin sel_tb = t0; sel_m = pm0; sel_d = pd0; have = 1'b1; end
        if (q1 && (!have || absf(t1) < absf(sel_tb))) begin
            sel_tb = t1; sel_m = pm1; sel_d = pd1; have = 1'b1;
        end
        if (q2 && (!have || absf(t2) < absf(sel_tb))) begin
            sel_tb = t2; sel_m = pm2; sel_d = pd2; have = 1'b1;
        end
    end

    // ---- restoring divider: |dn| * 1024 / m ------------------------------------
    reg [23:0] dnum_r = 24'd0, quo = 24'd0;
    reg [24:0] rem = 25'd0, den = 25'd0;
    reg [4:0]  dcnt = 5'd0;
    reg        dneg = 1'b0;
    reg signed [5:0] tb_r = 6'sd0;
    wire [24:0] rem_s = {rem[23:0], dnum_r[23]};
    wire        ge_d  = (rem_s >= den);

    wire signed [26:0] frac_s = dneg ? -$signed({3'b0, quo}) : $signed({3'b0, quo});
    wire signed [26:0] a_full = $signed({{21{tb_r[5]}}, tb_r}) * 27'sd1024 + frac_s;

    always @(posedge clk) begin
        done <= 1'b0;
        case (st)
            S_IDLE: if (start) begin
                hw <= hsnap; r <= 2'd0; b <= 6'd0; st <= S_SCAN;
            end
            S_SCAN: begin
                if (b == 6'd0 || wsum > best) begin best <= wsum; bp <= b[4:0]; end
                b <= b + 6'd1;
                if (b == 6'd31) st <= S_UPD;
            end
            S_UPD: begin
                case (r)
                    2'd0: begin pb0 <= bp; pm0 <= best; pd0 <= dnum; end
                    2'd1: begin pb1 <= bp; pm1 <= best; pd1 <= dnum; end
                    default: begin pb2 <= bp; pm2 <= best; pd2 <= dnum; end
                endcase
                hw[bp*13 +: 13]       <= 13'd0;
                hw[pm1_i*13 +: 13]    <= 13'd0;
                hw[pp1_i*13 +: 13]    <= 13'd0;
                r <= r + 2'd1;
                b <= 6'd0;
                st <= (r == 2'd2) ? S_SEL : S_SCAN;
            end
            S_SEL: begin
                p3  <= {2'b0, pm0} + {2'b0, pm1} + {2'b0, pm2};
                d12 <= cd(pb0, pb1);
                d13 <= cd(pb0, pb2);
                dnum_r <= {sel_d[14] ? -sel_d[13:0] : sel_d[13:0], 10'b0};
                dneg   <= sel_d[14];
                den    <= {10'b0, sel_m};
                tb_r   <= sel_tb;
                rem <= 25'd0; quo <= 24'd0; dcnt <= 5'd0;
                st <= (sel_m == 15'd0) ? S_FIN : S_DIV;
                if (sel_m == 15'd0) quo <= 24'd0;
            end
            S_DIV: begin
                dnum_r <= {dnum_r[22:0], 1'b0};
                if (ge_d) begin rem <= rem_s - den; quo <= {quo[22:0], 1'b1}; end
                else      begin rem <= rem_s;       quo <= {quo[22:0], 1'b0}; end
                dcnt <= dcnt + 5'd1;
                if (dcnt == 5'd23) st <= S_FIN;
            end
            S_FIN: begin
                tri_alpha <= (a_full >  27'sd8192) ? 16'sd8192 :
                             (a_full < -27'sd8192) ? -16'sd8192 : a_full[15:0];
                done <= 1'b1;
                st <= S_IDLE;
            end
            default: st <= S_IDLE;
        endcase
    end
endmodule
