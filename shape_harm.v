// =============================================================================
// shape_harm.v  (NEW in v3)  -  harmonics of the edge-direction histogram
//
// For k = 2, 4, 6 the histogram h[b] (32 bins of the edge-normal direction
// modulo 180 deg) is turned into one complex number
//        C_k = sum_b h[b] * exp( j * k * (b+0.5) * pi/32 )
// and a CORDIC converts it to  magnitude (x 1.6468, CORDIC gain)  and angle.
//
//        |C_2| large ... elongated shape  (ellipse; rectangle that is not square)
//        |C_4| large ... edge directions repeat every 90 deg  (square / rectangle)
//        |C_6| large ... edge directions repeat every 60 deg  (triangle)
//        all small  ... circle
//   arg(C_k) / k gives the tilt of the shape.
//
// Everything is integer arithmetic, bit-exact with ref.py.
//   n          total number of edge pixels (sum of the histogram)
//   m2,m4,m6   CORDIC magnitude  (= 1.6468 * |C_k| , |C_k| in units of 127*pixels)
//   a2,a4,a6   angle, signed 16 bit, 65536 = 360 deg
// `done` pulses once when all three are valid.   Run time about 330 clocks.
// =============================================================================
module shape_harm (
    input  wire         clk,
    input  wire         start,                // one clock pulse
    input  wire [415:0] hsnap,
    output reg  [17:0]  n  = 18'd0,
    output reg  [27:0]  m2 = 28'd0, m4 = 28'd0, m6 = 28'd0,
    output reg  signed [15:0] a2 = 16'sd0, a4 = 16'sd0, a6 = 16'sd0,
    output reg          done = 1'b0
);
    localparam [2:0] S_IDLE = 3'd0, S_MAC = 3'd1, S_PRE = 3'd2, S_ITER = 3'd3, S_STORE = 3'd4;

    reg [2:0]  st   = S_IDLE;
    reg [1:0]  kidx = 2'd0;
    reg [5:0]  b    = 6'd0;
    reg [17:0] nsum = 18'd0;
    reg signed [27:0] acc_r = 28'sd0, acc_i = 28'sd0;

    wire [12:0] hval = hsnap[b[4:0]*13 +: 13];
    wire signed [7:0] cosv, sinv;
    shape_trig_lut u_lut (.kb({kidx, b[4:0]}), .cosv(cosv), .sinv(sinv));

    wire signed [21:0] p_r = $signed({1'b0, hval}) * cosv;
    wire signed [21:0] p_i = $signed({1'b0, hval}) * sinv;

    // CORDIC (vectoring)
    reg signed [28:0] cx = 29'sd0, cy = 29'sd0;
    reg signed [16:0] cz = 17'sd0;
    reg [3:0]  it = 4'd0;

    function signed [16:0] atan_t;
        input [3:0] i;
        case (i)
            4'd0:  atan_t = 17'sd8192;
            4'd1:  atan_t = 17'sd4836;
            4'd2:  atan_t = 17'sd2555;
            4'd3:  atan_t = 17'sd1297;
            4'd4:  atan_t = 17'sd651;
            4'd5:  atan_t = 17'sd326;
            4'd6:  atan_t = 17'sd163;
            4'd7:  atan_t = 17'sd81;
            4'd8:  atan_t = 17'sd41;
            4'd9:  atan_t = 17'sd20;
            4'd10: atan_t = 17'sd10;
            4'd11: atan_t = 17'sd5;
            4'd12: atan_t = 17'sd3;
            default: atan_t = 17'sd1;
        endcase
    endfunction

    wire signed [28:0] sy = cy >>> it;
    wire signed [28:0] sx = cx >>> it;

    always @(posedge clk) begin
        done <= 1'b0;
        case (st)
            S_IDLE: begin
                if (start) begin
                    st <= S_MAC; kidx <= 2'd0; b <= 6'd0;
                    acc_r <= 28'sd0; acc_i <= 28'sd0; nsum <= 18'd0;
                end
            end
            S_MAC: begin
                acc_r <= acc_r + p_r;
                acc_i <= acc_i + p_i;
                if (kidx == 2'd0) nsum <= nsum + hval;
                if (b == 6'd31) st <= S_PRE;
                b <= b + 6'd1;
            end
            S_PRE: begin
                // bring the vector into the right half plane (x >= 0)
                if (acc_r[27]) begin
                    if (!acc_i[27]) begin cx <= acc_i;  cy <= -acc_r; cz <= 17'sd16384;  end
                    else            begin cx <= -acc_i; cy <= acc_r;  cz <= -17'sd16384; end
                end else begin
                    cx <= acc_r; cy <= acc_i; cz <= 17'sd0;
                end
                it <= 4'd0;
                st <= S_ITER;
            end
            S_ITER: begin
                if (!cy[28]) begin
                    cx <= cx + sy;  cy <= cy - sx;  cz <= cz + atan_t(it);
                end else begin
                    cx <= cx - sy;  cy <= cy + sx;  cz <= cz - atan_t(it);
                end
                it <= it + 4'd1;
                if (it == 4'd13) st <= S_STORE;
            end
            S_STORE: begin
                case (kidx)
                    2'd0: begin m2 <= cx[27:0]; a2 <= cz[15:0]; end
                    2'd1: begin m4 <= cx[27:0]; a4 <= cz[15:0]; end
                    default: begin m6 <= cx[27:0]; a6 <= cz[15:0]; end
                endcase
                if (kidx == 2'd2) begin
                    n <= nsum;
                    done <= 1'b1;
                    st <= S_IDLE;
                end else begin
                    kidx <= kidx + 2'd1; b <= 6'd0;
                    acc_r <= 28'sd0; acc_i <= 28'sd0;
                    st <= S_MAC;
                end
            end
            default: st <= S_IDLE;
        endcase
    end
endmodule
