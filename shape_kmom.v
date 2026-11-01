// =============================================================================
// shape_kmom.v  (NEW in v3)  -  affine-invariant triangle test
//
// From the moment sums of the solid object pixels
//      A = N,  Sx, Sy, Sxx, Syy, Sxy
//   N20 = A*Sxx - Sx^2     N02 = A*Syy - Sy^2     N11 = A*Sxy - Sx*Sy
//   det = N20*N02 - N11^2
// the number  K = A^6 / det  does not change when the shape is rotated, scaled
// or sheared:   triangle ~ 108,  rectangle ~ 144,  ellipse ~ 158.
// So a triangle of ANY kind (equilateral, right-angled, scalene) is recognised by
//      A^6 < 125 * det          (is_tri = 1)
// All products are done by ONE 56x56 shift-and-add multiplier, one step per
// 57 clocks (about 650 clocks for the whole test - a frame lasts 400 000).
// Bit-exact with ref2.k_is_tri().
// =============================================================================
module shape_kmom (
    input  wire        clk,
    input  wire        start,
    input  wire [16:0] a,
    input  wire [24:0] sx, sy,
    input  wire [35:0] sxx, syy, sxy,
    output reg         is_tri = 1'b0,
    output reg         done = 1'b0
);
    reg [111:0] mcand = 112'd0;
    reg [55:0]  mplr  = 56'd0;
    reg [111:0] acc   = 112'd0;
    reg [5:0]   cnt   = 6'd0;
    reg [3:0]   step  = 4'd0;
    reg         busy  = 1'b0;      // multiplier running
    reg         run   = 1'b0;      // sequence running

    reg [111:0] t1 = 112'd0, t2 = 112'd0, t3 = 112'd0, t4 = 112'd0, t5 = 112'd0, t6 = 112'd0;
    reg [111:0] d1 = 112'd0, d2 = 112'd0, a2 = 112'd0, a3 = 112'd0;
    reg [111:0] n20 = 112'd0, n02 = 112'd0, n11 = 112'd0;

    // operands of the next multiplication
    reg [55:0] opa, opb;
    always @(*) begin
        opa = 56'd0; opb = 56'd0;
        case (step)
            4'd0:  begin opa = {39'd0, a};  opb = {20'd0, sxx}; end
            4'd1:  begin opa = {31'd0, sx}; opb = {31'd0, sx};  end
            4'd2:  begin opa = {39'd0, a};  opb = {20'd0, syy}; end
            4'd3:  begin opa = {31'd0, sy}; opb = {31'd0, sy};  end
            4'd4:  begin opa = {39'd0, a};  opb = {20'd0, sxy}; end
            4'd5:  begin opa = {31'd0, sx}; opb = {31'd0, sy};  end
            4'd7:  begin opa = n20[55:0];   opb = n02[55:0];    end
            4'd8:  begin opa = n11[55:0];   opb = n11[55:0];    end   // |N11|
            4'd9:  begin opa = {39'd0, a};  opb = {39'd0, a};   end
            4'd10: begin opa = a2[55:0];    opb = {39'd0, a};   end
            default: begin opa = a3[55:0];  opb = a3[55:0];     end   // step 11
        endcase
    end

    wire [111:0] det    = d1 - d2;
    wire [111:0] rhs    = (det << 7) - (det << 1) - det;          // 125 * det
    wire         det_ok = (d1 > d2);
    wire [111:0] n11_w  = (t5 >= t6) ? (t5 - t6) : (t6 - t5);
    wire [111:0] prod   = acc + (mplr[0] ? mcand : 112'd0);       // result at the last iteration

    always @(posedge clk) begin
        done <= 1'b0;
        if (start) begin
            run <= 1'b1; busy <= 1'b0; step <= 4'd0;
        end else if (run) begin
            if (step == 4'd6) begin                  // no multiplication: form N20, N02, |N11|
                n20 <= t1 - t2;
                n02 <= t3 - t4;
                n11 <= n11_w;
                step <= 4'd7;
            end else if (!busy) begin
                mcand <= {56'd0, opa};
                mplr  <= opb;
                acc   <= 112'd0;
                cnt   <= 6'd0;
                busy  <= 1'b1;
            end else begin
                if (mplr[0]) acc <= acc + mcand;
                mcand <= mcand << 1;
                mplr  <= mplr >> 1;
                cnt   <= cnt + 6'd1;
                if (cnt == 6'd55) begin
                    busy <= 1'b0;
                    step <= step + 4'd1;
                    case (step)
                        4'd0:  t1 <= prod;
                        4'd1:  t2 <= prod;
                        4'd2:  t3 <= prod;
                        4'd3:  t4 <= prod;
                        4'd4:  t5 <= prod;
                        4'd5:  t6 <= prod;
                        4'd7:  d1 <= prod;
                        4'd8:  d2 <= prod;
                        4'd9:  a2 <= prod;
                        4'd10: a3 <= prod;
                        default: begin              // step 11: A^6 = prod
                            is_tri <= det_ok && (prod < rhs);
                            done <= 1'b1;
                            run  <= 1'b0;
                        end
                    endcase
                end
            end
        end
    end
endmodule
