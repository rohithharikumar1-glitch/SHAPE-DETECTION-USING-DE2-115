// =============================================================================
// shape_binarize.v  (NEW)  -  grey level -> object / background decision
//   Uses the 8-bit grey value that the EXISTING RGB2GREY module already makes
//   (range 0..221).  Threshold = thr_sel * 15  (thr_sel = SW[7:4], 0..15).
//   invert = 0 : object is DARKER  than background (dark object, white paper)
//   invert = 1 : object is BRIGHTER than background
// =============================================================================
module shape_binarize (
    input  wire [7:0] grey,
    input  wire [3:0] thr_sel,
    input  wire       invert,
    output wire       bin          // 1 = object pixel
);
    wire [7:0] thr = {thr_sel, 4'b0000} - {4'b0000, thr_sel};   // 16n - n = 15n
    assign bin = invert ? (grey > thr) : (grey < thr);
endmodule
