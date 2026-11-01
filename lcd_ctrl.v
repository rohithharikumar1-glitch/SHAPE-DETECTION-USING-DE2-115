// =============================================================================
// lcd_ctrl.v  (NEW)  -  DE2-115 16x2 character LCD (HD44780) text driver
//   Line 1 : "Detected shape:"
//   Line 2 : shape name  (NO OBJECT / CIRCLE / SQUARE / RECTANGLE /
//                         TRIANGLE / ELLIPSE)
// Write-only (RW = 0), 8-bit mode, 50 MHz clock.  Rewrites both lines forever.
// =============================================================================
module lcd_ctrl (
    input  wire       clk,            // 50 MHz
    input  wire [2:0] shape,
    output reg  [7:0] LCD_DATA = 8'h00,
    output reg        LCD_RS   = 1'b0,
    output reg        LCD_EN   = 1'b0,
    output wire       LCD_RW,
    output wire       LCD_ON
);
    assign LCD_RW = 1'b0;
    assign LCD_ON = 1'b1;

    // synchronise shape code into this clock domain
    reg [2:0] s1 = 3'd0, s2 = 3'd0;
    always @(posedge clk) begin s1 <= shape; s2 <= s1; end

    function [127:0] name_of;           // exactly 16 characters each
        input [2:0] s;
        case (s)
            3'd1:    name_of = "CIRCLE          ";
            3'd2:    name_of = "SQUARE          ";
            3'd3:    name_of = "RECTANGLE       ";
            3'd4:    name_of = "TRIANGLE        ";
            3'd5:    name_of = "ELLIPSE         ";
            default: name_of = "NO OBJECT       ";
        endcase
    endfunction

    wire [127:0] L1 = "Detected shape: ";
    wire [127:0] L2 = name_of(s2);

    // step: 0..4 init, 5 = addr line1, 6..21 = line1 chars, 22 = addr line2, 23..38 = line2 chars
    reg [5:0]  step = 6'd0;
    reg [19:0] cnt  = 20'd0;
    reg [1:0]  ph   = 2'd0;
    reg        pwr  = 1'b0;             // power-up wait finished

    reg [7:0] cur_byte;
    reg       cur_rs;
    reg [19:0] cur_wait;
    reg [4:0] ci;

    always @(*) begin
        cur_rs   = 1'b0;
        cur_wait = 20'd2500;            // 50 us
        cur_byte = 8'h00;
        ci       = 5'd0;
        case (step)
            6'd0: begin cur_byte = 8'h38; cur_wait = 20'd250000; end  // function set, 5 ms
            6'd1: begin cur_byte = 8'h38; cur_wait = 20'd10000;  end
            6'd2: begin cur_byte = 8'h0C; end                         // display on, cursor off
            6'd3: begin cur_byte = 8'h01; cur_wait = 20'd100000; end  // clear, 2 ms
            6'd4: begin cur_byte = 8'h06; end                         // entry mode
            6'd5: begin cur_byte = 8'h80; end                         // DDRAM 0x00
            6'd22:begin cur_byte = 8'hC0; end                         // DDRAM 0x40
            default: begin
                cur_rs = 1'b1;
                if (step < 6'd22) begin ci = step - 6'd6;  cur_byte = L1[127 - 8*ci -: 8]; end
                else              begin ci = step - 6'd23; cur_byte = L2[127 - 8*ci -: 8]; end
            end
        endcase
    end

    always @(posedge clk) begin
        if (!pwr) begin                       // ~20 ms after configuration
            cnt <= cnt + 1'b1;
            if (cnt == 20'd999_999) begin pwr <= 1'b1; cnt <= 20'd0; ph <= 2'd0; end
        end else begin
            case (ph)
                2'd0: begin                   // set up data, EN low
                    LCD_RS <= cur_rs; LCD_DATA <= cur_byte; LCD_EN <= 1'b0;
                    cnt <= cnt + 1'b1;
                    if (cnt == 20'd9) begin cnt <= 20'd0; ph <= 2'd1; end
                end
                2'd1: begin                   // EN high ~600 ns
                    LCD_EN <= 1'b1;
                    cnt <= cnt + 1'b1;
                    if (cnt == 20'd29) begin cnt <= 20'd0; ph <= 2'd2; end
                end
                2'd2: begin                   // EN low, wait for command to finish
                    LCD_EN <= 1'b0;
                    cnt <= cnt + 1'b1;
                    if (cnt == cur_wait) begin
                        cnt <= 20'd0; ph <= 2'd0;
                        step <= (step == 6'd38) ? 6'd5 : step + 1'b1;
                    end
                end
                default: ph <= 2'd0;
            endcase
        end
    end
endmodule
