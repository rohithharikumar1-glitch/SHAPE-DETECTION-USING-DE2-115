# =============================================================================
# pins_lcd.tcl  (NEW) - extra pin assignments for the 16x2 LCD on the DE2-115
# Run like your pins.tcl:  Tools > Tcl Scripts > pins_lcd.tcl   (project open)
#   LCD_DATA[7:0], LCD_EN, LCD_RS : checked against a published DE2-115 pin table
#   LCD_RW (M1), LCD_ON (L5)      : from my recollection of the Terasic pin list ->
#                                   VERIFY in the DE2-115 User Manual (LCD section)
#                                   before programming the board.
#   LCD_BLON is deliberately NOT used (AE22 is already your GPIO[18]).
# =============================================================================
package require ::quartus::project

proc assign_pin {name pin std} {
    set_location_assignment PIN_$pin -to $name
    set_instance_assignment -name IO_STANDARD $std -to $name
}
set STD33 "3.3-V LVTTL"

foreach {n p} {
    LCD_DATA[0] L3
    LCD_DATA[1] L1
    LCD_DATA[2] L2
    LCD_DATA[3] K7
    LCD_DATA[4] K1
    LCD_DATA[5] K2
    LCD_DATA[6] M3
    LCD_DATA[7] M5
    LCD_EN      L4
    LCD_RS      M2
    LCD_RW      M1
    LCD_ON      L5
} { assign_pin $n $p $STD33 }

puts "LCD pins assigned."
