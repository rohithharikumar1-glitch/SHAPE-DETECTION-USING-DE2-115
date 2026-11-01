# create_revision.tcl (NEW) - makes a SEPARATE Quartus revision "shape_detect",
# so your working top_ov7670_vga revision is left untouched.
# Run from the project folder:  quartus_sh -t create_revision.tcl
# (Not run in Quartus by me - if anything errors, do the same steps in the GUI:
#  Project > Revisions > Create... , then Assignments > Settings > Files/TopLevel.)
project_open top_ov7670_vga
create_revision shape_detect -based_on top_ov7670_vga -set_current
set_global_assignment -name TOP_LEVEL_ENTITY top_shape_detect
foreach f {top_shape_detect.v pixel_coord.v shape_binarize.v shape_features.v
           shape_classifier.v shape_overlay.v lcd_ctrl.v
           shape_bgseg.v shape_orient.v shape_harm.v shape_kmom.v shape_peaks.v
           shape_bin_lut.v shape_trig_lut.v shape_tilt_lut.v} {
    set_global_assignment -name VERILOG_FILE $f
}
set_global_assignment -name SDC_FILE shape_detect.sdc
source pins_lcd.tcl
project_close
