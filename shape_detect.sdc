# shape_detect.sdc (NEW, optional) - real clock constraints.
# Your last compile had no .sdc, so Quartus assumed 1 ns clocks and printed
# large negative slack; that report was not meaningful.
create_clock -name CLOCK_50 -period 20.000 [get_ports CLOCK_50]
create_clock -name cam_pclk -period 40.000 [get_ports {GPIO[4]}]
create_generated_clock -name vga_clk -source [get_ports CLOCK_50] -divide_by 2 [get_registers {vga_clk_r}]
set_clock_groups -asynchronous -group {CLOCK_50} -group {cam_pclk} -group {vga_clk}
derive_clock_uncertainty
