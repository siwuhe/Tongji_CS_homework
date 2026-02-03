#########################################
# Clock & Buttons
#########################################

set_property PACKAGE_PIN E3  [get_ports clk_in]     ; # 100MHz clock
set_property PACKAGE_PIN M13 [get_ports enable]     ; # BTNU
set_property PACKAGE_PIN L16 [get_ports reset]      ; # BTNC
set_property PACKAGE_PIN J15 [get_ports start]      ; # BTND
set_property PACKAGE_PIN R15 [get_ports sw_int]      ; # BTND

set_property IOSTANDARD LVCMOS33 [get_ports clk_in]
set_property IOSTANDARD LVCMOS33 [get_ports enable]
set_property IOSTANDARD LVCMOS33 [get_ports reset]
set_property IOSTANDARD LVCMOS33 [get_ports start]
set_property IOSTANDARD LVCMOS33 [get_ports sw_int]

#########################################
# Seven Segment Display (Your school template wiring)
#########################################

# Segment pins
set_property PACKAGE_PIN T10 [get_ports {o_seg[0]}]
set_property PACKAGE_PIN R10 [get_ports {o_seg[1]}]
set_property PACKAGE_PIN K16 [get_ports {o_seg[2]}]
set_property PACKAGE_PIN K13 [get_ports {o_seg[3]}]
set_property PACKAGE_PIN P15 [get_ports {o_seg[4]}]
set_property PACKAGE_PIN T11 [get_ports {o_seg[5]}]
set_property PACKAGE_PIN L18 [get_ports {o_seg[6]}]
set_property PACKAGE_PIN H15 [get_ports {o_seg[7]}]

# Digit select pins
set_property PACKAGE_PIN J17 [get_ports {o_sel[0]}]
set_property PACKAGE_PIN J18 [get_ports {o_sel[1]}]
set_property PACKAGE_PIN T9  [get_ports {o_sel[2]}]
set_property PACKAGE_PIN J14 [get_ports {o_sel[3]}]
set_property PACKAGE_PIN P14 [get_ports {o_sel[4]}]
set_property PACKAGE_PIN T14 [get_ports {o_sel[5]}]
set_property PACKAGE_PIN K2  [get_ports {o_sel[6]}]
set_property PACKAGE_PIN U13 [get_ports {o_sel[7]}]

# IOSTANDARD
set_property IOSTANDARD LVCMOS33 [get_ports {o_seg[*]}]
set_property IOSTANDARD LVCMOS33 [get_ports {o_sel[*]}]


#########################################
# Timing Constraints
#########################################

create_clock -period 90.000 -name clk_pin -waveform {0.000 50.000} [get_ports clk_in]

set_input_delay  -clock [get_clocks clk_pin] 1.000 [get_ports reset]
set_output_delay -clock [get_clocks clk_pin] 0.000 [get_ports -filter {DIRECTION == "OUT"}]
