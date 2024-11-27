#####################################################################
#   Includes
#####################################################################

[include eddy.cfg]
[include led.cfg]
[include KAMP_Settings.cfg]
[include mainsail.cfg]
[include macros.cfg]
[include knomi.cfg]

#####################################################################
#   Printer Settings
#####################################################################

[mcu]
serial: /dev/serial/by-id/usb-Klipper_stm32f446xx_3E001C000650535556323420-if00
restart_method: command

[mcu can0]
canbus_uuid: 5cc8126cd5b5

[printer]
kinematics: corexy
max_velocity: 700  
max_accel: 30000
#minimum_cruise_ratio: 0       #Max 4000
max_z_velocity: 20          #Max 15 for 12V TMC Drivers, can increase for 24V
max_z_accel: 400
square_corner_velocity: 6.0

[gcode_arcs]
resolution: 0.1  # Optional, sets the resolution for arc approximation in mm

[virtual_sdcard]
path: ~/printer_data/gcodes
on_error_gcode: CANCEL_PRINT

[temperature_sensor raspberry_pi]
sensor_type: temperature_host
max_temp: 100

[temperature_sensor mcu_temp]
sensor_type: temperature_mcu
max_temp: 100

[gcode_macro _global_var]
variable_heat_soak_time: 5  # Default heat soak time in minutes
variable_bed_mesh_calibrate_target_temp: 60  # Default mesh calibration temperature
gcode:
    # This is a placeholder to satisfy Klipper's requirement for a gcode option
    RESPOND MSG="This macro is for storing global variables only."

#####################################################################
#   Input Shaper Configuration
#####################################################################

[adxl345]
cs_pin: can0: PB12
spi_software_sclk_pin: can0: PB10
spi_software_mosi_pin: can0: PB11
spi_software_miso_pin: can0: PB2
axes_map: z,-y,x

[resonance_tester]
probe_points: 100, 100, 20
accel_chip: adxl345

#####################################################################
#   X/Y Stepper Motor Settings
#####################################################################

[stepper_x]
step_pin: PF13
dir_pin: PF12
enable_pin: !PF14
rotation_distance: 40
microsteps: 32
full_steps_per_rotation: 200
endstop_pin: tmc2209_stepper_x:virtual_endstop
position_min: 0
position_endstop: 345
position_max: 350
homing_speed: 20
homing_positive_dir: true
homing_retract_dist: 1.5

[tmc2209 stepper_x]
#interpolate = True
uart_pin: PC4
interpolate: false
run_current: 1.2
sense_resistor: 0.110
stealthchop_threshold: 0
diag_pin: ^PG6
driver_SGTHRS: 50

[stepper_y]
step_pin: PG0
dir_pin: PG1
enable_pin: !PF15
rotation_distance: 40.2
microsteps: 32
full_steps_per_rotation:200
endstop_pin: tmc2209_stepper_y:virtual_endstop
position_min: 0
position_endstop: 361
position_max: 361
homing_speed: 27
homing_retract_dist: 0
homing_positive_dir: true

[tmc2209 stepper_y]
uart_pin: PD11
interpolate: false
run_current: 1.2
sense_resistor: 0.110
stealthchop_threshold: 0
diag_pin: ^PG9
driver_SGTHRS: 76

#####################################################################
#   Z Stepper Motor Settings
#####################################################################

[safe_z_home]
home_xy_position: 175,175
speed: 50
z_hop: 10
z_hop_speed: 5

[stepper_z]
step_pin: PF11
dir_pin: PG3
enable_pin: !PG5
rotation_distance: 39.7
gear_ratio: 80:16
microsteps: 32
endstop_pin: probe:z_virtual_endstop   
position_min: -5
position_max: 330
homing_speed: 10
second_homing_speed: 5
homing_retract_dist: 0

[tmc2209 stepper_z]
uart_pin: PC6
diag_pin: ^PG10
interpolate: false
run_current: 0.9
sense_resistor: 0.110
stealthchop_threshold: 0
driver_SGTHRS: 100

[stepper_z1]
step_pin: PG4
dir_pin: !PC1
enable_pin: !PA0
rotation_distance: 39.7
gear_ratio: 80:16
microsteps: 32

[tmc2209 stepper_z1]
uart_pin: PC7
interpolate: false
run_current: 0.9
sense_resistor: 0.110
stealthchop_threshold: 0

[stepper_z2]
step_pin: PF9
dir_pin: PF10
enable_pin: !PG2
rotation_distance: 39.7
gear_ratio: 80:16
microsteps: 32

[tmc2209 stepper_z2]
uart_pin: PF2
interpolate: false
run_current: 0.9
sense_resistor: 0.110
stealthchop_threshold: 0

[stepper_z3]
step_pin: PC13
dir_pin: !PF0
enable_pin: !PF1
rotation_distance: 39.7
gear_ratio: 80:16
microsteps: 32

[tmc2209 stepper_z3]
uart_pin: PE4
interpolate: false
run_current: 0.9
sense_resistor: 0.110
stealthchop_threshold: 0

#####################################################################
#   Extruder Settings
#####################################################################

[filament_switch_sensor filament_sensor]
pause_on_runout: True
runout_gcode: PAUSE
insert_gcode: RESUME
switch_pin: can0:PB8

[firmware_retraction]
retract_length: 0.6
retract_speed: 35
unretract_extra_length: 0
unretract_speed: 35

[extruder]
step_pin: can0: PD0
max_extrude_only_distance: 1000.0
max_extrude_cross_section: 1000.0
dir_pin: can0: PD1
enable_pin: !can0: PD2
rotation_distance: 46.515828537772
gear_ratio: 9:1
microsteps: 32
full_steps_per_rotation: 200
nozzle_diameter: 0.400
filament_diameter: 1.75
heater_pin: can0: PB13
sensor_type: NTC 100K MGB18-104F39050L32
sensor_pin: can0: PA3
min_temp: 10
max_temp: 400
max_power: 1.0
min_extrude_temp: 170
pressure_advance: 0.04

[tmc2209 extruder]
uart_pin: can0:PA15
interpolate: true
run_current: 0.90
hold_current: 0.6
sense_resistor: 0.110
stealthchop_threshold: 0

#####################################################################
#   Bed Heater Settings
#####################################################################

[heater_bed]
heater_pin: PA3
sensor_type: EPCOS 100K B57560G104F
sensor_pin: PF3
max_power: 1
min_temp: 0
max_temp: 115

#####################################################################
#   Fan Control
#####################################################################

[fan]
pin: can0: PA1
kick_start_time: 0.5
max_power: 1
off_below: 0.10

[heater_fan hotend_fan]
pin: can0: PA0
max_power: 1.0
kick_start_time: 0.5
heater: extruder
heater_temp: 50.0

[controller_fan controller_fan]
pin: PD12
kick_start_time: 0.5
max_power: 0.5
heater: heater_bed

[controller_fan controller_fan2]
pin: PD13
kick_start_time: 0.5
max_power: 0.5
heater: heater_bed

[heater_fan nevermore_fan]
pin: PD14
max_power: 1.0
kick_start_time: 5.0
heater: extruder
heater_temp: 240.0

#####################################################################
#   Homing and Gantry Adjustment Routines
#####################################################################

[idle_timeout]
timeout: 1800

[quad_gantry_level]
gantry_corners:
   -60,-10
   410,420
points:
   25, 8.5
   25, 308.5
   325, 308.5
   325, 8.5
speed: 400
horizontal_move_z: 5
retry_tolerance: 0.05
retries: 5
max_adjust: 30

#####################################################################
#   Display and Pin Configuration
#####################################################################

[board_pins]
aliases:
    # EXP1 header
    EXP1_1=PE8, EXP1_2=PE7,
    EXP1_3=PE9, EXP1_4=PE10,
    EXP1_5=PE12, EXP1_6=PE13,
    EXP1_7=PE14, EXP1_8=PE15,
    EXP1_9=<GND>, EXP1_10=<5V>,

    # EXP2 header
    EXP2_1=PA6, EXP2_2=PA5,
    EXP2_3=PB1, EXP2_4=PA4,
    EXP2_5=PB2, EXP2_6=PA7,
    EXP2_7=PC15, EXP2_8=<RST>,
    EXP2_9=<GND>, EXP2_10=<5V>

[display]
lcd_type: uc1701
cs_pin: EXP1_3
a0_pin: EXP1_4
rst_pin: EXP1_5
encoder_pins: ^EXP2_5, ^EXP2_3
click_pin: ^!EXP1_2
contrast: 63
spi_software_miso_pin: EXP2_1
spi_software_mosi_pin: EXP2_6
spi_software_sclk_pin: EXP2_2

# [neopixel btt_mini12864]
# pin: EXP1_6
# chain_count: 3
# initial_RED: 0.1
# initial_GREEN: 0.5
# initial_BLUE: 0.0
# color_order: RGB

# [delayed_gcode setdisplayneopixel]
# initial_duration: 1
# gcode:
#     SET_LED LED=btt_mini12864 RED=1 GREEN=1 BLUE=1 INDEX=1 TRANSMIT=0
#     SET_LED LED=btt_mini12864 RED=1 GREEN=0 BLUE=0 INDEX=2 TRANSMIT=0
#     SET_LED LED=btt_mini12864 RED=1 GREEN=0 BLUE=0 INDEX=3

[exclude_object]

#*# <---------------------- SAVE_CONFIG ---------------------->
#*# DO NOT EDIT THIS BLOCK OR BELOW. The contents are auto-generated.
#*#
#*# [extruder]
#*# control = pid
#*# pid_kp = 34.519
#*# pid_ki = 11.506
#*# pid_kd = 25.889
#*#
#*# [probe]
#*#
#*# [heater_bed]
#*# control = pid
#*# pid_kp = 56.642
#*# pid_ki = 1.807
#*# pid_kd = 443.935
#*#
#*# [input_shaper]
#*# shaper_type_x = zv
#*# shaper_freq_x = 72.2
#*# shaper_type_y = mzv
#*# shaper_freq_y = 39.6
#*#
#*# [probe_eddy_current btt_eddy]
#*# reg_drive_current = 17
#*# calibrate =
#*# 	0.049625:3215825.541,0.090566:3214460.051,0.130266:3213104.334,
#*# 	0.169966:3211860.588,0.209666:3210657.236,0.250606:3209355.438,
#*# 	0.290306:3208142.417,0.330006:3206946.792,0.369706:3205809.267,
#*# 	0.409406:3204669.898,0.450347:3203508.452,0.490047:3202413.422,
#*# 	0.529747:3201372.324,0.569447:3200315.900,0.610388:3199240.355,
#*# 	0.650088:3198234.122,0.689788:3197255.373,0.729488:3196283.534,
#*# 	0.770428:3195316.822,0.810128:3194395.870,0.849828:3193498.108,
#*# 	0.889528:3192613.153,0.930469:3191728.273,0.970169:3190889.046,
#*# 	1.009869:3190049.440,1.049569:3189234.096,1.090509:3188430.160,
#*# 	1.130209:3187661.388,1.169909:3186887.542,1.209609:3186130.064,
#*# 	1.250550:3185401.604,1.290250:3184689.326,1.329950:3183994.823,
#*# 	1.369650:3183280.055,1.410591:3182596.242,1.450291:3181956.191,
#*# 	1.489991:3181298.430,1.529691:3180666.509,1.569391:3180009.972,
#*# 	1.610331:3179393.340,1.650031:3178830.792,1.689731:3178224.345,
#*# 	1.729431:3177663.435,1.770372:3177063.094,1.810072:3176516.181,
#*# 	1.849772:3175981.227,1.889472:3175464.452,1.930413:3174934.085,
#*# 	1.970113:3174404.207,2.009813:3173918.951,2.049513:3173407.698,
#*# 	2.090453:3172927.750,2.130153:3172462.165,2.169853:3172009.846,
#*# 	2.209553:3171523.321,2.250494:3171087.940,2.290194:3170642.672,
#*# 	2.329894:3170225.065,2.369594:3169813.183,2.410534:3169370.907,
#*# 	2.450234:3168987.540,2.489934:3168589.758,2.529634:3168175.965,
#*# 	2.570575:3167795.692,2.610275:3167410.020,2.649975:3167038.579,
#*# 	2.689675:3166662.127,2.730616:3166335.812,2.770316:3165955.646,
#*# 	2.810016:3165607.899,2.849716:3165280.797,2.889416:3164969.662,
#*# 	2.930356:3164627.786,2.970056:3164297.420,3.009756:3163980.720,
#*# 	3.049456:3163702.530,3.090397:3163366.787,3.130097:3163064.452,
#*# 	3.169797:3162770.042,3.209497:3162504.013,3.250438:3162200.124,
#*# 	3.290138:3161936.940,3.329838:3161646.098,3.369538:3161378.719,
#*# 	3.410478:3161123.961,3.450178:3160844.577,3.489878:3160617.898,
#*# 	3.529578:3160338.365,3.570519:3160086.345,3.610219:3159853.783,
#*# 	3.649919:3159617.742,3.689619:3159386.563,3.730559:3159127.314,
#*# 	3.770259:3158933.289,3.809959:3158693.955,3.849659:3158480.963,
#*# 	3.890600:3158250.842,3.930300:3158041.500,3.970000:3157832.841,
#*# 	4.009700:3157633.600,4.049400:3157445.807
#*#
#*# [temperature_probe btt_eddy]
#*# calibration_temp = 61.659973
#*# drift_calibration =
#*# 	3240943.203368, -589.134716, 4.941034
#*# 	3213776.929404, -233.054705, 2.018030
#*# 	3193917.331624, 0.207401, 0.092456
#*# 	3179021.889133, 170.503912, -1.342499
#*# 	3167936.055615, 286.395066, -2.320343
#*# 	3159443.113599, 372.176208, -3.048603
#*# 	3152854.249337, 437.275117, -3.613051
#*# 	3147834.526141, 479.792226, -3.970641
#*# 	3143861.849493, 511.816644, -4.235231
#*# drift_calibration_min_temp = 36.932440996638555
#*#
#*# [bed_mesh default]
#*# version = 1
#*# points =
#*# 	0.037704, 0.042393, 0.022421, 0.026553, 0.035916, 0.045106, 0.061763, 0.066636, 0.064601, 0.078069, 0.080167, 0.074464, 0.065433, 0.049296, 0.011579
#*# 	0.069876, 0.062719, 0.053761, 0.049025, 0.049703, 0.061825, 0.081320, 0.076916, 0.075468, 0.078166, 0.082443, 0.076620, 0.072011, 0.058681, 0.022265
#*# 	0.089800, 0.084769, 0.078430, 0.071863, 0.069197, 0.075764, 0.088825, 0.088824, 0.084867, 0.086343, 0.085633, 0.080817, 0.072514, 0.057848, 0.024488
#*# 	0.111536, 0.097390, 0.084867, 0.083529, 0.088620, 0.092280, 0.098440, 0.097799, 0.098189, 0.098691, 0.085517, 0.083418, 0.075765, 0.059844, 0.023266
#*# 	0.116239, 0.106997, 0.102593, 0.096112, 0.095384, 0.098041, 0.098816, 0.091751, 0.099341, 0.095588, 0.087970, 0.089121, 0.078867, 0.067014, 0.039154
#*# 	0.121545, 0.113304, 0.105045, 0.104394, 0.096089, 0.106196, 0.105045, 0.091871, 0.106847, 0.097088, 0.103205, 0.093145, 0.083418, 0.071863, 0.033880
#*# 	0.122229, 0.109447, 0.110409, 0.105045, 0.095885, 0.095588, 0.099843, 0.092485, 0.091984, 0.098663, 0.107817, 0.092931, 0.075114, 0.069505, 0.036748
#*# 	0.115394, 0.098542, 0.097538, 0.089047, 0.081468, 0.090552, 0.097464, 0.080817, 0.080817, 0.091277, 0.106196, 0.082768, 0.067315, 0.061395, 0.028031
#*# 	0.104245, 0.090330, 0.087319, 0.085220, 0.078867, 0.093433, 0.094287, 0.081844, 0.078319, 0.079369, 0.074316, 0.065957, 0.054762, 0.049858, 0.022421
#*# 	0.099527, 0.090553, 0.080817, 0.089121, 0.084068, 0.086669, 0.087023, 0.073962, 0.070462, 0.081133, 0.068827, 0.063916, 0.057230, 0.048500, 0.021045
#*# 	0.104394, 0.096535, 0.093786, 0.089270, 0.086609, 0.084718, 0.094938, 0.097390, 0.083418, 0.078512, 0.073289, 0.071716, 0.069167, 0.051369, 0.032152
#*# 	0.093786, 0.091483, 0.086874, 0.082930, 0.074966, 0.081356, 0.083418, 0.086669, 0.078217, 0.076768, 0.058956, 0.060717, 0.053406, 0.050707, 0.032962
#*# 	0.078867, 0.082768, 0.079015, 0.073164, 0.074966, 0.075764, 0.079517, 0.072691, 0.065433, 0.065742, 0.054609, 0.047976, 0.050537, 0.043225, 0.035114
#*# 	0.067917, 0.066505, 0.061361, 0.062504, 0.066112, 0.074966, 0.074316, 0.062195, 0.055287, 0.052742, 0.051894, 0.047653, 0.050537, 0.051061, 0.032307
#*# 	0.046464, 0.051369, 0.064016, 0.063028, 0.061022, 0.065487, 0.070554, 0.060253, 0.047822, 0.045262, 0.052263, 0.047144, 0.048500, 0.047652, 0.037581
#*# x_count = 15
#*# y_count = 15
#*# mesh_x_pps = 2
#*# mesh_y_pps = 2
#*# algo = bicubic
#*# tension = 0.2
#*# min_x = 15.0
#*# max_x = 334.90000000000003
#*# min_y = 22.0
#*# max_y = 331.95999999999987
