/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/*
 *  ArduCopter parameter definitions
 *
 *  This firmware is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 */

#define GSCALAR(v, name, def) { g.v.vtype, name, Parameters::k_param_ ## v, &g.v, {def_value : def} }
#define GGROUP(v, name, class) { AP_PARAM_GROUP, name, Parameters::k_param_ ## v, &g.v, {group_info : class::var_info} }
#define GOBJECT(v, name, class) { AP_PARAM_GROUP, name, Parameters::k_param_ ## v, &v, {group_info : class::var_info} }

const AP_Param::Info var_info[] PROGMEM = {
    // @Param: SYSID_SW_MREV
    // @DisplayName: Eeprom format version number
    // @Description: This value is incremented when changes are made to the eeprom format
    // @User: Advanced
    GSCALAR(format_version, "SYSID_SW_MREV",   0),

    // @Param: SYSID_SW_TYPE
    // @DisplayName: Software Type
    // @Description: This is used by the ground station to recognise the software type (eg ArduPlane vs ArduCopter)
    // @User: Advanced
    GSCALAR(software_type,  "SYSID_SW_TYPE",   Parameters::k_software_type),

    // @Param: SYSID_THISMAV
    // @DisplayName: Mavlink version
    // @Description: Allows reconising the mavlink version
    // @User: Advanced
    GSCALAR(sysid_this_mav, "SYSID_THISMAV",   MAV_SYSTEM_ID),

    // @Param: SYSID_MYGCS
    // @DisplayName: My ground station number
    // @Description: Allows restricting radio overrides to only come from my ground station
    // @User: Advanced
    GSCALAR(sysid_my_gcs,   "SYSID_MYGCS",     255),

    // @Param: SERIAL3_BAUD
    // @DisplayName: Telemetry Baud Rate
    // @Description: The baud rate used on the telemetry port
    // @Values: 1:1200,2:2400,4:4800,9:9600,19:19200,38:38400,57:57600,111:111100,115:115200
    // @User: Standard
    GSCALAR(serial3_baud,   "SERIAL3_BAUD",     SERIAL3_BAUD/1000),

    // @Param: TELEM_DELAY
    // @DisplayName: Telemetry startup delay
    // @Description: The amount of time (in seconds) to delay radio telemetry to prevent an Xbee bricking on power up
    // @User: Standard
    // @Units: seconds
    // @Range: 0 10
    // @Increment: 1
    GSCALAR(telem_delay,            "TELEM_DELAY",     0),


    // @Param: SONAR_ENABLE
    // @DisplayName: Enable Sonar
    // @Description: Setting this to Enabled(1) will enable the sonar. Setting this to Disabled(0) will disable the sonar
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    GSCALAR(sonar_enabled,  "SONAR_ENABLE",     DISABLED),

    // @Param: SONAR_TYPE
    // @DisplayName: Sonar type
    // @Description: Used to adjust scaling to match the sonar used (only Maxbotix sonars are supported at this time)
    // @Values: 0:XL-EZ0,1:LV-EZ0,2:XLL-EZ0,3:HRLV
    // @User: Standard
    GSCALAR(sonar_type,     "SONAR_TYPE",           AP_RANGEFINDER_MAXSONARXL),

    // @Param: SONAR_GAIN
    // @DisplayName: Sonar gain
    // @Description: Used to adjust the speed with which the target altitude is changed when objects are sensed below the copter
    // @Range: 0.01 0.5
    // @Increment: 0.01
    // @User: Standard
    GSCALAR(sonar_gain,     "SONAR_GAIN",           SONAR_GAIN_DEFAULT),

    // @Param: BATT_MONITOR
    // @DisplayName: Battery monitoring
    // @Description: Controls enabling monitoring of the battery's voltage and current
    // @Values: 0:Disabled,3:Voltage Only,4:Voltage and Current
    // @User: Standard
    GSCALAR(battery_monitoring, "BATT_MONITOR", BATT_MONITOR_DISABLED),

    // @Param: FS_BATT_ENABLE
    // @DisplayName: Battery Failsafe Enable
    // @Description: Controls whether failsafe will be invoked when battery voltage or current runs low
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    GSCALAR(failsafe_battery_enabled, "FS_BATT_ENABLE", FS_BATTERY),

    // @Param: FS_GPS_ENABLE
    // @DisplayName: GPS Failsafe Enable
    // @Description: Controls whether failsafe will be invoked when gps signal is lost
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    GSCALAR(failsafe_gps_enabled, "FS_GPS_ENABLE", FS_GPS),

    // @Param: FS_GCS_ENABLE
    // @DisplayName: Ground Station Failsafe Enable
    // @Description: Controls whether failsafe will be invoked (and what action to take) when connection with Ground station is lost for at least 5 seconds
    // @Values: 0:Disabled,1:Enabled always RTL,2:Enabled Continue with Mission in Auto Mode
    // @User: Standard
    GSCALAR(failsafe_gcs, "FS_GCS_ENABLE", FS_GCS_DISABLED),

    // @Param: VOLT_DIVIDER
    // @DisplayName: Voltage Divider
    // @Description: Used to convert the voltage of the voltage sensing pin (BATT_VOLT_PIN) to the actual battery's voltage (pin voltage * INPUT_VOLTS/1024 * VOLT_DIVIDER)
    // @User: Advanced
    GSCALAR(volt_div_ratio, "VOLT_DIVIDER",     VOLT_DIV_RATIO),

    // @Param: AMP_PER_VOLT
    // @DisplayName: Current Amps per volt
    // @Description: Used to convert the voltage on the current sensing pin (BATT_CURR_PIN) to the actual current being consumed in amps (curr pin voltage * INPUT_VOLTS/1024 * AMP_PER_VOLT )
    // @User: Advanced
    GSCALAR(curr_amp_per_volt,      "AMP_PER_VOLT", CURR_AMP_PER_VOLT),

    // @Param: BATT_CAPACITY
    // @DisplayName: Battery Capacity
    // @Description: Battery capacity in milliamp-hours (mAh)
    // @Units: mAh
    GSCALAR(pack_capacity,  "BATT_CAPACITY",    HIGH_DISCHARGE),

    // @Param: MAG_ENABLE
    // @DisplayName: Enable Compass
    // @Description: Setting this to Enabled(1) will enable the compass. Setting this to Disabled(0) will disable the compass
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    GSCALAR(compass_enabled,        "MAG_ENABLE",   MAGNETOMETER),

    // @Param: LOW_VOLT
    // @DisplayName: Low Voltage
    // @Description: Set this to the voltage you want to represent low voltage
    // @Range: 0 20
    // @Increment: .1
    // @User: Standard
    GSCALAR(low_voltage,    "LOW_VOLT",         LOW_VOLTAGE),

    // @Param: BATT_VOLT_PIN
    // @DisplayName: Battery Voltage sensing pin
    // @Description: Setting this to 0 ~ 13 will enable battery current sensing on pins A0 ~ A13.
    // @Values: -1:Disabled, 0:A0, 1:A1, 13:A13
    // @User: Standard
    GSCALAR(battery_volt_pin,    "BATT_VOLT_PIN",    BATTERY_VOLT_PIN),

    // @Param: BATT_CURR_PIN
    // @DisplayName: Battery Current sensing pin
    // @Description: Setting this to 0 ~ 13 will enable battery current sensing on pins A0 ~ A13.
    // @Values: -1:Disabled, 1:A1, 2:A2, 12:A12
    // @User: Standard
    GSCALAR(battery_curr_pin,    "BATT_CURR_PIN",    BATTERY_CURR_PIN),

    // @Param: RSSI_PIN
    // @DisplayName: Receiver RSSI sensing pin
    // @Description: This selects an analog pin for the receiver RSSI voltage. It assumes the voltage is 5V for max rssi, 0V for minimum
    // @Values: -1:Disabled, 0:A0, 1:A1, 2:A2, 13:A13
    // @User: Standard
    GSCALAR(rssi_pin,            "RSSI_PIN",         -1),


    // @Param: WP_TOTAL
    // @DisplayName: Waypoint Total
    // @Description: Total number of commands in the mission stored in the eeprom.  Do not update this parameter directly!
    // @User: Advanced
    GSCALAR(command_total,  "WP_TOTAL",         0),

    // @Param: WP_INDEX
    // @DisplayName: Waypoint Index
    // @Description: The index number of the command that is currently being executed.  Do not update this parameter directly!
    // @User: Advanced
    GSCALAR(command_index,  "WP_INDEX",         0),

    // @Param: CIRCLE_RADIUS
    // @DisplayName: Circle radius
    // @Description: Defines the radius of the circle the vehicle will fly when in Circle flight mode
    // @Units: Meters
    // @Range: 1 127
    // @Increment: 1
    // @User: Standard
    GSCALAR(circle_radius,  "CIRCLE_RADIUS",    CIRCLE_RADIUS),

    // @Param: FLTMODE1
    // @DisplayName: Flight Mode 1
    // @Description: Flight mode when Channel 5 pwm is <= 1230
    // @User: Standard
    GSCALAR(flight_mode1, "FLTMODE1",               FLIGHT_MODE_1),

    // @Param: FLTMODE2
    // @DisplayName: Flight Mode 2
    // @Description: Flight mode when Channel 5 pwm is >1230, <= 1360
    // @User: Standard
    GSCALAR(flight_mode2, "FLTMODE2",               FLIGHT_MODE_2),

    // @Param: FLTMODE3
    // @DisplayName: Flight Mode 3
    // @Description: Flight mode when Channel 5 pwm is >1360, <= 1490
    // @User: Standard
    GSCALAR(flight_mode3, "FLTMODE3",               FLIGHT_MODE_3),

    // @Param: FLTMODE4
    // @DisplayName: Flight Mode 4
    // @Description: Flight mode when Channel 5 pwm is >1490, <= 1620
    // @User: Standard
    GSCALAR(flight_mode4, "FLTMODE4",               FLIGHT_MODE_4),

    // @Param: FLTMODE5
    // @DisplayName: Flight Mode 5
    // @Description: Flight mode when Channel 5 pwm is >1620, <= 1749
    // @User: Standard
    GSCALAR(flight_mode5, "FLTMODE5",               FLIGHT_MODE_5),

    // @Param: FLTMODE6
    // @DisplayName: Flight Mode 6
    // @Description: Flight mode when Channel 5 pwm is >=1750
    // @User: Standard
    GSCALAR(flight_mode6, "FLTMODE6",               FLIGHT_MODE_6),

    // @Param: SIMPLE
    // @DisplayName: Simple mode bitmask
    // @Description: Bitmask which holds which flight modes use simple heading mode (eg bit 0 = 1 means Flight Mode 0 uses simple mode)
    // @User: Advanced
    GSCALAR(simple_modes, "SIMPLE",                 0),

    // @Param: LOG_BITMASK
    // @DisplayName: Log bitmask
    // @Description: 2 byte bitmap of log types to enable
    // @User: Advanced
    GSCALAR(log_bitmask,    "LOG_BITMASK",          DEFAULT_LOG_BITMASK),

    // @Param: TUNE
    // @DisplayName: Channel 6 Tuning
    // @Description: Controls which parameters (normally PID gains) are being tuned with transmitter's channel 6 knob
    // @User: Standard
    // @Values: 0:CH6_NONE,1:CH6_STABILIZE_KP,2:CH6_STABILIZE_KI,3:CH6_YAW_KP,4:CH6_RATE_KP,5:CH6_RATE_KI,6:CH6_YAW_RATE_KP,7:CH6_THROTTLE_KP,8:CH6_TOP_BOTTOM_RATIO,9:CH6_RELAY,10:CH6_WP_SPEED,12:CH6_LOITER_KP,13:CH6_HELI_EXTERNAL_GYRO,14:CH6_THR_HOLD_KP,17:CH6_OPTFLOW_KP,18:CH6_OPTFLOW_KI,19:CH6_OPTFLOW_KD,21:CH6_RATE_KD,22:CH6_LOITER_RATE_KP,23:CH6_LOITER_RATE_KD,24:CH6_YAW_KI,25:CH6_ACRO_KP,26:CH6_YAW_RATE_KD,27:CH6_LOITER_KI,28:CH6_LOITER_RATE_KI,29:CH6_STABILIZE_KD,30:CH6_AHRS_YAW_KP,31:CH6_AHRS_KP,32:CH6_INAV_TC,33:CH6_THROTTLE_KI,34:CH6_THR_ACCEL_KP,35:CH6_THR_ACCEL_KI,36:CH6_THR_ACCEL_KD,38:CH6_DECLINATION,39:CH6_CIRCLE_RATE
    GSCALAR(radio_tuning, "TUNE",                   0),

    // @Param: TUNE_LOW
    // @DisplayName: Tuning minimum
    // @Description: The minimum value that will be applied to the parameter currently being tuned with the transmitter's channel 6 knob
    // @User: Standard
    // @Range: 0 32767
    GSCALAR(radio_tuning_low, "TUNE_LOW",           0),

    // @Param: TUNE_HIGH
    // @DisplayName: Tuning maximum
    // @Description: The maximum value that will be applied to the parameter currently being tuned with the transmitter's channel 6 knob
    // @User: Standard
    // @Range: 0 32767
    GSCALAR(radio_tuning_high, "TUNE_HIGH",         1000),

    // @Param: CH7_OPT
    // @DisplayName: Channel 7 option
    // @Description: Select which function if performed when CH7 is above 1800 pwm
    // @Values: 0:Do Nothing, 2:Flip, 3:Simple Mode, 4:RTL, 5:Save Trim, 7:Save WP, 9:Camera Trigger, 10:Sonar
    // @User: Standard
    GSCALAR(ch7_option, "CH7_OPT",                  CH7_OPTION),

	// @Param: AUTO_SLEW
    // @DisplayName: Auto Slew Rate
    // @Description: This restricts the rate of change of the roll and pitch attitude commanded by the auto pilot
    // @Units: Degrees/Second
	// @Range: 1 90
    // @Increment: 1
    // @User: Advanced
    GSCALAR(auto_slew_rate, "AUTO_SLEW",            AUTO_SLEW_RATE),

    // RC channel
    //-----------
    // @Group: RC1_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp
    GGROUP(rc_1,    "RC1_", RC_Channel),
    // @Group: RC2_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp
    GGROUP(rc_2,    "RC2_", RC_Channel),
    // @Group: RC3_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp
    GGROUP(rc_3,    "RC3_", RC_Channel),
    // @Group: RC4_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp
    GGROUP(rc_4,    "RC4_", RC_Channel),
    // @Group: RC5_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp,../libraries/RC_Channel/RC_Channel_aux.cpp
    GGROUP(rc_5,    "RC5_", RC_Channel_aux),
    // @Group: RC6_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp,../libraries/RC_Channel/RC_Channel_aux.cpp
    GGROUP(rc_6,    "RC6_", RC_Channel_aux),
    // @Group: RC7_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp,../libraries/RC_Channel/RC_Channel_aux.cpp
    GGROUP(rc_7,    "RC7_", RC_Channel_aux),
    // @Group: RC8_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp,../libraries/RC_Channel/RC_Channel_aux.cpp
    GGROUP(rc_8,    "RC8_", RC_Channel_aux),

#if MOUNT == ENABLED
    // @Group: RC10_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp,../libraries/RC_Channel/RC_Channel_aux.cpp
    GGROUP(rc_10,                    "RC10_", RC_Channel_aux),

    // @Group: RC11_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp,../libraries/RC_Channel/RC_Channel_aux.cpp
    GGROUP(rc_11,                    "RC11_", RC_Channel_aux),
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
    // @Group: RC9_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp,../libraries/RC_Channel/RC_Channel_aux.cpp
    GGROUP(rc_9,                    "RC9_", RC_Channel_aux),
    // @Group: RC12_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp,../libraries/RC_Channel/RC_Channel_aux.cpp
    GGROUP(rc_12,                   "RC12_", RC_Channel_aux),
#endif

    // @Param: RC_SPEED
    // @DisplayName: ESC Update Speed
    // @Description: This is the speed in Hertz that your ESCs will receive updates
    // @Units: Hertz (Hz)
    // @Values: 125,400,490
    // @User: Advanced
    GSCALAR(rc_speed, "RC_SPEED",              RC_FAST_SPEED),


    // @Param: LED_MODE
    // @DisplayName: Copter LED Mode
    // @Description: bitmap to control the copter led mode
    // @Values: 0:Disabled,1:Enable,2:GPS On,4:Aux,8:Buzzer,16:Oscillate,32:Nav Blink,64:GPS Nav Blink
    // @User: Standard
    GSCALAR(copter_leds_mode,       "LED_MODE",         9),


    // @Param: RATE_RLL_D
    // @DisplayName: Roll axis rate controller D gain
    // @Description: Roll axis rate controller D gain.  Compensates for short-term change in desired roll rate vs actual roll rate
    // @Range: 0.001 0.008
    // @User: Standard
    GGROUP(pid_rate_roll,     "RATE_RLL_", AC_PID),

    // @Param: RATE_PIT_D
    // @DisplayName: Pitch axis rate controller D gain
    // @Description: Pitch axis rate controller D gain.  Compensates for short-term change in desired pitch rate vs actual pitch rate
    // @Range: 0.001 0.008
    // @User: Standard
    GGROUP(pid_rate_pitch,    "RATE_PIT_", AC_PID),

    // @Param: RATE_YAW_D
    // @DisplayName: Yaw axis rate controller D gain
    // @Description: Yaw axis rate controller D gain.  Compensates for short-term change in desired yaw rate vs actual yaw rate
    // @Range: 0.000 0.001
    // @User: Standard
    GGROUP(pid_rate_yaw,      "RATE_YAW_", AC_PID),

    // @Param: LOITER_LAT_D
    // @DisplayName: Loiter latitude rate controller D gain
    // @Description: Loiter latitude rate controller D gain.  Compensates for short-term change in desired speed vs actual speed
    // @Range: 0.200 0.600
    // @User: Standard
    GGROUP(pid_loiter_rate_lat,      "LOITER_LAT_",  AC_PID),

    // @Param: LOITER_LON_D
    // @DisplayName: Loiter longituderate controller D gain
    // @Description: Loiter longitude rate controller D gain.  Compensates for short-term change in desired speed vs actual speed
    // @Range: 0.200 0.600
    // @User: Standard
    GGROUP(pid_loiter_rate_lon,      "LOITER_LON_",  AC_PID),

    // @Param: THR_RATE_D
    // @DisplayName: Throttle rate controller D gain
    // @Description: Throttle rate controller D gain.  Compensates for short-term change in desired vertical speed vs actual speed
    // @Range: 0.000 0.400
    // @User: Standard
    GGROUP(pid_throttle,      "THR_RATE_", AC_PID),

    // @Param: THR_ACCEL_P
    // @DisplayName: Throttle acceleration controller P gain
    // @Description: Throttle acceleration controller P gain.  Converts the difference between desired vertical acceleration and actual acceleration into a motor output
    // @Range: 0.500 1.500
    // @User: Standard

    // @Param: THR_ACCEL_I
    // @DisplayName: Throttle acceleration controller I gain
    // @Description: Throttle acceleration controller I gain.  Corrects long-term difference in desired vertical acceleration and actual acceleration
    // @Range: 0.000 3.000
    // @User: Standard

    // @Param: THR_ACCEL_IMAX
    // @DisplayName: Throttle acceleration controller I gain maximum
    // @Description: Throttle acceleration controller I gain maximum.  Constrains the maximum pwm that the I term will generate
    // @Range: 0 500
    // @Unit: PWM
    // @User: Standard

    // @Param: THR_ACCEL_D
    // @DisplayName: Throttle acceleration controller D gain
    // @Description: Throttle acceleration controller D gain.  Compensates for short-term change in desired vertical acceleration vs actual acceleration
    // @Range: 0.000 0.400
    // @User: Standard
    GGROUP(pid_throttle_accel,"THR_ACCEL_", AC_PID),

    // PI controller
    //--------------
    // @Param: STB_RLL_P
    // @DisplayName: Roll axis stabilize controller P gain
    // @Description: Roll axis stabilize (i.e. angle) controller P gain.  Converts the error between the desired roll angle and actual angle to a desired roll rate
    // @Range: 3.000 6.000
    // @User: Standard

    // @Param: STB_RLL_I
    // @DisplayName: Roll axis stabilize controller I gain
    // @Description: Roll axis stabilize (i.e. angle) controller I gain.  Corrects for longer-term difference in desired roll angle and actual angle
    // @Range: 0.000 0.100
    // @User: Standard

    // @Param: STB_RLL_IMAX
    // @DisplayName: Roll axis stabilize controller I gain maximum
    // @Description: Roll axis stabilize (i.e. angle) controller I gain maximum.  Constrains the maximum roll rate that the I term will generate
    // @Range: 0 4500
    // @Unit: Centi-Degrees/Sec
    // @User: Standard
    GGROUP(pi_stabilize_roll,       "STB_RLL_", APM_PI),

    // @Param: STB_PIT_P
    // @DisplayName: Pitch axis stabilize controller P gain
    // @Description: Pitch axis stabilize (i.e. angle) controller P gain.  Converts the error between the desired pitch angle and actual angle to a desired pitch rate
    // @Range: 3.000 6.000
    // @User: Standard

    // @Param: STB_PIT_I
    // @DisplayName: Pitch axis stabilize controller I gain
    // @Description: Pitch axis stabilize (i.e. angle) controller I gain.  Corrects for longer-term difference in desired pitch angle and actual angle
    // @Range: 0.000 0.100
    // @User: Standard

    // @Param: STB_PIT_IMAX
    // @DisplayName: Pitch axis stabilize controller I gain maximum
    // @Description: Pitch axis stabilize (i.e. angle) controller I gain maximum.  Constrains the maximum pitch rate that the I term will generate
    // @Range: 0 4500
    // @Unit: Centi-Degrees/Sec
    // @User: Standard
    GGROUP(pi_stabilize_pitch,      "STB_PIT_", APM_PI),

    // @Param: STB_YAW_P
    // @DisplayName: Yaw axis stabilize controller P gain
    // @Description: Yaw axis stabilize (i.e. angle) controller P gain.  Converts the error between the desired yaw angle and actual angle to a desired yaw rate
    // @Range: 3.000 6.000
    // @User: Standard

    // @Param: STB_YAW_I
    // @DisplayName: Yaw axis stabilize controller I gain
    // @Description: Yaw axis stabilize (i.e. angle) controller I gain.  Corrects for longer-term difference in desired yaw angle and actual angle
    // @Range: 0.000 0.100
    // @User: Standard

    // @Param: STB_YAW_IMAX
    // @DisplayName: Yaw axis stabilize controller I gain maximum
    // @Description: Yaw axis stabilize (i.e. angle) controller I gain maximum.  Constrains the maximum yaw rate that the I term will generate
    // @Range: 0 4500
    // @Unit: Centi-Degrees/Sec
    // @User: Standard
    GGROUP(pi_stabilize_yaw,        "STB_YAW_", APM_PI),

    // @Param: THR_ALT_P
    // @DisplayName: Altitude controller P gain
    // @Description: Altitude controller P gain.  Converts the difference between the desired altitude and actual altitude into a climb or descent rate which is passed to the throttle rate controller
    // @Range: 1.000 3.000
    // @User: Standard

    // @Param: THR_ALT_I
    // @DisplayName: Altitude controller I gain
    // @Description: Altitude controller I gain.  Corrects for longer-term difference in desired altitude and actual altitude
    // @Range: 0.000 0.100
    // @User: Standard

    // @Param: THR_ALT_IMAX
    // @DisplayName: Altitude controller I gain maximum
    // @Description: Altitude controller I gain maximum.  Constrains the maximum climb rate rate that the I term will generate
    // @Range: 0 500
    // @Unit: cm/s
    // @User: Standard
    GGROUP(pi_alt_hold,     "THR_ALT_", APM_PI),

    // @Param: HLD_LAT_P
    // @DisplayName: Loiter latitude position controller P gain
    // @Description: Loiter latitude position controller P gain.  Converts the distance (in the latitude direction) to the target location into a desired speed which is then passed to the loiter latitude rate controller
    // @Range: 0.100 0.300
    // @User: Standard

    // @Param: HLD_LAT_I
    // @DisplayName: Loiter latitude position controller I gain
    // @Description: Loiter latitude position controller I gain.  Corrects for longer-term distance (in latitude) to the target location
    // @Range: 0.000 0.100
    // @User: Standard

    // @Param: HLD_LAT_IMAX
    // @DisplayName: Loiter latitude position controller I gain maximum
    // @Description: Loiter latitude position controller I gain maximum.  Constrains the maximum desired speed that the I term will generate
    // @Range: 0 3000
    // @Unit: cm/s
    // @User: Standard
    GGROUP(pi_loiter_lat,   "HLD_LAT_", APM_PI),

    // @Param: HLD_LON_P
    // @DisplayName: Loiter longitude position controller P gain
    // @Description: Loiter longitude position controller P gain.  Converts the distance (in the longitude direction) to the target location into a desired speed which is then passed to the loiter longitude rate controller
    // @Range: 0.100 0.300
    // @User: Standard

    // @Param: HLD_LON_I
    // @DisplayName: Loiter longitude position controller I gain
    // @Description: Loiter longitude position controller I gain.  Corrects for longer-term distance (in longitude direction) to the target location
    // @Range: 0.000 0.100
    // @User: Standard

    // @Param: HLD_LON_IMAX
    // @DisplayName: Loiter longitudeposition controller I gain maximum
    // @Description: Loiter  longitudeposition controller I gain maximum.  Constrains the maximum desired speed that the I term will generate
    // @Range: 0 3000
    // @Unit: cm/s
    // @User: Standard
    GGROUP(pi_loiter_lon,   "HLD_LON_", APM_PI),

    // variables not in the g class which contain EEPROM saved variables

    // variables not in the g class which contain EEPROM saved variables
#if CAMERA == ENABLED
    // @Group: CAM_
    // @Path: ../libraries/AP_Camera/AP_Camera.cpp
    GOBJECT(camera,           "CAM_", AP_Camera),
#endif

    // @Group: COMPASS_
    // @Path: ../libraries/AP_Compass/Compass.cpp
    GOBJECT(compass,        "COMPASS_", Compass),

    // @Group: INS_
    // @Path: ../libraries/AP_InertialSensor/AP_InertialSensor.cpp
    GOBJECT(ins,            "INS_", AP_InertialSensor),

    // @Group: INAV_
    // @Path: ../libraries/AR_InertialNav/AR_InertialNav.cpp
    GOBJECT(inertial_nav,           "INAV_",    AR_InertialNav),

    //@Group: WPNAV_
    //@Path: ../libraries/AR_WPNav/AR_WPNav.cpp
    GOBJECT(wp_nav, "WPNAV_",       AR_WPNav),

    // @Group: SR0_
    // @Path: ./GCS_Mavlink.pde
    GOBJECT(gcs0,                   "SR0_",     GCS_MAVLINK),

    // @Group: SR3_
    // @Path: ./GCS_Mavlink.pde
    GOBJECT(gcs3,                   "SR3_",     GCS_MAVLINK),

    // @Group: AHRS_
    // @Path: ../libraries/AP_AHRS/AP_AHRS.cpp
    GOBJECT(ahrs,                   "AHRS_",    AP_AHRS),

#if MOUNT == ENABLED
    // @Group: MNT_
    // @Path: ../libraries/AP_Mount/AP_Mount.cpp
    GOBJECT(camera_mount,           "MNT_", AP_Mount),
#endif

#if MOUNT2 == ENABLED
    // @Group: MNT2_
    // @Path: ../libraries/AP_Mount/AP_Mount.cpp
    GOBJECT(camera_mount2,           "MNT2_",       AP_Mount),
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
    GOBJECT(sitl, "SIM_", SITL),
#endif

    GOBJECT(barometer, "GND_", AP_Baro),
    GOBJECT(scheduler, "SCHED_", AP_Scheduler),

    // @Group: MOT_
    // @Path: ../libraries/AP_Motors/AP_Motors_Class.cpp
    GOBJECT(motors, "MOT_",         AP_Motors),

    AP_VAREND
};


static void load_parameters(void)
{
    // change the default for the AHRS_GPS_GAIN for ArduCopter
    // if it hasn't been set by the user
    if (!ahrs.gps_gain.load()) {
        ahrs.gps_gain.set_and_save(1.0);
    }

    // setup different AHRS gains for ArduCopter than the default
    // but allow users to override in their config
    if (!ahrs._kp.load()) {
        ahrs._kp.set_and_save(0.1);
    }
    if (!ahrs._kp_yaw.load()) {
        ahrs._kp_yaw.set_and_save(0.1);
    }

#if SECONDARY_DMP_ENABLED == ENABLED
    if (!ahrs2._kp.load()) {
        ahrs2._kp.set(0.1);
    }
    if (!ahrs2._kp_yaw.load()) {
        ahrs2._kp_yaw.set(0.1);
    }
#endif

    // setup different Compass learn setting for ArduCopter than the default
    // but allow users to override in their config
    if (!compass._learn.load()) {
        compass._learn.set_and_save(0);
    }

    if (!g.format_version.load() ||
        g.format_version != Parameters::k_format_version) {

        // erase all parameters
        cliSerial->printf_P(PSTR("Firmware change: erasing EEPROM...\n"));
        AP_Param::erase_all();

        // save the current format version
        g.format_version.set_and_save(Parameters::k_format_version);
        default_dead_zones();
        cliSerial->println_P(PSTR("done."));
    } else {
        uint32_t before = micros();
        // Load all auto-loaded EEPROM variables
        AP_Param::load_all();

        cliSerial->printf_P(PSTR("load_all took %luus\n"), micros() - before);
    }
}
