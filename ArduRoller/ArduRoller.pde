/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#define THISFIRMWARE "ArduRoller V1a"
/*
 *  ArduCopter Version 2.9
 *  Lead author:	Jason Short
 *  Based on code and ideas from the Arducopter team: Randy Mackay, Pat Hickey, Jose Julio, Jani Hirvinen, Andrew Tridgell, Justin Beech, Adam Rivera, Jean-Louis Naudin, Roberto Navoni
 *  Thanks to:	Chris Anderson, Mike Smith, Jordi Munoz, Doug Weibel, James Goppert, Benjamin Pelletier, Robert Lefebvre, Marco Robustini
 *
 *  This firmware is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *
 *  Requires modified "mrelax" version of Arduino, which can be found here:
 *  http://code.google.com/p/ardupilot-mega/downloads/list
 *
 */

////////////////////////////////////////////////////////////////////////////////
// Header includes
////////////////////////////////////////////////////////////////////////////////

#include <math.h>
#include <stdio.h>
#include <stdarg.h>

// Common dependencies
#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_Menu.h>
#include <AP_Param.h>
// AP_HAL
#include <AP_HAL.h>
#include <AP_HAL_AVR.h>
#include <AP_HAL_AVR_SITL.h>
#include <AP_HAL_SMACCM.h>
#include <AP_HAL_PX4.h>
#include <AP_HAL_Empty.h>

// Application dependencies
#include <GCS_MAVLink.h>        // MAVLink GCS definitions
#include <AP_GPS.h>             // ArduPilot GPS library
#include <DataFlash.h>          // ArduPilot Mega Flash Memory Library
#include <AP_ADC.h>             // ArduPilot Mega Analog to Digital Converter Library
#include <AP_ADC_AnalogSource.h>
//#include <AP_Baro.h>
#include <AP_Compass.h>         // ArduPilot Mega Magnetometer Library
#include <AP_Math.h>            // ArduPilot Mega Vector/Matrix math Library
#include <AP_Curve.h>           // Curve used to linearlise throttle pwm to thrust
#include <AP_InertialSensor.h>  // ArduPilot Mega Inertial Sensor (accel & gyro) Library
#include <AP_AHRS.h>
#include <APM_PI.h>             // PI library
#include <AC_PID.h>             // PID library
#include <RC_Channel.h>         // RC Channel Library
#include <AP_RangeFinder.h>     // Range finder library
#include <AP_OpticalFlow.h>     // Optical Flow library
#include <Filter.h>             // Filter library
#include <AP_Buffer.h>          // APM FIFO Buffer
#include <AP_Relay.h>           // APM relay
#include <AP_Camera.h>          // Photo or video camera
#include <AP_Mount.h>           // Camera/Antenna mount
#include <AP_Airspeed.h>        // needed for AHRS build
#include <AR_EncoderNav.h>     // ArduPilot Mega inertial navigation library
#include <AP_Declination.h>     // ArduPilot Mega Declination Helper Library
#include <memcheck.h>           // memory limit checker
#include <SITL.h>               // software in the loop support
#include <AP_Scheduler.h>       // main loop scheduler

// AP_HAL to Arduino compatibility layer
#include "compat.h"
// Configuration
#include "defines.h"
#include "config.h"
#include "config_channels.h"

// Local modules
#include "Parameters.h"
#include "GCS.h"

////////////////////////////////////////////////////////////////////////////////
// cliSerial
////////////////////////////////////////////////////////////////////////////////
// cliSerial isn't strictly necessary - it is an alias for hal.console. It may
// be deprecated in favor of hal.console in later releases.
static AP_HAL::BetterStream* cliSerial;

// N.B. we need to keep a static declaration which isn't guarded by macros
// at the top to cooperate with the prototype mangler.

////////////////////////////////////////////////////////////////////////////////
// AP_HAL instance
////////////////////////////////////////////////////////////////////////////////

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;


////////////////////////////////////////////////////////////////////////////////
// Parameters
////////////////////////////////////////////////////////////////////////////////
//
// Global parameters are all contained within the 'g' class.
//
static Parameters g;

// main loop scheduler
static AP_Scheduler scheduler;

////////////////////////////////////////////////////////////////////////////////
// prototypes
////////////////////////////////////////////////////////////////////////////////
static void update_events(void);
static void print_flight_mode(AP_HAL::BetterStream *port, uint8_t mode);

////////////////////////////////////////////////////////////////////////////////
// Dataflash
////////////////////////////////////////////////////////////////////////////////
#if CONFIG_HAL_BOARD == HAL_BOARD_APM2
	static DataFlash_APM2 DataFlash;
#elif CONFIG_HAL_BOARD == HAL_BOARD_APM1
	static DataFlash_APM1 DataFlash;
#elif CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
	//static DataFlash_File DataFlash("/tmp/APMlogs");
	static DataFlash_SITL DataFlash;
#elif CONFIG_HAL_BOARD == HAL_BOARD_PX4
	static DataFlash_File DataFlash("/fs/microsd/APM/logs");
#else
	static DataFlash_Empty DataFlash;
#endif


////////////////////////////////////////////////////////////////////////////////
// the rate we run the main loop at
////////////////////////////////////////////////////////////////////////////////
static const AP_InertialSensor::Sample_rate ins_sample_rate = AP_InertialSensor::RATE_200HZ;

////////////////////////////////////////////////////////////////////////////////
// Sensors
////////////////////////////////////////////////////////////////////////////////
//

// All GPS access should be through this pointer.
static GPS         *g_gps;

// flight modes convenience array
static AP_Int8 *flight_modes = &g.flight_mode1;

#if HIL_MODE == HIL_MODE_DISABLED

#if CONFIG_ADC == ENABLED
	static AP_ADC_ADS7844 adc;
#endif

#if CONFIG_IMU_TYPE == CONFIG_IMU_MPU6000
	static AP_InertialSensor_MPU6000 ins;
#elif CONFIG_IMU_TYPE == CONFIG_IMU_OILPAN
	static AP_InertialSensor_Oilpan ins(&adc);
#elif CONFIG_IMU_TYPE == CONFIG_IMU_SITL
	static AP_InertialSensor_Stub ins;
#elif CONFIG_IMU_TYPE == CONFIG_IMU_PX4
	static AP_InertialSensor_PX4 ins;
#endif

 #if CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
 // When building for SITL we use the HIL barometer and compass drivers
//static AP_Baro_BMP085_HIL barometer;
static AP_Compass_HIL compass;
static SITL sitl;
 #else
// Otherwise, instantiate a real barometer and compass driver
#if CONFIG_BARO == AP_BARO_BMP085
    //static AP_Baro_BMP085 barometer;
#elif CONFIG_BARO == AP_BARO_PX4
    //static AP_Baro_PX4 barometer;
#elif CONFIG_BARO == AP_BARO_MS5611
    #if CONFIG_MS5611_SERIAL == AP_BARO_MS5611_SPI
        //static AP_Baro_MS5611 barometer(&AP_Baro_MS5611::spi);
    #elif CONFIG_MS5611_SERIAL == AP_BARO_MS5611_I2C
     //static AP_Baro_MS5611 barometer(&AP_Baro_MS5611::i2c);
    #else
    #error Unrecognized CONFIG_MS5611_SERIAL setting.
#endif
  #endif

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
    static AP_Compass_PX4 compass;
#else
    static AP_Compass_HMC5843 compass;
#endif
#endif

// real GPS selection
 #if   GPS_PROTOCOL == GPS_PROTOCOL_AUTO
AP_GPS_Auto     g_gps_driver(&g_gps);

 #elif GPS_PROTOCOL == GPS_PROTOCOL_NMEA
AP_GPS_NMEA     g_gps_driver();

 #elif GPS_PROTOCOL == GPS_PROTOCOL_SIRF
AP_GPS_SIRF     g_gps_driver();

 #elif GPS_PROTOCOL == GPS_PROTOCOL_UBLOX
AP_GPS_UBLOX    g_gps_driver();

 #elif GPS_PROTOCOL == GPS_PROTOCOL_MTK
AP_GPS_MTK      g_gps_driver();

 #elif GPS_PROTOCOL == GPS_PROTOCOL_MTK19
AP_GPS_MTK19    g_gps_driver();

 #elif GPS_PROTOCOL == GPS_PROTOCOL_NONE
AP_GPS_None     g_gps_driver();

 #else
  #error Unrecognised GPS_PROTOCOL setting.
 #endif // GPS PROTOCOL

#if DMP_ENABLED == ENABLED && CONFIG_HAL_BOARD == HAL_BOARD_APM2
    static AP_AHRS_MPU6000  ahrs(&ins, g_gps);               // only works with APM2
#else
    static AP_AHRS_DCM ahrs(&ins, g_gps);
#endif

// ahrs2 object is the secondary ahrs to allow running DMP in parallel with DCM
#if SECONDARY_DMP_ENABLED == ENABLED && CONFIG_HAL_BOARD == HAL_BOARD_APM2
    static AP_AHRS_MPU6000  ahrs2(&ins, g_gps);               // only works with APM2
#endif

#elif HIL_MODE == HIL_MODE_SENSORS
// sensor emulators
static AP_ADC_HIL              adc;
//static AP_Baro_BMP085_HIL      barometer;
static AP_Compass_HIL          compass;
static AP_GPS_HIL              g_gps_driver;
static AP_InertialSensor_Stub  ins;
static AP_AHRS_DCM             ahrs(&ins, g_gps);

static int32_t gps_base_alt;

 #if CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
 // When building for SITL we use the HIL barometer and compass drivers
static SITL sitl;
#endif

#elif HIL_MODE == HIL_MODE_ATTITUDE
static AP_ADC_HIL              adc;
static AP_InertialSensor_Stub  ins;
static AP_AHRS_HIL             ahrs(&ins, g_gps);
static AP_GPS_HIL              g_gps_driver;
static AP_Compass_HIL          compass;                  // never used
//static AP_Baro_BMP085_HIL      barometer;

static int32_t gps_base_alt;

#if CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
 // When building for SITL we use the HIL barometer and compass drivers
static SITL sitl;
#endif

#else
 #error Unrecognised HIL_MODE setting.
#endif // HIL MODE

////////////////////////////////////////////////////////////////////////////////
// Optical flow sensor
////////////////////////////////////////////////////////////////////////////////
 #if OPTFLOW == ENABLED
static AP_OpticalFlow_ADNS3080 optflow;
 #else
static AP_OpticalFlow optflow;
 #endif

////////////////////////////////////////////////////////////////////////////////
// GCS selection
////////////////////////////////////////////////////////////////////////////////
static GCS_MAVLINK gcs0;
static GCS_MAVLINK gcs3;

////////////////////////////////////////////////////////////////////////////////
// SONAR selection
////////////////////////////////////////////////////////////////////////////////
//
ModeFilterInt16_Size3 sonar_mode_filter(1);
#if CONFIG_SONAR == ENABLED
static AP_HAL::AnalogSource *sonar_analog_source;
static AP_RangeFinder_MaxsonarXL *sonar;
#endif

////////////////////////////////////////////////////////////////////////////////
// Global variables
////////////////////////////////////////////////////////////////////////////////

/* Radio values
 *               Channel assignments
 *                       1	Ailerons (rudder if no ailerons)
 *                       2	Elevator
 *                       3	Throttle
 *                       4	Rudder (if we have ailerons)
 *                       5	Mode - 3 position switch
 *                       6  User assignable
 *                       7	trainer switch - sets throttle nominal (toggle switch), sets accels to Level (hold > 1 second)
 *                       8	TBD
 *               Each Aux channel can be configured to have any of the available auxiliary functions assigned to it.
 *               See libraries/RC_Channel/RC_Channel_aux.h for more information
 */

//Documentation of GLobals:
static union {
    struct {
        uint8_t home_is_set        : 1; // 0
        uint8_t low_battery        : 1; // 4    // Used to track if the battery is low - LED output flashes when the batt is low
        uint8_t pre_arm_check      : 1; // 5    // true if the radio and accel calibration have been performed
        uint8_t armed              : 1; // 6
        uint8_t failsafe_batt      : 1; // 9    // A status flag for the battery failsafe
        uint8_t failsafe_gps       : 1; // 10   // A status flag for the gps failsafe
        uint8_t failsafe_gcs       : 1; // 11   // A status flag for the ground station failsafe
        uint8_t rc_override_active : 1; // 12   // true if rc control are overwritten by ground station
        uint8_t compass_status     : 1; // 16
        uint8_t gps_status         : 1; // 17
        uint8_t position_hold      : 1; // 17
        uint8_t obstacle           : 1; // 17
    };
    uint16_t value;
} ap;


static struct AP_System{
    uint8_t GPS_light               : 1; // 1   // Solid indicates we have full 3D lock and can navigate, flash = read
    uint8_t motor_light             : 1; // 2   // Solid indicates Armed state
    uint8_t new_radio_frame         : 1; // 3   // Set true if we have new PWM data to act on from the Radio
    uint8_t CH7_flag                : 1; // 4   // manages state of the ch7 toggle switch
    uint8_t usb_connected           : 1; // 5   // true if APM is powered from USB connection
    uint8_t yaw_stopped             : 1; // 6   // Used to manage the Yaw hold capabilities

} ap_system;



////////////////////////////////////////////////////////////////////////////////
// Radio
////////////////////////////////////////////////////////////////////////////////
// This is the state of the flight control system
// There are multiple states defined such as STABILIZE, ACRO,
static int8_t control_mode = STABILIZE;
// Used to maintain the state of the previous control switch position
// This is set to -1 when we need to re-read the switch
static uint8_t oldSwitchPosition;

// receiver RSSI
static uint8_t receiver_rssi;



////////////////////////////////////////////////////////////////////////////////
// PIDs
////////////////////////////////////////////////////////////////////////////////
// This is a convienience accessor for the IMU roll rates. It's currently the raw IMU rates
// and not the adjusted omega rates, but the name is stuck
static Vector3f omega;
// This is used to hold radio tuning values for in-flight CH6 tuning
float tuning_value;

////////////////////////////////////////////////////////////////////////////////
// LED output
////////////////////////////////////////////////////////////////////////////////
// This is current status for the LED lights state machine
// setting this value changes the output of the LEDs
static uint8_t led_mode = NORMAL_LEDS;
// Blinking indicates GPS status
static uint8_t copter_leds_GPS_blink;
// Blinking indicates battery status
static uint8_t copter_leds_motor_blink;
// Navigation confirmation blinks
static int8_t copter_leds_nav_blink;

////////////////////////////////////////////////////////////////////////////////
// GPS variables
////////////////////////////////////////////////////////////////////////////////
// This is used to scale GPS values for EEPROM storage
// 10^7 times Decimal GPS means 1 == 1cm
// This approximation makes calculations integer and it's easy to read
static const float t7 = 10000000.0;
// We use atan2 and other trig techniques to calaculate angles
// We need to scale the longitude up to make these calcs work
// to account for decreasing distance between lines of longitude away from the equator
static float scaleLongUp = 1;
// Sometimes we need to remove the scaling for distance calcs
static float scaleLongDown = 1;


////////////////////////////////////////////////////////////////////////////////
// Mavlink specific
////////////////////////////////////////////////////////////////////////////////
// Used by Mavlink for unknow reasons
static const float radius_of_earth = 6378100;   // meters

////////////////////////////////////////////////////////////////////////////////
// Location & Navigation
////////////////////////////////////////////////////////////////////////////////
// This is the angle from the copter to the next waypoint in centi-degrees
static int32_t wp_bearing;
// The original bearing to the next waypoint.  used to check if we've passed the waypoint
static int32_t original_wp_bearing;
// The location of home in relation to the copter in centi-degrees
static int32_t home_bearing;
// distance between plane and home in cm
static int32_t home_distance;
// distance between plane and next waypoint in cm.  is not static because AP_Camera uses it
uint32_t wp_distance;
float loiter_distance;
// navigation mode - options include NAV_NONE, NAV_LOITER, NAV_CIRCLE, NAV_WP
static uint8_t nav_mode;
// Register containing the index of the current navigation command in the mission script
static int16_t command_nav_index;
// Register containing the index of the previous navigation command in the mission script
// Used to manage the execution of conditional commands
static uint8_t prev_nav_index;
// Register containing the index of the current conditional command in the mission script
static uint8_t command_cond_index;
// Used to track the required WP navigation information
// options include
// NAV_ALTITUDE - have we reached the desired altitude?
// NAV_LOCATION - have we reached the desired location?
// NAV_DELAY    - have we waited at the waypoint the desired time?


static int16_t _crosstrack_fix;
static int16_t _avoid_obstacle;


////////////////////////////////////////////////////////////////////////////////
// Orientation
////////////////////////////////////////////////////////////////////////////////
// Convienience accessors for commonly used trig functions. These values are generated
// by the DCM through a few simple equations. They are used throughout the code where cos and sin
// would normally be used.
// The cos values are defaulted to 1 to get a decent initial value for a level state
static float cos_yaw            = 1;
static float sin_yaw            = 1;


////////////////////////////////////////////////////////////////////////////////
// Circle Mode / Loiter control
////////////////////////////////////////////////////////////////////////////////
Vector3f circle_center;     // circle position expressed in cm from home location.  x = lat, y = lon
// angle from the circle center to the copter's desired location.
static float circle_angle;
// the total angle (in radians) travelled
static float circle_angle_total;
// deg : how many times to circle as specified by mission command
static uint8_t circle_desired_rotations;
// How long we should stay in Loiter Mode for mission scripting (time in seconds)
static uint16_t loiter_time_max;
// How long have we been loitering - The start time in millis
static uint32_t loiter_time;


////////////////////////////////////////////////////////////////////////////////
// CH7 control
////////////////////////////////////////////////////////////////////////////////
// This register tracks the current Mission Command index when writing
// a mission using CH7 in flight
static int8_t CH7_wp_index;


////////////////////////////////////////////////////////////////////////////////
// Battery Sensors
////////////////////////////////////////////////////////////////////////////////
// Battery Voltage of battery, initialized above threshold for filter
static float battery_voltage1 = LOW_VOLTAGE * 1.05f;
// refers to the instant amp draw – based on an Attopilot Current sensor
static float current_amps1;
// refers to the total amps drawn – based on an Attopilot Current sensor
static float current_total1;


////////////////////////////////////////////////////////////////////////////////
// Sonar
////////////////////////////////////////////////////////////////////////////////
// The altitude as reported by Sonar in cm – Values are 20 to 700 generally.
static int16_t sonar_distance;
static uint8_t sonar_health;   // true if we can trust the altitude from the sonar


////////////////////////////////////////////////////////////////////////////////
// flight modes
////////////////////////////////////////////////////////////////////////////////
// Flight modes are combinations of Roll/Pitch, Yaw and Throttle control modes
// Each Flight mode is a unique combination of these modes
//
// The current desired control scheme for Yaw
static uint8_t yaw_mode;
// The current desired control scheme for roll and pitch / navigation
static uint8_t roll_pitch_mode;


////////////////////////////////////////////////////////////////////////////////
// 3D Location vectors
////////////////////////////////////////////////////////////////////////////////
// home location is stored when we have a good GPS lock and arm the copter
// Can be reset each the copter is re-armed
static struct   Location home;
// Current location of the copter
static struct   Location current_loc;
// Holds the current loaded command from the EEPROM for navigation
static struct   Location command_nav_queue;
// Holds the current loaded command from the EEPROM for conditional scripts
static struct   Location command_cond_queue;


////////////////////////////////////////////////////////////////////////////////
// Navigation Roll/Pitch functions
////////////////////////////////////////////////////////////////////////////////
// all angles are deg * 100 : target yaw angle
// The Commanded ROll from the autopilot.
static int32_t nav_roll;
// The Commanded pitch from the autopilot. negative Pitch means go forward.
static int32_t nav_pitch;


////////////////////////////////////////////////////////////////////////////////
// Navigation Throttle control
////////////////////////////////////////////////////////////////////////////////
// This could be useful later in determining and debuging current usage and predicting battery life
//static uint32_t throttle_integrator;


////////////////////////////////////////////////////////////////////////////////
// Navigation Yaw control
////////////////////////////////////////////////////////////////////////////////
// The Commanded Yaw from the autopilot.
static int32_t nav_yaw;
//static uint8_t yaw_timer;
// Yaw will point at this location if yaw_mode is set to YAW_LOOK_AT_LOCATION
static Vector3f yaw_look_at_WP;
// bearing from current location to the yaw_look_at_WP
static int32_t yaw_look_at_WP_bearing;
// yaw used for YAW_LOOK_AT_HEADING yaw_mode
static int32_t yaw_look_at_heading;
// Deg/s we should turn
static int16_t yaw_look_at_heading_slew;



////////////////////////////////////////////////////////////////////////////////
// Repeat Mission Scripting Command
////////////////////////////////////////////////////////////////////////////////
// The type of repeating event - Toggle a servo channel, Toggle the APM1 relay, etc
static uint8_t event_id;
// Used to manage the timimng of repeating events
static uint32_t event_timer;
// How long to delay the next firing of event in millis
static uint16_t event_delay;
// how many times to fire : 0 = forever, 1 = do once, 2 = do twice
static int16_t event_repeat;
// per command value, such as PWM for servos
static int16_t event_value;
// the stored value used to undo commands - such as original PWM command
static int16_t event_undo_value;

////////////////////////////////////////////////////////////////////////////////
// Delay Mission Scripting Command
////////////////////////////////////////////////////////////////////////////////
static int32_t condition_value;  // used in condition commands (eg delay, change alt, etc.)
static uint32_t condition_start;


////////////////////////////////////////////////////////////////////////////////
// IMU variables
////////////////////////////////////////////////////////////////////////////////
// Integration time for the gyros (DCM algorithm)
// Updated with the fast loop
static float G_Dt = 0.01;

////////////////////////////////////////////////////////////////////////////////
// Inertial Navigation
////////////////////////////////////////////////////////////////////////////////
static AR_EncoderNav encoder_nav(&g_gps);

////////////////////////////////////////////////////////////////////////////////
// Performance monitoring
////////////////////////////////////////////////////////////////////////////////
// The number of GPS fixes we have had
static int16_t pmTest1;


////////////////////////////////////////////////////////////////////////////////
// Balance specific
////////////////////////////////////////////////////////////////////////////////
int16_t I2Cfail;

static int16_t motor_out[2];    // This is the array of PWM values being sent to the motors
static float balance_offset;

static float ground_speed;

static int16_t pitch_out_right;
static int16_t pitch_out_left;
static int16_t yaw_out;

static float desired_speed;
static float desired_ticks;

static float wheel_ratio;
static float current_speed;
static float current_encoder_y;
static float current_encoder_x;
static uint32_t balance_timer;

static uint16_t obstacle_counter = 0;

// used for simple distance calcs
static float distance;

//static bool gps_available;

AP_HAL::Semaphore*  _i2c_sem;

////////////////////////////////////////////////////////////////////////////////
// WP Nav
////////////////////////////////////////////////////////////////////////////////
Vector3f    _origin;                // starting point of trip to next waypoint in cm from home (equivalent to next_WP)
Vector3f    _destination;           // target destination in cm from home (equivalent to next_WP)
float       _distance_to_target;    // distance to loiter target
bool        _reached_destination;   // true if we have reached the destination

////////////////////////////////////////////////////////////////////////////////
// Wheels
////////////////////////////////////////////////////////////////////////////////

// I2C address of wheel encoders
#define ENCODER_ADDRESS       0x29

static struct {
	int16_t left_distance;
	int16_t right_distance;
	int16_t left_speed;
	int16_t right_speed;
    int16_t left_speed_output;
    int16_t right_speed_output;
	float speed;
} wheel;

// I2C Receive buffer
static union {
    int32_t long_value;
    int16_t int_value;
    uint8_t bytes[];
} bytes_union;



// System Timers
// --------------
// Time in microseconds of main control loop
static uint32_t fast_loopTimer;
// Counters for branching from 10 hz control loop
static uint8_t medium_loopCounter;
// Counters for branching from 3 1/3hz control loop
static uint8_t slow_loopCounter;
// Counter of main loop executions.  Used for performance monitoring and failsafe processing
static uint16_t mainLoop_count;
// Delta Time in milliseconds for navigation computations, updated with every good GPS read
//static float dTnav;
// Counters for branching from 4 minute control loop used to save Compass offsets
static uint16_t superslow_loopCounter;

// I hate when the motors spin up during startup
static int8_t startup_counter;


// prevents duplicate GPS messages from entering system
static uint32_t last_gps_time;
// the time when the last HEARTBEAT message arrived from a GCS - used for triggering gcs failsafe
static uint32_t last_heartbeat_ms;

// Used to exit the roll and pitch auto trim function
static uint8_t auto_trim_counter;

// Reference to the relay object (APM1 -> PORTL 2) (APM2 -> PORTB 7)
static AP_Relay relay;

//Reference to the camera object (it uses the relay object inside it)
#if CAMERA == ENABLED
  static AP_Camera camera(&relay);
#endif

// a pin for reading the receiver RSSI voltage. The scaling by 0.25
// is to take the 0 to 1024 range down to an 8 bit range for MAVLink
static AP_HAL::AnalogSource* rssi_analog_source;


// Input sources for battery voltage, battery current, board vcc
static AP_HAL::AnalogSource* batt_volt_analog_source;
static AP_HAL::AnalogSource* batt_curr_analog_source;
static AP_HAL::AnalogSource* board_vcc_analog_source;


#if CLI_ENABLED == ENABLED
    static int8_t   setup_show (uint8_t argc, const Menu::arg *argv);
#endif

// Camera/Antenna mount tracking and stabilisation stuff
// --------------------------------------
#if MOUNT == ENABLED
// current_loc uses the baro/gps soloution for altitude rather than gps only.
// mabe one could use current_loc for lat/lon too and eliminate g_gps alltogether?
static AP_Mount camera_mount(&current_loc, g_gps, &ahrs, 0);
#endif

#if MOUNT2 == ENABLED
// current_loc uses the baro/gps soloution for altitude rather than gps only.
// mabe one could use current_loc for lat/lon too and eliminate g_gps alltogether?
static AP_Mount camera_mount2(&current_loc, g_gps, &ahrs, 1);
#endif

////////////////////////////////////////////////////////////////////////////////
// Top-level logic
////////////////////////////////////////////////////////////////////////////////

// setup the var_info table
AP_Param param_loader(var_info, WP_START_BYTE);

/*
  scheduler table - all regular tasks apart from the fast_loop()
  should be listed here, along with how often they should be called
  (in 10ms units) and the maximum time they are expected to take (in
  microseconds)
 */
static const AP_Scheduler::Task scheduler_tasks[] PROGMEM = {
    { update_GPS,            2,     900 }, // 0
    { medium_loop,           2,     700 }, // 2
    { fifty_hz_loop,         2,     950 }, // 3
    { run_nav_updates,      10,     800 }, // 4
    { slow_loop,            10,     500 }, // 5
    { gcs_check_input,	     2,     700 }, // 6
    { gcs_send_heartbeat,  100,     700 }, // 7
    { gcs_data_stream_send,  2,    1500 }, // 8
    { gcs_send_deferred,     2,    1200 }, // 9
    { compass_accumulate,    2,     950 }, // 10
    { super_slow_loop,     100,    1100 }, // 11
    { perf_update,        1000,     500 }  // 12
};


void setup() {
    // this needs to be the first call, as it fills memory with
    // sentinel values
    memcheck_init();
    cliSerial = hal.console;

    // Load the default values of variables listed in var_info[]s
    AP_Param::setup_sketch_defaults();

#if CONFIG_SONAR == ENABLED
 #if CONFIG_SONAR_SOURCE == SONAR_SOURCE_ADC
    sonar_analog_source = new AP_ADC_AnalogSource(
            &adc, CONFIG_SONAR_SOURCE_ADC_CHANNEL, 0.25);
 #elif CONFIG_SONAR_SOURCE == SONAR_SOURCE_ANALOG_PIN
    sonar_analog_source = hal.analogin->channel(
            CONFIG_SONAR_SOURCE_ANALOG_PIN);
 #else
  #warning "Invalid CONFIG_SONAR_SOURCE"
 #endif
    sonar = new AP_RangeFinder_MaxsonarXL(sonar_analog_source,
            &sonar_mode_filter);
#endif

    rssi_analog_source      = hal.analogin->channel(g.rssi_pin, 0.25);
    batt_volt_analog_source = hal.analogin->channel(g.battery_volt_pin);
    batt_curr_analog_source = hal.analogin->channel(g.battery_curr_pin);
    board_vcc_analog_source = hal.analogin->channel(ANALOG_INPUT_BOARD_VCC);

    init_ardupilot();

    // initialise the main loop scheduler
    scheduler.init(&scheduler_tasks[0], sizeof(scheduler_tasks)/sizeof(scheduler_tasks[0]));
}

/*
  if the compass is enabled then try to accumulate a reading
 */
static void compass_accumulate(void)
{
    if (g.compass_enabled) {
        compass.accumulate();
    }
}


// enable this to get console logging of scheduler performance
#define SCHEDULER_DEBUG 1

static void perf_update(void)
{
    if (g.log_bitmask & MASK_LOG_PM)
        Log_Write_Performance();

    if (scheduler.debug()) {
        cliSerial->printf_P(PSTR("PERF: %u/%u %lu\n"),
                            (unsigned)perf_info_get_num_long_running(),
                            (unsigned)perf_info_get_num_loops(),
                            (unsigned long)perf_info_get_max_time());
    }
    perf_info_reset();
    pmTest1 = 0;
}

void loop()
{
    uint32_t timer = micros();

    // We want this to execute fast
    // ----------------------------
    if (ins.num_samples_available() >= 2) {

        // check loop time
        perf_info_check_loop_time(timer - fast_loopTimer);

        G_Dt            = (float)(timer - fast_loopTimer) / 1000000.f;                  // used by PI Loops
        fast_loopTimer  = timer;

        // for mainloop failure monitoring
        mainLoop_count++;

        // Execute the fast loop
        // ---------------------
        fast_loop();

        // tell the scheduler one tick has passed
        scheduler.tick();
        
    } else {
        uint16_t dt = timer - fast_loopTimer;
        if (dt < 10000) {
            uint16_t time_to_next_loop = 10000 - dt;
            scheduler.run(time_to_next_loop);
        }
    }
}


// Main loop - 100hz
static void fast_loop()
{
    // IMU DCM Algorithm
    // --------------------
    read_AHRS();

    // reads all of the necessary trig functions for cameras, throttle, etc.
    // --------------------------------------------------------------------
    update_trig();

    // Read radio and 3-position switch on radio
    // -----------------------------------------
    read_radio();
    read_control_switch();

    // custom code/exceptions for flight modes
    // ---------------------------------------
    if(check_attitude()){
        update_yaw_mode();
        update_roll_pitch_mode();
    }

    /*
    cliSerial->printf_P(PSTR("a:%d\tP: %d\tY: %d\n"),
            (int16_t)ahrs.pitch_sensor,
            pitch_out,
            yaw_out);
    //*/

    // write out the servo PWM values
    // ------------------------------
    update_servos();
}

static void medium_loop()
{
    // This is the start of the medium (10 Hz) loop pieces
    // -----------------------------------------
    switch(medium_loopCounter) {

    // This case deals with the GPS and Compass
    //-----------------------------------------
    case 0:
        medium_loopCounter++;

        // read battery before compass because it may be used for motor interference compensation
        if (g.battery_monitoring != 0) {
            read_battery();
        }

        if(g.compass_enabled) {
            if (compass.read()) {
                compass.null_offsets();
            }
        }

        // auto_trim - stores roll and pitch radio inputs to ahrs
        auto_trim();
        break;

    // This case performs some navigation computations
    //------------------------------------------------
    case 1:
        medium_loopCounter++;
        read_receiver_rssi();
        break;

    // command processing
    //-------------------
    case 2:
        medium_loopCounter++;

        // log compass information
        if (ap.armed && (g.log_bitmask & MASK_LOG_COMPASS)) {
            Log_Write_Compass();
        }
        break;

    // This case deals with sending high rate telemetry
    //-------------------------------------------------
    case 3:
        medium_loopCounter++;

        if(ap.armed) {
            if (g.log_bitmask & MASK_LOG_ATTITUDE_MED) {
                //Log_Write_Attitude();
                #if SECONDARY_DMP_ENABLED == ENABLED
                Log_Write_DMP();
                #endif
            }
        }
        break;

    // This case controls the slow loop
    //---------------------------------
    case 4:
        medium_loopCounter = 0;

        // Accel trims      = hold > 2 seconds
        // Throttle cruise  = switch less than 1 second
        // --------------------------------------------
        read_trim_switch();


#if COPTER_LEDS == ENABLED
        update_copter_leds();
#endif
        break;

    default:
        // this is just a catch all
        // ------------------------
        medium_loopCounter = 0;
        break;
    }
}

// stuff that happens at 50 hz
// ---------------------------
static void fifty_hz_loop()
{
    // read wheel encoders:
    // -------------------
    if(update_wheel_encoders()){
        //cliSerial->println("OK");
    }else{
        //cliSerial->printf("E %d\n", I2Cfail);
    }

    if(ap.armed){
        // encoder nav
        // --------------------
        encoder_nav_update();
    }else{
        // XXX temp
        encoder_nav_update();
    }

    #if MOUNT == ENABLED
    // update camera mount's position
    camera_mount.update_mount_position();
    #endif

    #if MOUNT2 == ENABLED
    // update camera mount's position
    camera_mount2.update_mount_position();
    #endif

    #if CAMERA == ENABLED
    camera.trigger_pic_cleanup();
    #endif

    if (g.log_bitmask & MASK_LOG_ATTITUDE_FAST && ap.armed) {
        //Log_Write_Attitude();
        #if SECONDARY_DMP_ENABLED == ENABLED
        Log_Write_DMP();
        #endif
    }

    if (g.log_bitmask & MASK_LOG_IMU && ap.armed)
        DataFlash.Log_Write_IMU(&ins);

}

// slow_loop - 3.3hz loop
static void slow_loop()
{
    // This is the slow (3 1/3 Hz) loop pieces
    //----------------------------------------
    switch (slow_loopCounter) {
    case 0:
        slow_loopCounter++;
        superslow_loopCounter++;

        if(startup_counter == -1 || startup_counter > 33) {
            startup_counter = -1;
        }else{
            startup_counter++;
        }

        // record if the compass is healthy
        set_compass_healthy(compass.healthy);

        if(superslow_loopCounter > 4000) {  // 20 min
            if(control_mode == FBW && g.compass_enabled) {
                compass.save_offsets();
                superslow_loopCounter = 0;
            }
        }

        break;

    case 1:
        slow_loopCounter++;

        #if CONFIG_HAL_BOARD == HAL_BOARD_PX4
        update_aux_servo_function(&g.rc_5, &g.rc_6, &g.rc_7, &g.rc_8, &g.rc_9, &g.rc_10, &g.rc_11, &g.rc_12);
        #elif MOUNT == ENABLED
        update_aux_servo_function(&g.rc_5, &g.rc_6, &g.rc_7, &g.rc_8, &g.rc_10, &g.rc_11);
        #endif
        enable_aux_servos();

        #if MOUNT == ENABLED
        camera_mount.update_mount_type();
        #endif

        #if MOUNT2 == ENABLED
        camera_mount2.update_mount_type();
        #endif

        break;

    case 2:
        slow_loopCounter = 0;
        update_events();

        // blink if we are armed
        update_lights();

        if(g.radio_tuning > 0)
            tuning();

        #if USB_MUX_PIN > 0
        check_usb_mux();
        #endif
        break;

    default:
        slow_loopCounter = 0;
        break;
    }
}

// 1Hz loop
static void super_slow_loop()
{
    if (g.log_bitmask != 0) {
        Log_Write_Data(DATA_AP_STATE, ap.value);
    }
}

// called at 50hz
static void update_GPS(void)
{
    // A counter that is used to grab at least 10 reads before commiting the Home location
    //static uint8_t ground_start_count  = 10;

    g_gps->update();
    update_GPS_light();

    set_gps_healthy(g_gps->status() >= GPS::GPS_OK_FIX_3D);

    if (g_gps->new_data && last_gps_time != g_gps->time && g_gps->status() >= GPS::GPS_OK_FIX_2D) {
        // clear new data flag
        g_gps->new_data = false;

        // save GPS time so we don't get duplicate reads
        last_gps_time = g_gps->time;

        // log location if we have at least a 2D fix
        if (g.log_bitmask & MASK_LOG_GPS && ap.armed) {
            DataFlash.Log_Write_GPS(g_gps, current_loc.alt);
        }
    }
}

// update_yaw_mode - run high level yaw controllers
// 100hz update rate
void update_yaw_mode(void)
{
    static int8_t yaw_counter = 0;
    static int32_t target_yaw = 0;

    switch(yaw_mode){
        case YAW_ACRO:
            yaw_out = g.rc_1.control_in / 2;
            target_yaw = ahrs.yaw_sensor;
            break;

        case YAW_HOLD:
            target_yaw = ahrs.yaw_sensor;
            if(g.rc_1.control_in != 0){
                // manual control
                yaw_out   = g.rc_1.control_in;
                yaw_counter    = 100; // one second
            }else{
                // Hold Yaw
                if(yaw_counter > 0){
                    // timer is used to decelerate yaw.
                    yaw_counter--;
                    
                    // Reset Yaw to current angle
                    if(yaw_counter == 0){
                        nav_yaw     = ahrs.yaw_sensor;
                    }
                    yaw_out   = 0;
                    
                }else{
                    //yaw_out   = get_stabilize_yaw(avoid_obstacle(nav_yaw));
                    yaw_out   = get_stabilize_yaw(nav_yaw);
                }
            }
            break;

        case YAW_LOOK_AT_NEXT_WP:            
            if(nav_mode == NAV_AVOID_TURN){
                // set hold to 90° to
                nav_yaw = wrap_360_cd(wp_bearing + 9000);
                target_yaw = get_yaw_slew(target_yaw, nav_yaw, AUTO_YAW_SLEW_RATE+80);
                yaw_out = get_stabilize_yaw(target_yaw);
            }else{
                if(g.sonar_enabled){
                    nav_yaw = avoid_obstacle(wp_bearing);
                }else{
                    nav_yaw = wp_bearing;
        
                    // calc crosstrack
                    nav_yaw = get_crosstrack(nav_yaw);
                }

                target_yaw = get_yaw_slew(target_yaw, nav_yaw, AUTO_YAW_SLEW_RATE);
                yaw_out = get_stabilize_yaw(target_yaw);
                break;
            }
    }
}

// update_roll_pitch_mode - run high level roll and pitch controllers
// 100hz update rate
void update_roll_pitch_mode(void)
{
    switch(roll_pitch_mode){
        case ROLL_PITCH_STABLE:
            // we always hold position
            if(g.rc_2.control_in == 0){
                if(!ap.position_hold){
                    ap.position_hold = true;
                    set_destination(encoder_nav.get_position());
                    _reached_destination = true;
                }
                // we are in position hold
                //desired_speed = limit_acceleration(loiter_distance, 60.0); // cm/s;
                desired_speed = limit_acceleration(get_loiter_speed(), 60.0); // cm/s;
                calc_pitch_out(desired_speed);
            }else{
                // manual control
                pitch_out_left  = (get_stabilize_pitch(g.rc_2.control_in) + get_velocity_pitch());
                pitch_out_right = pitch_out_left;
            }
            break;

        case ROLL_PITCH_FBW:
            // hold position if we let go of sticks
            if(g.rc_2.control_in == 0){
                if(!ap.position_hold){
                    ap.position_hold = true;
                    set_destination(encoder_nav.get_position());
                    _reached_destination = true;
                }
                // we are in position hold
                desired_speed = limit_acceleration(get_loiter_speed(), 60.0); // cm/s;
            } else{
                ap.position_hold = false;
                // calc speed of bot                
                desired_speed = ((float)g.rc_2.control_in / (float)MAX_INPUT_PITCH_ANGLE) * -g.waypoint_speed;
				// limit speed
				desired_speed   = limit_acceleration(desired_speed, 200.0); // cm/s
            }
            calc_pitch_out(desired_speed);
            break;

        case ROLL_PITCH_AUTO:
            // calc speed of bot
            if(nav_mode == NAV_WP){
                desired_speed  = get_desired_wp_speed();
				desired_speed   = limit_acceleration(desired_speed, 200.0); // cm/s
                ap.position_hold = false;
                //are we stuck?
                check_stall();

            }else if (nav_mode == NAV_LOITER){
                desired_speed = limit_acceleration(get_loiter_speed(), 60.0); // cm/s;
                ap.position_hold = true;
                reset_stall_checker();

            }else if (nav_mode == NAV_AVOID_BACK){
                desired_speed = limit_acceleration(-100, 200.0); // cm/s;
                // clear obstacle counter
                reset_stall_checker();
                if(distance > 100){ //roll back 1 meter
                    nav_mode = NAV_AVOID_TURN;
                    distance = 0;
                }

            }else if (nav_mode == NAV_AVOID_TURN){
                desired_speed = limit_acceleration(100, 200.0); // cm/s;
                reset_stall_checker();
                if(distance > 100){
                    nav_mode = NAV_WP;
                }
            }

            calc_pitch_out(desired_speed);
            break;
    }

    if(ap_system.new_radio_frame) {
        // clear new radio frame info
        ap_system.new_radio_frame = false;
    }
}



static void read_AHRS(void)
{
    // Perform IMU calculations and get attitude info
    //-----------------------------------------------
    ahrs.update();
    omega = ins.get_gyro();

#if SECONDARY_DMP_ENABLED == ENABLED
    ahrs2.update();
#endif
}

static void update_trig(void){
    Vector2f yawvector;
    const Matrix3f &temp   = ahrs.get_dcm_matrix();

    yawvector.x     = temp.a.x; // sin
    yawvector.y     = temp.b.x; // cos
    yawvector.normalize();

    sin_yaw         = constrain(yawvector.y, -1.0, 1.0);
    cos_yaw         = constrain(yawvector.x, -1.0, 1.0);

    //flat:
    // 0 ° = cos_yaw:  1.00, sin_yaw:  0.00,
    // 90° = cos_yaw:  0.00, sin_yaw:  1.00,
    // 180 = cos_yaw: -1.00, sin_yaw:  0.00,
    // 270 = cos_yaw:  0.00, sin_yaw: -1.00,
}

static void tuning(){
    tuning_value = (float)g.rc_6.control_in / 1000.0f;
    g.rc_6.set_range(g.radio_tuning_low,g.radio_tuning_high);                   // 0 to 1

    switch(g.radio_tuning) {

    case 0:
        break;

    }
}

AP_HAL_MAIN();

