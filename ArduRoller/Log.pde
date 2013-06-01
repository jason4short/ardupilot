// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// Code to Write and Read packets from DataFlash log memory
// Code to interact with the user to dump or erase logs

// These are function definitions so the Menu can be constructed before the functions
// are defined below. Order matters to the compiler.
static bool     print_log_menu(void);
static int8_t   dump_log(uint8_t argc,                  const Menu::arg *argv);
static int8_t   erase_logs(uint8_t argc,                const Menu::arg *argv);
static int8_t   select_logs(uint8_t argc,               const Menu::arg *argv);

// Creates a constant array of structs representing menu options
// and stores them in Flash memory, not RAM.
// User enters the string in the console to call the functions on the right.
// See class Menu in AP_Coommon for implementation details
const struct Menu::command log_menu_commands[] PROGMEM = {
    {"dump",        dump_log},
    {"erase",       erase_logs},
    {"enable",      select_logs},
    {"disable",     select_logs}
};

// A Macro to create the Menu
MENU2(log_menu, "Log", log_menu_commands, print_log_menu);

static bool
print_log_menu(void)
{
    cliSerial->printf_P(PSTR("logs enabled: "));

    if(0 == g.log_bitmask){
        cliSerial->printf_P(PSTR("none"));
    }else{
        if (g.log_bitmask & MASK_LOG_ATTITUDE_FAST) cliSerial->printf_P(PSTR(" ATTITUDE_FAST"));
        if (g.log_bitmask & MASK_LOG_ATTITUDE_MED) cliSerial->printf_P(PSTR(" ATTITUDE_MED"));
        if (g.log_bitmask & MASK_LOG_GPS) cliSerial->printf_P(PSTR(" GPS"));
        if (g.log_bitmask & MASK_LOG_PM) cliSerial->printf_P(PSTR(" PM"));
        if (g.log_bitmask & MASK_LOG_NTUN) cliSerial->printf_P(PSTR(" NTUN"));
        if (g.log_bitmask & MASK_LOG_IMU) cliSerial->printf_P(PSTR(" IMU"));
        if (g.log_bitmask & MASK_LOG_CMD) cliSerial->printf_P(PSTR(" CMD"));
        if (g.log_bitmask & MASK_LOG_COMPASS) cliSerial->printf_P(PSTR(" COMPASS"));
        if (g.log_bitmask & MASK_LOG_INAV) cliSerial->printf_P(PSTR(" INAV"));
        if (g.log_bitmask & MASK_LOG_CAMERA) cliSerial->printf_P(PSTR(" CAMERA"));
    }

    cliSerial->println();
    DataFlash.ListAvailableLogs(cliSerial);
    return(true);
}

static int8_t
dump_log(uint8_t argc, const Menu::arg *argv)
{
    int16_t dump_log;
    uint16_t dump_log_start;
    uint16_t dump_log_end;
    uint16_t last_log_num;

    // check that the requested log number can be read
    dump_log = argv[1].i;
    last_log_num = DataFlash.find_last_log();

    if (dump_log == -2) {
        DataFlash.DumpPageInfo(cliSerial);
        return(-1);
    } else if (dump_log <= 0) {
        cliSerial->printf_P(PSTR("dumping all\n"));
        Log_Read(0, 1, 0);
        return(-1);
    } else if ((argc != 2) || (dump_log <= (int16_t)(last_log_num - DataFlash.get_num_logs())) || (dump_log > (int16_t)last_log_num)) {
        cliSerial->printf_P(PSTR("bad log number\n"));
        return(-1);
    }

    DataFlash.get_log_boundaries(dump_log, dump_log_start, dump_log_end);
    Log_Read((uint16_t)dump_log, dump_log_start, dump_log_end);
    return (0);
}

static void do_erase_logs(void)
{
	gcs_send_text_P(SEVERITY_LOW, PSTR("Erasing logs\n"));
    DataFlash.EraseAll();
	gcs_send_text_P(SEVERITY_LOW, PSTR("Log erase complete\n"));
}

static int8_t
erase_logs(uint8_t argc, const Menu::arg *argv)
{
    in_mavlink_delay = true;
    do_erase_logs();
    in_mavlink_delay = false;
    return 0;
}

static int8_t
select_logs(uint8_t argc, const Menu::arg *argv)
{
    uint16_t bits;

    if (argc != 2) {
        cliSerial->printf_P(PSTR("missing log type\n"));
        return(-1);
    }

    bits = 0;

    // Macro to make the following code a bit easier on the eye.
    // Pass it the capitalised name of the log option, as defined
    // in defines.h but without the LOG_ prefix.  It will check for
    // that name as the argument to the command, and set the bit in
    // bits accordingly.
    //
    if (!strcasecmp_P(argv[1].str, PSTR("all"))) {
        bits = ~0;
    } else {
 #define TARG(_s)        if (!strcasecmp_P(argv[1].str, PSTR(# _s))) bits |= MASK_LOG_ ## _s
        TARG(ATTITUDE_FAST);
        TARG(ATTITUDE_MED);
        TARG(GPS);
        TARG(PM);
        TARG(NTUN);
        TARG(MODE);
        TARG(IMU);
        TARG(CMD);
        TARG(COMPASS);
        TARG(INAV);
        TARG(CAMERA);
 #undef TARG
    }

    if (!strcasecmp_P(argv[0].str, PSTR("enable"))) {
        g.log_bitmask.set_and_save(g.log_bitmask | bits);
    }else{
        g.log_bitmask.set_and_save(g.log_bitmask & ~bits);
    }

    return(0);
}

static int8_t
process_logs(uint8_t argc, const Menu::arg *argv)
{
    log_menu.run();
    return 0;
}


struct PACKED log_Compass {
    LOG_PACKET_HEADER;
    int16_t mag_x;
    int16_t mag_y;
    int16_t mag_z;
    int16_t offset_x;
    int16_t offset_y;
    int16_t offset_z;
    int16_t motor_offset_x;
    int16_t motor_offset_y;
    int16_t motor_offset_z;
};

// Write a Compass packet
static void Log_Write_Compass()
{
    Vector3f mag_offsets = compass.get_offsets();
    Vector3f mag_motor_offsets = compass.get_motor_offsets();
    struct log_Compass pkt = {
        LOG_PACKET_HEADER_INIT(LOG_COMPASS_MSG),
        mag_x           : compass.mag_x,
        mag_y           : compass.mag_y,
        mag_z           : compass.mag_z,
        offset_x        : (int16_t)mag_offsets.x,
        offset_y        : (int16_t)mag_offsets.y,
        offset_z        : (int16_t)mag_offsets.z,
        motor_offset_x  : (int16_t)mag_motor_offsets.x,
        motor_offset_y  : (int16_t)mag_motor_offsets.y,
        motor_offset_z  : (int16_t)mag_motor_offsets.z
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

struct PACKED log_Performance {
    LOG_PACKET_HEADER;
    uint8_t renorm_count;
    uint8_t renorm_blowup;
    uint16_t num_long_running;
    uint16_t num_loops;
    uint32_t max_time;
    int16_t  pm_test;
    uint8_t i2c_lockup_count;
};

// Write a performance monitoring packet
static void Log_Write_Performance()
{
    struct log_Performance pkt = {
        LOG_PACKET_HEADER_INIT(LOG_PERFORMANCE_MSG),
        renorm_count     : ahrs.renorm_range_count,
        renorm_blowup    : ahrs.renorm_blowup_count,
        num_long_running : perf_info_get_num_long_running(),
        num_loops        : perf_info_get_num_loops(),
        max_time         : perf_info_get_max_time(),
        pm_test          : pmTest1,
        i2c_lockup_count : hal.i2c->lockup_count()
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

struct PACKED log_Cmd {
    LOG_PACKET_HEADER;

    uint8_t command_total;
    uint8_t command_number;
    uint8_t waypoint_id;
    uint8_t waypoint_options;
    uint8_t waypoint_param1;
    int32_t waypoint_altitude;
    int32_t waypoint_latitude;
    int32_t waypoint_longitude;
};

// Write a command processing packet
static void Log_Write_Cmd(uint8_t num, const struct Location *wp)
{
    struct log_Cmd pkt = {
        LOG_PACKET_HEADER_INIT(LOG_CMD_MSG),
        command_total       : g.command_total,
        command_number      : num,
        waypoint_id         : wp->id,
        waypoint_options    : wp->options,
        waypoint_param1     : wp->p1,
        waypoint_altitude   : wp->alt,
        waypoint_latitude   : wp->lat,
        waypoint_longitude  : wp->lng
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

struct PACKED log_Attitude {
    LOG_PACKET_HEADER;
    int16_t pitch_in;
    int16_t pitch;
    int16_t yaw_in;
    uint16_t yaw;
    uint16_t nav_yaw;
};

// Write an attitude packet
static void Log_Write_Attitude()
{
    struct log_Attitude pkt = {
        LOG_PACKET_HEADER_INIT(LOG_ATTITUDE_MSG),
        pitch_in    : (int16_t)g.rc_2.control_in,
        pitch       : (int16_t)ahrs.pitch_sensor,
        yaw_in      : (int16_t)g.rc_4.control_in,
        yaw         : (uint16_t)ahrs.yaw_sensor,
        nav_yaw     : (uint16_t)nav_yaw
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

struct PACKED log_INAV {
    LOG_PACKET_HEADER;
	float _pos_est_y;
	float _pos_cor_y;
	float _pos_err_y;
	float _pos_est_x;
	float _pos_cor_x;
	float _pos_err_x;
};



// Write an INAV packet
static void Log_Write_INAV()
{
    struct log_INAV pkt = {
        LOG_PACKET_HEADER_INIT(LOG_INAV_MSG),
		_pos_est_y: encoder_nav._position_estimation.y,
		_pos_cor_y: encoder_nav._position_correction.y,
		_pos_err_y: encoder_nav._position_error.y,
		_pos_est_x: encoder_nav._position_estimation.x,
		_pos_cor_x: encoder_nav._position_correction.x,
		_pos_err_x: encoder_nav._position_error.x,
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

struct PACKED log_Mode {
    LOG_PACKET_HEADER;
    uint8_t mode;
    int16_t throttle_cruise;
};

// Write a mode packet
static void Log_Write_Mode(uint8_t mode)
{
    struct log_Mode pkt = {
        LOG_PACKET_HEADER_INIT(LOG_MODE_MSG),
        mode            : mode,
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

struct PACKED log_Startup {
    LOG_PACKET_HEADER;
};

// Write Startup packet
static void Log_Write_Startup()
{
    struct log_Startup pkt = {
        LOG_PACKET_HEADER_INIT(LOG_STARTUP_MSG)
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

struct PACKED log_Event {
    LOG_PACKET_HEADER;
    uint8_t id;
};

// Wrote an event packet
static void Log_Write_Event(uint8_t id)
{
    if (g.log_bitmask != 0) {
        struct log_Event pkt = {
            LOG_PACKET_HEADER_INIT(LOG_EVENT_MSG),
            id  : id
        };
        DataFlash.WriteBlock(&pkt, sizeof(pkt));
    }
}

struct PACKED log_Data_Int16t {
    LOG_PACKET_HEADER;
    uint8_t id;
    int16_t data_value;
};

// Write an int16_t data packet
static void Log_Write_Data(uint8_t id, int16_t value)
{
    if (g.log_bitmask != 0) {
        struct log_Data_Int16t pkt = {
            LOG_PACKET_HEADER_INIT(LOG_DATA_INT16_MSG),
            id          : id,
            data_value  : value
        };
        DataFlash.WriteBlock(&pkt, sizeof(pkt));
    }
}

struct PACKED log_Data_UInt16t {
    LOG_PACKET_HEADER;
    uint8_t id;
    uint16_t data_value;
};

// Write an uint16_t data packet
static void Log_Write_Data(uint8_t id, uint16_t value)
{
    if (g.log_bitmask != 0) {
        struct log_Data_UInt16t pkt = {
            LOG_PACKET_HEADER_INIT(LOG_DATA_UINT16_MSG),
            id          : id,
            data_value  : value
        };
        DataFlash.WriteBlock(&pkt, sizeof(pkt));
    }
}

struct PACKED log_Data_Int32t {
    LOG_PACKET_HEADER;
    uint8_t id;
    int32_t data_value;
};

// Write an int32_t data packet
static void Log_Write_Data(uint8_t id, int32_t value)
{
    if (g.log_bitmask != 0) {
        struct log_Data_Int32t pkt = {
            LOG_PACKET_HEADER_INIT(LOG_DATA_INT32_MSG),
            id          : id,
            data_value  : value
        };
        DataFlash.WriteBlock(&pkt, sizeof(pkt));
    }
}

struct PACKED log_Data_UInt32t {
    LOG_PACKET_HEADER;
    uint8_t id;
    uint32_t data_value;
};

// Write a uint32_t data packet
static void Log_Write_Data(uint8_t id, uint32_t value)
{
    if (g.log_bitmask != 0) {
        struct log_Data_UInt32t pkt = {
            LOG_PACKET_HEADER_INIT(LOG_DATA_UINT32_MSG),
            id          : id,
            data_value  : value
        };
        DataFlash.WriteBlock(&pkt, sizeof(pkt));
    }
}

struct PACKED log_Data_Float {
    LOG_PACKET_HEADER;
    uint8_t id;
    float data_value;
};

// Write a float data packet
static void Log_Write_Data(uint8_t id, float value)
{
    if (g.log_bitmask != 0) {
        struct log_Data_Float pkt = {
            LOG_PACKET_HEADER_INIT(LOG_DATA_FLOAT_MSG),
            id          : id,
            data_value  : value
        };
        DataFlash.WriteBlock(&pkt, sizeof(pkt));
    }
}

struct PACKED log_DMP {
    LOG_PACKET_HEADER;
    int16_t  dcm_roll;
    int16_t  dmp_roll;
    int16_t  dcm_pitch;
    int16_t  dmp_pitch;
    uint16_t dcm_yaw;
    uint16_t dmp_yaw;
};

#if SECONDARY_DMP_ENABLED == ENABLED
// Write a DMP attitude packet
static void Log_Write_DMP()
{
    struct log_DMP pkt = {
        LOG_PACKET_HEADER_INIT(LOG_DMP_MSG),
        dcm_roll    : (int16_t)ahrs.roll_sensor,
        dmp_roll    : (int16_t)ahrs2.roll_sensor,
        dcm_pitch   : (int16_t)ahrs.pitch_sensor,
        dmp_pitch   : (int16_t)ahrs2.pitch_sensor,
        dcm_yaw     : (uint16_t)ahrs.yaw_sensor,
        dmp_yaw     : (uint16_t)ahrs2.yaw_sensor
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}
#endif

struct PACKED log_Camera {
    LOG_PACKET_HEADER;
    uint32_t gps_time;
    int32_t  latitude;
    int32_t  longitude;
    int32_t  altitude;
    int16_t  roll;
    int16_t  pitch;
    uint16_t yaw;
};


// Write a Camera packet
static void Log_Write_Camera()
{
#if CAMERA == ENABLED
    struct log_Camera pkt = {
        LOG_PACKET_HEADER_INIT(LOG_CAMERA_MSG),
        gps_time    : g_gps->time,
        latitude    : current_loc.lat,
        longitude   : current_loc.lng,
        altitude    : current_loc.alt,
        roll        : (int16_t)ahrs.roll_sensor,
        pitch       : (int16_t)ahrs.pitch_sensor,
        yaw         : (uint16_t)ahrs.yaw_sensor
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
#endif
}

struct PACKED log_Error {
    LOG_PACKET_HEADER;
    uint8_t sub_system;
    uint8_t error_code;
};

// Write an error packet
static void Log_Write_Error(uint8_t sub_system, uint8_t error_code)
{
    struct log_Error pkt = {
        LOG_PACKET_HEADER_INIT(LOG_ERROR_MSG),
        sub_system    : sub_system,
        error_code    : error_code,
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

struct PACKED log_Nav_Tuning {
    LOG_PACKET_HEADER;
    int16_t		wp_dist;
    int32_t		wp_bear;
	int16_t		x_track;
	int16_t		avoid;
	int16_t 	speed;
};

// Write an WPNAV packet
static void Log_Write_NTUN()
{
    Vector3f velocity = encoder_nav.get_velocity();

    struct log_Nav_Tuning pkt = {
        LOG_PACKET_HEADER_INIT(LOG_NAV_TUNING_MSG),
        wp_dist         : (int16_t)wp_distance,
        wp_bear         : wp_bearing,
        x_track 		: _crosstrack_fix,
        avoid			: _avoid_obstacle,
        speed	      	: (int16_t)ground_speed,
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

static const struct LogStructure log_structure[] PROGMEM = {
    LOG_COMMON_STRUCTURES,
    //{ LOG_NAV_TUNING_MSG, sizeof(log_Nav_Tuning),
    //  "NTUN", "Ecffcccc",    "WPDist,TargBrg,LatErr,LngErr,NavPtch,NavRll,LatSpd,LngSpd" },
    { LOG_COMPASS_MSG, sizeof(log_Compass),
      "MAG", "hhhhhhhhh",    "MagX,MagY,MagZ,OfsX,OfsY,OfsZ,MOfsX,MOfsY,MOfsZ" },
    { LOG_PERFORMANCE_MSG, sizeof(log_Performance),
      "PM",  "BBHHIhB",      "RenCnt,RenBlw,NLon,NLoop,MaxT,PMT,I2CErr" },
    { LOG_CMD_MSG, sizeof(log_Cmd),
      "CMD", "BBBBBeLL",     "CTot,CNum,CId,COpt,Prm1,Alt,Lat,Lng" },
    { LOG_ATTITUDE_MSG, sizeof(log_Attitude),
      "ATT", "cccccCC",      "RollIn,Roll,PitchIn,Pitch,YawIn,Yaw,NavYaw" },
    { LOG_MODE_MSG, sizeof(log_Mode),
      "MODE", "M",          "Mode" },
    { LOG_STARTUP_MSG, sizeof(log_Startup),
      "STRT", "",            "" },
    { LOG_EVENT_MSG, sizeof(log_Event),
      "EV",   "B",           "Id" },
    { LOG_DATA_INT16_MSG, sizeof(log_Data_Int16t),
      "D16",   "Bh",         "Id,Value" },
    { LOG_DATA_UINT16_MSG, sizeof(log_Data_UInt16t),
      "DU16",  "BH",         "Id,Value" },
    { LOG_DATA_INT32_MSG, sizeof(log_Data_Int32t),
      "D32",   "Bi",         "Id,Value" },
    { LOG_DATA_UINT32_MSG, sizeof(log_Data_UInt32t),
      "DU32",  "BI",         "Id,Value" },
    { LOG_DATA_FLOAT_MSG, sizeof(log_Data_Float),
      "DFLT",  "Bf",         "Id,Value" },
    { LOG_DMP_MSG, sizeof(log_DMP),
      "DMP",   "ccccCC",     "DCMRoll,DMPRoll,DCMPtch,DMPPtch,DCMYaw,DMPYaw" },
    { LOG_CAMERA_MSG, sizeof(log_Camera),
      "CAM",   "ILLeccC",    "GPSTime,Lat,Lng,Alt,Roll,Pitch,Yaw" },
    { LOG_ERROR_MSG, sizeof(log_Error),
      "ERR",   "BB",         "Subsys,ECode" },
    { LOG_INAV_MSG, sizeof(log_INAV),
      "INAV", "ffffff",  	  "est_y,cor_y,err_y,est_x,cor_x,err_x" },
    { LOG_NAV_TUNING_MSG, sizeof(log_Nav_Tuning),
      "NTUN",  "hihhh", "WPDist,TargBrg,Xtrack,Avoid,Speed" },
};

// Read the DataFlash log memory
static void Log_Read(uint16_t log_num, uint16_t start_page, uint16_t end_page)
{
 #ifdef AIRFRAME_NAME
    cliSerial->printf_P(PSTR((AIRFRAME_NAME)));
 #endif

    cliSerial->printf_P(PSTR("\n" THISFIRMWARE
                             "\nFree RAM: %u\n"),
                        (unsigned) memcheck_available_memory());

    cliSerial->println_P(PSTR(HAL_BOARD_NAME));

	DataFlash.LogReadProcess(log_num, start_page, end_page,
                             sizeof(log_structure)/sizeof(log_structure[0]),
                             log_structure,
                             print_flight_mode,
                             cliSerial);
}

// start a new log
static void start_logging()
{
    DataFlash.StartNewLog(sizeof(log_structure)/sizeof(log_structure[0]), log_structure);
}
