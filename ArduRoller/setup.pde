// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#if CLI_ENABLED == ENABLED

// Functions called from the setup menu
static int8_t   setup_radio             (uint8_t argc, const Menu::arg *argv);
static int8_t   setup_motors            (uint8_t argc, const Menu::arg *argv);
static int8_t   setup_accel             (uint8_t argc, const Menu::arg *argv);
static int8_t   setup_accel_scale       (uint8_t argc, const Menu::arg *argv);
static int8_t   setup_factory           (uint8_t argc, const Menu::arg *argv);
static int8_t   setup_erase             (uint8_t argc, const Menu::arg *argv);
static int8_t   setup_flightmodes       (uint8_t argc, const Menu::arg *argv);
static int8_t   setup_batt_monitor      (uint8_t argc, const Menu::arg *argv);
static int8_t   setup_sonar             (uint8_t argc, const Menu::arg *argv);
static int8_t   setup_compass           (uint8_t argc, const Menu::arg *argv);
static int8_t   setup_compassmot        (uint8_t argc, const Menu::arg *argv);
static int8_t   setup_tune              (uint8_t argc, const Menu::arg *argv);
static int8_t   setup_range             (uint8_t argc, const Menu::arg *argv);
static int8_t   setup_declination       (uint8_t argc, const Menu::arg *argv);
static int8_t   setup_set               (uint8_t argc, const Menu::arg *argv);

// Command/function table for the setup menu
const struct Menu::command setup_menu_commands[] PROGMEM = {
    // command			function called
    // =======          ===============
    {"erase",                       setup_erase},
    {"reset",                       setup_factory},
    {"radio",                       setup_radio},
    {"motors",                      setup_motors},
    {"level",                       setup_accel},
    {"accel",                       setup_accel_scale},
    {"modes",                       setup_flightmodes},
    {"battery",                     setup_batt_monitor},
    {"sonar",                       setup_sonar},
    {"compass",                     setup_compass},
    {"compassmot",                  setup_compassmot},
    {"tune",                        setup_tune},
    {"range",                       setup_range},
    {"declination",         setup_declination},
    {"show",                        setup_show},
    {"set",                         setup_set}
};

// Create the setup menu object.
MENU(setup_menu, "setup", setup_menu_commands);

// Called from the top-level menu to run the setup menu.
static int8_t
setup_mode(uint8_t argc, const Menu::arg *argv)
{
    // Give the user some guidance
    cliSerial->printf_P(PSTR("Setup Mode\n\n\n"));

    if(g.rc_1.radio_min >= 1300) {
        delay(1000);
        cliSerial->printf_P(PSTR("\n!Warning, radio not configured!"));
        delay(1000);
        cliSerial->printf_P(PSTR("\n Type 'radio' now.\n\n"));
    }

    // Run the setup menu.  When the menu exits, we will return to the main menu.
    setup_menu.run();
    return 0;
}

// Print the current configuration.
// Called by the setup menu 'show' command.
static int8_t
setup_show(uint8_t argc, const Menu::arg *argv)
{
    AP_Param *param;
    ap_var_type type;

    //If a parameter name is given as an argument to show, print only that parameter
    if(argc>=2)
    {

        param=AP_Param::find(argv[1].str, &type);

        if(!param)
        {
            cliSerial->printf_P(PSTR("Parameter not found: '%s'\n"), argv[1]);
            return 0;
        }
        AP_Param::show(param, argv[1].str, type, cliSerial);
        return 0;
    }

    // clear the area
    print_blanks(8);

    report_version();
    report_radio();
    report_frame();
    report_batt_monitor();
    report_sonar();
    report_flight_modes();
    report_ins();
    report_compass();
    AP_Param::show_all(cliSerial);

    return(0);
}

// Initialise the EEPROM to 'factory' settings (mostly defined in APM_Config.h or via defaults).
// Called by the setup menu 'factoryreset' command.
static int8_t
setup_factory(uint8_t argc, const Menu::arg *argv)
{
    int16_t c;

    cliSerial->printf_P(PSTR("\n'Y' = factory reset, any other key to abort:\n"));

    do {
        c = cliSerial->read();
    } while (-1 == c);

    if (('y' != c) && ('Y' != c))
        return(-1);

    AP_Param::erase_all();
    cliSerial->printf_P(PSTR("\nReboot APM"));

    delay(1000);

    for (;; ) {
    }
    // note, cannot actually return here
    return(0);
}

// Perform radio setup.
// Called by the setup menu 'radio' command.
static int8_t
setup_radio(uint8_t argc, const Menu::arg *argv)
{
    cliSerial->println_P(PSTR("\n\nRadio Setup:"));
    uint8_t i;

    for(i = 0; i < 100; i++) {
        delay(20);
        read_radio();
    }

    if(g.rc_1.radio_in < 500) {
        while(1) {
            //cliSerial->printf_P(PSTR("\nNo radio; Check connectors."));
            delay(1000);
            // stop here
        }
    }

    g.rc_1.radio_min = g.rc_1.radio_in;
    g.rc_2.radio_min = g.rc_2.radio_in;
    g.rc_3.radio_min = g.rc_3.radio_in;
    g.rc_4.radio_min = g.rc_4.radio_in;
    g.rc_5.radio_min = g.rc_5.radio_in;
    g.rc_6.radio_min = g.rc_6.radio_in;
    g.rc_7.radio_min = g.rc_7.radio_in;
    g.rc_8.radio_min = g.rc_8.radio_in;

    g.rc_1.radio_max = g.rc_1.radio_in;
    g.rc_2.radio_max = g.rc_2.radio_in;
    g.rc_3.radio_max = g.rc_3.radio_in;
    g.rc_4.radio_max = g.rc_4.radio_in;
    g.rc_5.radio_max = g.rc_5.radio_in;
    g.rc_6.radio_max = g.rc_6.radio_in;
    g.rc_7.radio_max = g.rc_7.radio_in;
    g.rc_8.radio_max = g.rc_8.radio_in;

    g.rc_1.radio_trim = g.rc_1.radio_in;
    g.rc_2.radio_trim = g.rc_2.radio_in;
    g.rc_4.radio_trim = g.rc_4.radio_in;
    // 3 is not trimed
    g.rc_5.radio_trim = 1500;
    g.rc_6.radio_trim = 1500;
    g.rc_7.radio_trim = 1500;
    g.rc_8.radio_trim = 1500;


    cliSerial->printf_P(PSTR("\nMove all controls to extremes. Enter to save: "));
    while(1) {

        delay(20);
        // Filters radio input - adjust filters in the radio.pde file
        // ----------------------------------------------------------
        read_radio();

        g.rc_1.update_min_max();
        g.rc_2.update_min_max();
        g.rc_3.update_min_max();
        g.rc_4.update_min_max();
        g.rc_5.update_min_max();
        g.rc_6.update_min_max();
        g.rc_7.update_min_max();
        g.rc_8.update_min_max();

        if(cliSerial->available() > 0) {
            delay(20);
            while (cliSerial->read() != -1); /* flush */

            g.rc_1.save_eeprom();
            g.rc_2.save_eeprom();
            g.rc_3.save_eeprom();
            g.rc_4.save_eeprom();
            g.rc_5.save_eeprom();
            g.rc_6.save_eeprom();
            g.rc_7.save_eeprom();
            g.rc_8.save_eeprom();

            print_done();
            break;
        }
    }
    report_radio();
    return(0);
}

static int8_t
setup_motors(uint8_t argc, const Menu::arg *argv)
{
	return 0;
}

static int8_t
setup_accel(uint8_t argc, const Menu::arg *argv)
{
    ahrs.init();
    ins.init(AP_InertialSensor::COLD_START,
             ins_sample_rate,
             flash_leds);
    ins.init_accel(flash_leds);
    ahrs.set_trim(Vector3f(0,0,0));     // clear out saved trim
    report_ins();
    return(0);
}

static int8_t
setup_accel_scale(uint8_t argc, const Menu::arg *argv)
{
    float trim_roll, trim_pitch;

    cliSerial->println_P(PSTR("Initialising gyros"));
    ahrs.init();
    ins.init(AP_InertialSensor::COLD_START,
             ins_sample_rate,
             flash_leds);
    AP_InertialSensor_UserInteractStream interact(hal.console);
    if(ins.calibrate_accel(flash_leds, &interact, trim_roll, trim_pitch)) {
        // reset ahrs's trim to suggested values from calibration routine
        ahrs.set_trim(Vector3f(trim_roll, trim_pitch, 0));
    }
    report_ins();
    return(0);
}

static int8_t
setup_flightmodes(uint8_t argc, const Menu::arg *argv)
{
    uint8_t _switchPosition = 0;
    uint8_t _oldSwitchPosition = 0;
    int8_t mode = 0;

    cliSerial->printf_P(PSTR("\nMode switch to edit, aileron: select modes\n"));
    print_hit_enter();

    while(1) {
        delay(20);
        read_radio();
        _switchPosition = readSwitch();


        // look for control switch change
        if (_oldSwitchPosition != _switchPosition) {

            mode = flight_modes[_switchPosition];
            mode = constrain_int16(mode, 0, NUM_MODES-1);

            // update the user
            print_switch(_switchPosition, mode);

            // Remember switch position
            _oldSwitchPosition = _switchPosition;
        }

        // look for stick input
        if (abs(g.rc_1.control_in) > 3000) {
            mode++;
            if(mode >= NUM_MODES)
                mode = 0;

            // save new mode
            flight_modes[_switchPosition] = mode;

            // print new mode
            print_switch(_switchPosition, mode);
            delay(500);
        }

        // escape hatch
        if(cliSerial->available() > 0) {
            for (mode = 0; mode < 6; mode++)
                flight_modes[mode].save();

            print_done();
            report_flight_modes();
            return (0);
        }
    }
}

static int8_t
setup_declination(uint8_t argc, const Menu::arg *argv)
{
    compass.set_declination(radians(argv[1].f));
    report_compass();
    return 0;
}

static int8_t
setup_tune(uint8_t argc, const Menu::arg *argv)
{
    g.radio_tuning.set_and_save(argv[1].i);
    report_tuning();
    return 0;
}

static int8_t
setup_range(uint8_t argc, const Menu::arg *argv)
{
    cliSerial->printf_P(PSTR("\nCH 6 Ranges are divided by 1000: [low, high]\n"));

    g.radio_tuning_low.set_and_save(argv[1].i);
    g.radio_tuning_high.set_and_save(argv[2].i);
    report_tuning();
    return 0;
}



static int8_t
setup_erase(uint8_t argc, const Menu::arg *argv)
{
    zero_eeprom();
    return 0;
}

static int8_t
setup_compass(uint8_t argc, const Menu::arg *argv)
{
    if (!strcmp_P(argv[1].str, PSTR("on"))) {
        g.compass_enabled.set_and_save(true);
        init_compass();

    } else if (!strcmp_P(argv[1].str, PSTR("off"))) {
        clear_offsets();
        g.compass_enabled.set_and_save(false);

    }else{
        cliSerial->printf_P(PSTR("\nOp:[on,off]\n"));
        report_compass();
        return 0;
    }

    g.compass_enabled.save();
    report_compass();
    return 0;
}

// setup_compassmot - sets compass's motor interference parameters
static int8_t
setup_compassmot(uint8_t argc, const Menu::arg *argv)
{
    int8_t   comp_type;             // throttle or current based compensation
    Vector3f compass_base;          // compass vector when throttle is zero
    Vector3f motor_impact;          // impact of motors on compass vector
    Vector3f motor_impact_scaled;   // impact of motors on compass vector scaled with throttle
    Vector3f motor_compensation;    // final compensation to be stored to eeprom
    float    throttle_pct;          // throttle as a percentage 0.0 ~ 1.0
    uint32_t last_run_time;
    uint8_t  print_counter = 49;
    bool     updated = false;       // have we updated the compensation vector at least once

    // default compensation type to use current if possible
    if( g.battery_monitoring == BATT_MONITOR_VOLTAGE_AND_CURRENT ) {
        comp_type = AP_COMPASS_MOT_COMP_CURRENT;
    }else{
        comp_type = AP_COMPASS_MOT_COMP_THROTTLE;
    }

    // check if user wants throttle compensation
    if( !strcmp_P(argv[1].str, PSTR("t")) || !strcmp_P(argv[1].str, PSTR("T")) ) {
        comp_type = AP_COMPASS_MOT_COMP_THROTTLE;
    }

    // check if user wants current compensation
    if( !strcmp_P(argv[1].str, PSTR("c")) || !strcmp_P(argv[1].str, PSTR("C")) ) {
        comp_type = AP_COMPASS_MOT_COMP_CURRENT;
    }

    // check compass is enabled
    if( !g.compass_enabled ) {
        cliSerial->print_P(PSTR("compass disabled, exiting"));
        return 0;
    }

    // check if we have a current monitor
    if( comp_type == AP_COMPASS_MOT_COMP_CURRENT && g.battery_monitoring != BATT_MONITOR_VOLTAGE_AND_CURRENT ) {
        cliSerial->print_P(PSTR("current monitor disabled, exiting"));
        return 0;
    }

    // initialise compass
    init_compass();

    // disable motor compensation
    compass.motor_compensation_type(AP_COMPASS_MOT_COMP_DISABLED);
    compass.set_motor_compensation(Vector3f(0,0,0));

    // print warning that motors will spin
    // ask user to raise throttle
    // inform how to stop test
    cliSerial->print_P(PSTR("This setup records the impact on the compass of spinning up the motors.  The motors will spin!\nHold throttle low, then raise as high as safely possible for 10 sec.\nAt any time you may press any key to exit.\nmeasuring compass vs "));

    // inform what type of compensation we are attempting
    if( comp_type == AP_COMPASS_MOT_COMP_CURRENT ) {
        cliSerial->print_P(PSTR("CURRENT\n"));
    }else{
        cliSerial->print_P(PSTR("THROTTLE\n"));
    }

    // clear out user input
    while( cliSerial->available() ) {
        cliSerial->read();
    }

    // disable throttle and battery failsafe
    g.failsafe_battery_enabled = false;

    // read radio
    read_radio();

    // exit immediately if throttle is not zero
    if( g.rc_3.control_in != 0 ) {
        cliSerial->print_P(PSTR("throttle not zero, exiting"));
        return 0;
    }

    // get some initial compass readings
    last_run_time = millis();
    while( millis() - last_run_time < 2000 ) {
        compass.accumulate();
    }
    compass.read();

    // exit immediately if the compass is not healthy
    if( !compass.healthy ) {
        cliSerial->print_P(PSTR("compass not healthy, exiting"));
        return 0;
    }

    // store initial x,y,z compass values
    compass_base.x = compass.mag_x;
    compass_base.y = compass.mag_y;
    compass_base.z = compass.mag_z;

    // initialise motor compensation
    motor_compensation = Vector3f(0,0,0);

    // clear out any user input
    while( cliSerial->available() ) {
        cliSerial->read();
    }

    // enable motors and pass through throttle
    //motors.enable();
    //motors.armed(true);
    //motors.output_min();

    // initialise run time
    last_run_time = millis();

    // main run while there is no user input and the compass is healthy
    while(!cliSerial->available() && compass.healthy) {

        // 50hz loop
        if( millis() - last_run_time > 20 ) {
            last_run_time = millis();

            // read radio input
            read_radio();

            // pass through throttle to motors
            //motors.throttle_pass_through();

            // read some compass values
            compass.read();

            // read current
            read_battery();

            // calculate scaling for throttle
            throttle_pct = (float)g.rc_3.control_in / 1000.0f;
            throttle_pct = constrain(throttle_pct,0.0f,1.0f);

            // if throttle is zero, update base x,y,z values
            if( throttle_pct == 0.0f ) {
                compass_base.x = compass_base.x * 0.99f + (float)compass.mag_x * 0.01f;
                compass_base.y = compass_base.y * 0.99f + (float)compass.mag_y * 0.01f;
                compass_base.z = compass_base.z * 0.99f + (float)compass.mag_z * 0.01f;

                // causing printing to happen as soon as throttle is lifted
                print_counter = 49;
            }else{

                // calculate diff from compass base and scale with throttle
                motor_impact.x = compass.mag_x - compass_base.x;
                motor_impact.y = compass.mag_y - compass_base.y;
                motor_impact.z = compass.mag_z - compass_base.z;

                // throttle based compensation
                if( comp_type == AP_COMPASS_MOT_COMP_THROTTLE ) {
                    // scale by throttle
                    motor_impact_scaled = motor_impact / throttle_pct;

                    // adjust the motor compensation to negate the impact
                    motor_compensation = motor_compensation * 0.99f - motor_impact_scaled * 0.01f;
                    updated = true;
                }else{
                    // current based compensation if more than 3amps being drawn
                    motor_impact_scaled = motor_impact / current_amps1;

                    // adjust the motor compensation to negate the impact if drawing over 3amps
                    if( current_amps1 >= 3.0f ) {
                        motor_compensation = motor_compensation * 0.99f - motor_impact_scaled * 0.01f;
                        updated = true;
                    }
                }

                // display output at 1hz if throttle is above zero
                print_counter++;
                if(print_counter >= 50) {
                    print_counter = 0;
                    cliSerial->printf_P(PSTR("thr:%d cur:%4.2f mot x:%4.1f y:%4.1f z:%4.1f  comp x:%4.2f y:%4.2f z:%4.2f\n"),(int)g.rc_3.control_in, (float)current_amps1, (float)motor_impact.x, (float)motor_impact.y, (float)motor_impact.z, (float)motor_compensation.x, (float)motor_compensation.y, (float)motor_compensation.z);
                }
            }
        }else{
            // grab some compass values
            compass.accumulate();
        }
    }

    // stop motors
    //motors.output_min();
    //motors.armed(false);

    // clear out any user input
    while( cliSerial->available() ) {
        cliSerial->read();
    }

    // print one more time so the last thing printed matches what appears in the report_compass
    cliSerial->printf_P(PSTR("thr:%d cur:%4.2f mot x:%4.1f y:%4.1f z:%4.1f  comp x:%4.2f y:%4.2f z:%4.2f\n"),(int)g.rc_3.control_in, (float)current_amps1, (float)motor_impact.x, (float)motor_impact.y, (float)motor_impact.z, (float)motor_compensation.x, (float)motor_compensation.y, (float)motor_compensation.z);

    // set and save motor compensation
    if( updated ) {
        compass.motor_compensation_type(comp_type);
        compass.set_motor_compensation(motor_compensation);
        compass.save_motor_compensation();
    }else{
        // compensation vector never updated, report failure
        cliSerial->printf_P(PSTR("Failed! Compensation disabled.  Did you forget to raise the throttle high enough?"));
        compass.motor_compensation_type(AP_COMPASS_MOT_COMP_DISABLED);
    }

    // display new motor offsets and save
    report_compass();

    return 0;
}

static int8_t
setup_batt_monitor(uint8_t argc, const Menu::arg *argv)
{
    if (!strcmp_P(argv[1].str, PSTR("off"))) {
        g.battery_monitoring.set_and_save(0);

    } else if(argv[1].i > 0 && argv[1].i <= 4) {
        g.battery_monitoring.set_and_save(argv[1].i);

    } else {
        cliSerial->printf_P(PSTR("\nOp: off, 3-4"));
    }

    report_batt_monitor();
    return 0;
}

static int8_t
setup_sonar(uint8_t argc, const Menu::arg *argv)
{
    if (!strcmp_P(argv[1].str, PSTR("on"))) {
        g.sonar_enabled.set_and_save(true);

    } else if (!strcmp_P(argv[1].str, PSTR("off"))) {
        g.sonar_enabled.set_and_save(false);

    } else if (argc > 1 && (argv[1].i >= 0 && argv[1].i <= 3)) {
        g.sonar_enabled.set_and_save(true);      // if you set the sonar type, surely you want it on
        g.sonar_type.set_and_save(argv[1].i);

    }else{
        cliSerial->printf_P(PSTR("\nOp:[on, off, 0-3]\n"));
        report_sonar();
        return 0;
    }

    report_sonar();
    return 0;
}

static void clear_offsets()
{
    Vector3f _offsets(0.0,0.0,0.0);
    compass.set_offsets(_offsets);
    compass.save_offsets();
}

//Set a parameter to a specified value. It will cast the value to the current type of the
//parameter and make sure it fits in case of INT8 and INT16
static int8_t setup_set(uint8_t argc, const Menu::arg *argv)
{
    int8_t value_int8;
    int16_t value_int16;

    AP_Param *param;
    enum ap_var_type p_type;

    if(argc!=3)
    {
        cliSerial->printf_P(PSTR("Invalid command. Usage: set <name> <value>\n"));
        return 0;
    }

    param = AP_Param::find(argv[1].str, &p_type);
    if(!param)
    {
        cliSerial->printf_P(PSTR("Param not found: %s\n"), argv[1].str);
        return 0;
    }

    switch(p_type)
    {
        case AP_PARAM_INT8:
            value_int8 = (int8_t)(argv[2].i);
            if(argv[2].i!=value_int8)
            {
                cliSerial->printf_P(PSTR("Value out of range for type INT8\n"));
                return 0;
            }
            ((AP_Int8*)param)->set_and_save(value_int8);
            break;
        case AP_PARAM_INT16:
            value_int16 = (int16_t)(argv[2].i);
            if(argv[2].i!=value_int16)
            {
                cliSerial->printf_P(PSTR("Value out of range for type INT16\n"));
                return 0;
            }
            ((AP_Int16*)param)->set_and_save(value_int16);
            break;

        //int32 and float don't need bounds checking, just use the value provoded by Menu::arg
        case AP_PARAM_INT32:
            ((AP_Int32*)param)->set_and_save(argv[2].i);
            break;
        case AP_PARAM_FLOAT:
            ((AP_Float*)param)->set_and_save(argv[2].f);
            break;
        default:
            cliSerial->printf_P(PSTR("Cannot set parameter of type %d.\n"), p_type);
            break;
    }

    return 0;
}

/***************************************************************************/
// CLI reports
/***************************************************************************/

static void report_batt_monitor()
{
    cliSerial->printf_P(PSTR("\nBatt Mon:\n"));
    print_divider();
    if(g.battery_monitoring == BATT_MONITOR_DISABLED) print_enabled(false);
    if(g.battery_monitoring == BATT_MONITOR_VOLTAGE_ONLY) cliSerial->printf_P(PSTR("volts"));
    if(g.battery_monitoring == BATT_MONITOR_VOLTAGE_AND_CURRENT) cliSerial->printf_P(PSTR("volts and cur"));
    print_blanks(2);
}

static void report_wp(uint8_t index = 255)
{
    if(index == 255) {
        for(uint8_t i = 0; i < g.command_total; i++) {
            struct Location temp = get_cmd_with_index(i);
            print_wp(&temp, i);
        }
    }else{
        struct Location temp = get_cmd_with_index(index);
        print_wp(&temp, index);
    }
}

static void report_sonar()
{
    cliSerial->printf_P(PSTR("Sonar\n"));
    print_divider();
    print_enabled(g.sonar_enabled.get());
    cliSerial->printf_P(PSTR("Type: %d (0=XL, 1=LV, 2=XLL, 3=HRLV)"), (int)g.sonar_type);
    print_blanks(2);
}

static void report_frame()
{}

static void report_radio()
{
    cliSerial->printf_P(PSTR("Radio\n"));
    print_divider();
    // radio
    print_radio_values();
    print_blanks(2);
}

static void report_ins()
{
    cliSerial->printf_P(PSTR("INS\n"));
    print_divider();

    print_gyro_offsets();
    print_accel_offsets_and_scaling();
    print_blanks(2);
}

static void report_compass()
{
    cliSerial->printf_P(PSTR("Compass\n"));
    print_divider();

    print_enabled(g.compass_enabled);

    // mag declination
    cliSerial->printf_P(PSTR("Mag Dec: %4.4f\n"),
                    degrees(compass.get_declination()));

    Vector3f offsets = compass.get_offsets();

    // mag offsets
    cliSerial->printf_P(PSTR("Mag off: %4.4f, %4.4f, %4.4f\n"),
                    offsets.x,
                    offsets.y,
                    offsets.z);

    // motor compensation
    cliSerial->print_P(PSTR("Motor Comp: "));
    if( compass.motor_compensation_type() == AP_COMPASS_MOT_COMP_DISABLED ) {
        cliSerial->print_P(PSTR("Off\n"));
    }else{
        if( compass.motor_compensation_type() == AP_COMPASS_MOT_COMP_THROTTLE ) {
            cliSerial->print_P(PSTR("Throttle"));
        }
        if( compass.motor_compensation_type() == AP_COMPASS_MOT_COMP_CURRENT ) {
            cliSerial->print_P(PSTR("Current"));
        }
        Vector3f motor_compensation = compass.get_motor_compensation();
        cliSerial->printf_P(PSTR("\nComp Vec: %4.2f, %4.2f, %4.2f\n"),
                        motor_compensation.x,
                        motor_compensation.y,
                        motor_compensation.z);
    }
    print_blanks(1);
}

static void report_flight_modes()
{
    cliSerial->printf_P(PSTR("Flight modes\n"));
    print_divider();

    for(int16_t i = 0; i < 6; i++ ) {
        print_switch(i, flight_modes[i]);
    }
    print_blanks(2);
}



/***************************************************************************/
// CLI utilities
/***************************************************************************/

/*static void
 *  print_PID(PI * pid)
 *  {
 *       cliSerial->printf_P(PSTR("P: %4.2f, I:%4.2f, IMAX:%ld\n"),
 *                                               pid->kP(),
 *                                               pid->kI(),
 *                                               (long)pid->imax());
 *  }
 */

static void
print_radio_values()
{
    cliSerial->printf_P(PSTR("CH1: %d | %d\n"), (int)g.rc_1.radio_min, (int)g.rc_1.radio_max);
    cliSerial->printf_P(PSTR("CH2: %d | %d\n"), (int)g.rc_2.radio_min, (int)g.rc_2.radio_max);
    cliSerial->printf_P(PSTR("CH3: %d | %d\n"), (int)g.rc_3.radio_min, (int)g.rc_3.radio_max);
    cliSerial->printf_P(PSTR("CH4: %d | %d\n"), (int)g.rc_4.radio_min, (int)g.rc_4.radio_max);
    cliSerial->printf_P(PSTR("CH5: %d | %d\n"), (int)g.rc_5.radio_min, (int)g.rc_5.radio_max);
    cliSerial->printf_P(PSTR("CH6: %d | %d\n"), (int)g.rc_6.radio_min, (int)g.rc_6.radio_max);
    cliSerial->printf_P(PSTR("CH7: %d | %d\n"), (int)g.rc_7.radio_min, (int)g.rc_7.radio_max);
    //cliSerial->printf_P(PSTR("CH8: %d | %d\n"), (int)g.rc_8.radio_min, (int)g.rc_8.radio_max);
}

static void
print_switch(uint8_t p, uint8_t m)
{
    cliSerial->printf_P(PSTR("Pos %d:\t"),p);
    print_flight_mode(cliSerial, m);
    cliSerial->println();
}

static void
print_done()
{
    cliSerial->printf_P(PSTR("\nSaved\n"));
}


static void zero_eeprom(void)
{
    cliSerial->printf_P(PSTR("\nErasing EEPROM\n"));

    for (uint16_t i = 0; i < EEPROM_MAX_ADDR; i++) {
        hal.storage->write_byte(i, 0);
    }

    cliSerial->printf_P(PSTR("done\n"));
}

static void
print_accel_offsets_and_scaling(void)
{
    Vector3f accel_offsets = ins.get_accel_offsets();
    Vector3f accel_scale = ins.get_accel_scale();
    cliSerial->printf_P(PSTR("A_off: %4.2f, %4.2f, %4.2f\nA_scale: %4.2f, %4.2f, %4.2f\n"),
                    (float)accel_offsets.x,                           // Pitch
                    (float)accel_offsets.y,                           // Roll
                    (float)accel_offsets.z,                           // YAW
                    (float)accel_scale.x,                             // Pitch
                    (float)accel_scale.y,                             // Roll
                    (float)accel_scale.z);                            // YAW
}

static void
print_gyro_offsets(void)
{
    Vector3f gyro_offsets = ins.get_gyro_offsets();
    cliSerial->printf_P(PSTR("G_off: %4.2f, %4.2f, %4.2f\n"),
                    (float)gyro_offsets.x,
                    (float)gyro_offsets.y,
                    (float)gyro_offsets.z);
}

#endif // CLI_ENABLED

static void
print_blanks(int16_t num)
{
    while(num > 0) {
        num--;
        cliSerial->println("");
    }
}

static void
print_divider(void)
{
    for (int i = 0; i < 40; i++) {
        cliSerial->print_P(PSTR("-"));
    }
    cliSerial->println();
}

static void print_enabled(bool b)
{
    if(b)
        cliSerial->print_P(PSTR("en"));
    else
        cliSerial->print_P(PSTR("dis"));
    cliSerial->print_P(PSTR("abled\n"));
}


static void print_wp(const struct Location *cmd, uint8_t index)
{
    cliSerial->printf_P(PSTR("cmd#: %d | %d, %d, %d, %ld, %ld, %ld\n"),
                    index,
                    cmd->id,
                    cmd->options,
                    cmd->p1,
                    cmd->alt,
                    cmd->lat,
                    cmd->lng);
}

static void report_version()
{
    cliSerial->printf_P(PSTR("FW Ver: %d\n"),(int)g.k_format_version);
    print_divider();
    print_blanks(2);
}


static void report_tuning()
{
    cliSerial->printf_P(PSTR("\nTUNE:\n"));
    print_divider();
    if (g.radio_tuning == 0) {
        print_enabled(g.radio_tuning.get());
    }else{
        float low  = (float)g.radio_tuning_low.get() / 1000;
        float high = (float)g.radio_tuning_high.get() / 1000;
        cliSerial->printf_P(PSTR(" %d, Low:%1.4f, High:%1.4f\n"),(int)g.radio_tuning.get(), low, high);
    }
    print_blanks(2);
}
