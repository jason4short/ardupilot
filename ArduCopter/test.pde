// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#if CLI_ENABLED == ENABLED

// These are function definitions so the Menu can be constructed before the functions
// are defined below. Order matters to the compiler.
#if HIL_MODE == HIL_MODE_DISABLED
static int8_t   test_baro(uint8_t argc,                 const Menu::arg *argv);
#endif
static int8_t   test_radio_pwm(uint8_t argc,            const Menu::arg *argv);
static int8_t   test_radio(uint8_t argc,                const Menu::arg *argv);
static int8_t   test_compass(uint8_t argc,              const Menu::arg *argv);
static int8_t   test_ins(uint8_t argc,                  const Menu::arg *argv);
static int8_t   test_optflow(uint8_t argc,              const Menu::arg *argv);
static int8_t   test_relay(uint8_t argc,                const Menu::arg *argv);
static int8_t   test_input(uint8_t argc,                const Menu::arg *argv);
static int8_t   test_motor(uint8_t argc,                const Menu::arg *argv);
static int8_t   test_motor2(uint8_t argc,                const Menu::arg *argv);
static int8_t   test_motor3(uint8_t argc,                const Menu::arg *argv);
static int8_t   test_gimbal(uint8_t argc,                const Menu::arg *argv);
static int8_t   test_gimbal2(uint8_t argc,                const Menu::arg *argv);

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
static int8_t   test_shell(uint8_t argc,                const Menu::arg *argv);
#endif
#if HIL_MODE == HIL_MODE_DISABLED
static int8_t   test_sonar(uint8_t argc,                const Menu::arg *argv);
#endif

// Creates a constant array of structs representing menu options
// and stores them in Flash memory, not RAM.
// User enters the string in the console to call the functions on the right.
// See class Menu in AP_Coommon for implementation details
const struct Menu::command test_menu_commands[] PROGMEM = {
#if HIL_MODE == HIL_MODE_DISABLED
    {"baro",                test_baro},
#endif
    {"pwm",                 test_radio_pwm},
    {"radio",               test_radio},
    {"compass",             test_compass},
    {"ins",                 test_ins},
    {"optflow",             test_optflow},
    {"relay",               test_relay},
    {"input",               test_input},
    {"motor",               test_motor},
    {"motor2",              test_motor2},
    {"motor3",              test_motor3},
    {"gimbal",              test_gimbal},
    {"gimbal2",              test_gimbal2},
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
    {"shell", 				test_shell},
#endif
#if HIL_MODE == HIL_MODE_DISABLED
    {"rangefinder",         test_sonar},
#endif
};

// A Macro to create the Menu
MENU(test_menu, "test", test_menu_commands);

static int8_t
test_mode(uint8_t argc, const Menu::arg *argv)
{
    test_menu.run();
    return 0;
}

#if HIL_MODE == HIL_MODE_DISABLED
static int8_t
test_baro(uint8_t argc, const Menu::arg *argv)
{
    int32_t alt;
    print_hit_enter();
    init_barometer(true);

    while(1) {
        delay(100);
        read_barometer();

        if (!barometer.healthy()) {
            cliSerial->println_P(PSTR("not healthy"));
        } else {
            cliSerial->printf_P(PSTR("Alt: %0.2fm, Raw: %f Temperature: %.1f\n"),
                                baro_alt / 100.0,
                                barometer.get_pressure(), 
                                barometer.get_temperature());
        }
        if(cliSerial->available() > 0) {
            return (0);
        }
    }
    return 0;
}
#endif

static int8_t
test_radio_pwm(uint8_t argc, const Menu::arg *argv)
{
    print_hit_enter();
    delay(1000);

    while(1) {
        delay(20);

        // Filters radio input - adjust filters in the radio.pde file
        // ----------------------------------------------------------
        read_radio();

        // servo Yaw
        //APM_RC.OutputCh(CH_7, g.rc_4.radio_out);

        cliSerial->printf_P(PSTR("IN: 1: %d\t2: %d\t3: %d\t4: %d\t5: %d\t6: %d\t7: %d\t8: %d\n"),
                        g.rc_1.radio_in,
                        g.rc_2.radio_in,
                        g.rc_3.radio_in,
                        g.rc_4.radio_in,
                        g.rc_5.radio_in,
                        g.rc_6.radio_in,
                        g.rc_7.radio_in,
                        g.rc_8.radio_in);

        if(cliSerial->available() > 0) {
            return (0);
        }
    }
}

static int8_t
test_radio(uint8_t argc, const Menu::arg *argv)
{
    print_hit_enter();
    delay(1000);
    int16_t throttle_limited = 0;
    
    while(1) {
        delay(20);
        read_radio();
        int16_t pilot_throttle_scaled = get_pilot_desired_throttle(g.rc_3.control_in);
        motors.set_throttle(pilot_throttle_scaled);
        
        //
        
        /*
        motors.output_armed();
        _rc_roll.servo_out = out_best_thr_pwm;
        _rc_pitch.servo_out = _min_throttle;
        _rc_throttle.servo_out = thr_adj_min;
        _rc_yaw.servo_out = out_min_pwm;
        
        IN  0  1028  127   0  1140  1013
        IN 18  1030  127  18  1140  1030

        */
        
        /*
        cliSerial->printf_P(PSTR("IN %d  %d  %d  %d  %d  %d\n"),
                        g.rc_3.control_in,
                        
                        g.rc_1.servo_out, // out_best_thr_pwm
                        g.rc_2.servo_out, // _min_throttle
                        g.rc_4.servo_out, // out_min_pwm
                        g.rc_3.servo_out, // output!!
                        g.rc_3.radio_out);
        */
        ///*
        g.rc_3.servo_out = pilot_throttle_scaled;
        g.rc_3.calc_pwm();
        cliSerial->printf_P(PSTR("IN  1: %d\t2: %d\t3: %d (%d)\t4: %d\t5: %d\t6: %d\t7: %d\n"),
                        g.rc_1.control_in,
                        g.rc_2.control_in,
                        g.rc_3.control_in,
                        pilot_throttle_scaled,
                        g.rc_4.control_in,
                        g.rc_5.control_in,
                        g.rc_6.control_in,
                        g.rc_7.control_in);

        //*/
        //cliSerial->printf_P(PSTR("OUT 1: %d\t2: %d\t3: %d\t4: %d\n"), (g.rc_1.servo_out / 100), (g.rc_2.servo_out / 100), g.rc_3.servo_out, (g.rc_4.servo_out / 100));

        /*cliSerial->printf_P(PSTR(	"min: %d"
         *                                               "\t in: %d"
         *                                               "\t pwm_in: %d"
         *                                               "\t sout: %d"
         *                                               "\t pwm_out %d\n"),
         *                                               g.rc_3.radio_min,
         *                                               g.rc_3.control_in,
         *                                               g.rc_3.radio_in,
         *                                               g.rc_3.servo_out,
         *                                               g.rc_3.pwm_out
         *                                               );
         */
        if(cliSerial->available() > 0) {
            return (0);
        }
    }
}


static int8_t
test_compass(uint8_t argc, const Menu::arg *argv)
{
    uint8_t delta_ms_fast_loop;
    uint8_t medium_loopCounter = 0;

    if (!g.compass_enabled) {
        cliSerial->printf_P(PSTR("Compass: "));
        print_enabled(false);
        return (0);
    }

    if (!compass.init()) {
        cliSerial->println_P(PSTR("Compass initialisation failed!"));
        return 0;
    }

    ahrs.init();
    ahrs.set_fly_forward(true);
    ahrs.set_compass(&compass);
    report_compass();

    // we need the AHRS initialised for this test
    ins.init(AP_InertialSensor::COLD_START, 
             ins_sample_rate);
    ahrs.reset();
    int16_t counter = 0;
    float heading = 0;

    print_hit_enter();

    while(1) {
        delay(20);
        if (millis() - fast_loopTimer > 19) {
            delta_ms_fast_loop      = millis() - fast_loopTimer;
            G_Dt                    = (float)delta_ms_fast_loop / 1000.f;                       // used by DCM integrator
            fast_loopTimer          = millis();

            // INS
            // ---
            ahrs.update();

            medium_loopCounter++;
            if(medium_loopCounter == 5) {
                if (compass.read()) {
                    // Calculate heading
                    const Matrix3f &m = ahrs.get_dcm_matrix();
                    heading = compass.calculate_heading(m);
                    compass.learn_offsets();
                }
                medium_loopCounter = 0;
            }

            counter++;
            if (counter>20) {
                if (compass.healthy()) {
                    const Vector3f &mag_ofs = compass.get_offsets();
                    const Vector3f &mag = compass.get_field();
                    cliSerial->printf_P(PSTR("Heading: %ld, XYZ: %.0f, %.0f, %.0f,\tXYZoff: %6.2f, %6.2f, %6.2f\n"),
                                        (wrap_360_cd(ToDeg(heading) * 100)) /100,
                                        mag.x,
                                        mag.y,
                                        mag.z,
                                        mag_ofs.x,
                                        mag_ofs.y,
                                        mag_ofs.z);
                } else {
                    cliSerial->println_P(PSTR("compass not healthy"));
                }
                counter=0;
            }
        }
        if (cliSerial->available() > 0) {
            break;
        }
    }

    // save offsets. This allows you to get sane offset values using
    // the CLI before you go flying.
    cliSerial->println_P(PSTR("saving offsets"));
    compass.save_offsets();
    return (0);
}

static int8_t
test_ins(uint8_t argc, const Menu::arg *argv)
{
    Vector3f gyro, accel;
    print_hit_enter();
    cliSerial->printf_P(PSTR("INS\n"));
    delay(1000);

    ahrs.init();
    ins.init(AP_InertialSensor::COLD_START, 
             ins_sample_rate);
    cliSerial->printf_P(PSTR("...done\n"));

    delay(50);

    while(1) {
        ins.update();
        gyro = ins.get_gyro();
        accel = ins.get_accel();

        float test = accel.length() / GRAVITY_MSS;

        cliSerial->printf_P(PSTR("a %7.4f %7.4f %7.4f g %7.4f %7.4f %7.4f t %7.4f \n"),
            accel.x, accel.y, accel.z,
            gyro.x, gyro.y, gyro.z,
            test);

        delay(40);
        if(cliSerial->available() > 0) {
            return (0);
        }
    }
}

static int8_t
test_optflow(uint8_t argc, const Menu::arg *argv)
{
#if OPTFLOW == ENABLED
    if(optflow.enabled()) {
        cliSerial->printf_P(PSTR("dev id: %d\t"),(int)optflow.device_id());
        print_hit_enter();

        while(1) {
            delay(200);
            optflow.update();
            const Vector2i& raw = optflow.raw();
            cliSerial->printf_P(PSTR("dx:%d\t dy:%d\t squal:%d\n"),
                            (int)raw.x,
                            (int)raw.y,
                            (int)optflow.quality());

            if(cliSerial->available() > 0) {
                return (0);
            }
        }
    } else {
        cliSerial->printf_P(PSTR("OptFlow: "));
        print_enabled(false);
    }
    return (0);
#else
    return (0);
#endif      // OPTFLOW == ENABLED
}


static int8_t test_input(uint8_t argc, const Menu::arg *argv)
{
    print_hit_enter();
    delay(1000);

    cliSerial->printf_P(PSTR("Input\n"));

    int16_t i;
    int16_t scaled;
    
    // make sure throttle is 1000-2000
    for (i = 1000; i <= 2000; i++){
        g.rc_3.set_pwm(i);
        scaled = get_pilot_desired_throttle(g.rc_3.control_in);
        cliSerial->printf_P(PSTR("pwm in:%d, control_in %d, scaled: %d\n"), i, g.rc_3.control_in, scaled);
    }    
    cliSerial->printf_P(PSTR("\n\nComplete\n"));
    return (0);
}


static int8_t test_motor(uint8_t argc, const Menu::arg *argv)
{
	if (argc < 3) {
		cliSerial->printf_P(PSTR("Usage: throttle 0-1000, time in milliseconds, smoothing [0,10-1000]\n"));
        return(0);
	}
    int16_t elapsed         = 0;
	
    int16_t motor_output    = argv[1].i;
    int16_t duration        = argv[2].i;
    int16_t smoothing       = argv[3].i;

    if(smoothing < 10)
        smoothing = 1000;
    
    g.rc_3.servo_out    = constrain_int16(motor_output, 0, 1000);
    duration            = constrain_int16(duration, 100, 5000);
    g.rc_3.calc_pwm();
	int16_t output = g.rc_3.radio_out;
	int16_t limited = 100;

    // arm and enable motors
    motors.armed(true);
    motors.enable();
    
    // reduce throttle to minimum
    motors.throttle_pass_through(g.rc_3.radio_min);
	cliSerial->printf_P(PSTR("smooth %d "), smoothing);

	cliSerial->printf_P(PSTR("\n\nHold on to your F'in hats... in 5 .. "));
	delay(1000);
	cliSerial->printf_P(PSTR("4 .. "));
	delay(1000);
	cliSerial->printf_P(PSTR("3 .. "));
	delay(1000);
	cliSerial->printf_P(PSTR("2 .. "));
	delay(1000);
	cliSerial->printf_P(PSTR("1\n"));
	
    g.rc_3.servo_out = limited;
    g.rc_3.calc_pwm();
    motors.throttle_pass_through(g.rc_3.radio_out);
	delay(2000);

    /*
    while(elapsed < duration) {
        elapsed += 10;
        delay(10);
        
        if(limited < (motor_output/2)){
            limited += smoothing;
        }
        
        limited = constrain_int16(limited, 0, (motor_output/2));
        g.rc_3.servo_out = limited;
        g.rc_3.calc_pwm();

        // raise throttle
        motors.throttle_pass_through(g.rc_3.radio_out);
    }
    
    elapsed = 0;
    */
    
    while(elapsed < duration) {
        elapsed += 10;
        delay(10);
        
        if(limited < motor_output){
            limited += smoothing;
        }
        
        limited = constrain_int16(limited, 0, motor_output);
        g.rc_3.servo_out = limited;
        g.rc_3.calc_pwm();

        // raise throttle
        motors.throttle_pass_through(g.rc_3.radio_out);
    }

	cliSerial->printf_P(PSTR("%d\n"), limited);

    motors.throttle_pass_through(g.rc_3.radio_min);

    cliSerial->printf_P(PSTR("\n\nComplete\n"));
    return (0);
}

AverageFilterInt16_Size8 motorFilter;

static int8_t test_motor2(uint8_t argc, const Menu::arg *argv)
{
	if (argc < 2) {
		cliSerial->printf_P(PSTR("Usage: throttle 0-1000, time in milliseconds\n"));
        return(0);
	}
    uint32_t elapsed;
    uint32_t duration       = (uint32_t)argv[2].i * 1000;
	duration                = min(duration, 30000000);
	
    int16_t motor_output    = argv[1].i;

    g.rc_3.servo_out        = constrain_int16(motor_output, 0, 1000);
    g.rc_3.calc_pwm();
    
    // arm and enable motors
    motors.armed(true);
    motors.enable();
    
    // reduce throttle to minimum
    motors.throttle_pass_through(g.rc_3.radio_min);

    // init the filter
    //for (int8_t i = 0; i++; i < 8){
      //  motorFilter.apply(g.rc_3.radio_min);
    //}


	cliSerial->printf_P(PSTR("\nduration %ld "), duration);

	cliSerial->printf_P(PSTR("\n\nHold on to your F'in hats... in 5 .. "));
	delay(1000);
	cliSerial->printf_P(PSTR("4 .. "));
	delay(1000);
	cliSerial->printf_P(PSTR("3 .. "));
	delay(1000);
	cliSerial->printf_P(PSTR("2 .. "));
	delay(1000);
	cliSerial->printf_P(PSTR("1\n"));

	// reset the time;
	elapsed = 0;

	// raise throttle to throttle_min
    while(elapsed < (2 * 1000000)) {
        elapsed += 2500;
        hal.scheduler->delay_microseconds(2500);

        // raise throttle
        motors.throttle_pass_through(motorFilter.apply(g.rc_3.radio_min+100));
    }
    cliSerial->printf_P(PSTR("warmed up\n"));

	// reset the time;
	elapsed = 0;
	
    while(elapsed < duration) {
        if(cliSerial->available() > 0) {
            return (0);
        }
        elapsed += 2500;
        hal.scheduler->delay_microseconds(2500);

        // raise throttle
        motors.throttle_pass_through(motorFilter.apply(g.rc_3.radio_out));
    }

    motors.throttle_pass_through(g.rc_3.radio_min);

    cliSerial->printf_P(PSTR("\nComplete  %d,  %d !\n"), g.rc_3.radio_out, motorFilter.apply(g.rc_3.radio_out));
    return (0);
}

static int8_t test_motor3(uint8_t argc, const Menu::arg *argv)
{
    uint32_t elapsed;
	    
    // arm and enable motors
    motors.armed(true);
    motors.enable();
    
    // reduce throttle to minimum
    motors.throttle_pass_through(g.rc_3.radio_min);


	cliSerial->printf_P(PSTR("\n\nHold on to your F'in hats... in 5 .. "));
	delay(1000);
	cliSerial->printf_P(PSTR("4 .. "));
	delay(1000);
	cliSerial->printf_P(PSTR("3 .. "));
	delay(1000);
	cliSerial->printf_P(PSTR("2 .. "));
	delay(1000);
	cliSerial->printf_P(PSTR("1\n"));

	// reset the time;
	elapsed = 0;

	// raise throttle to throttle_min
    while(elapsed < (2 * 1000000)) {
        elapsed += 2500;
        hal.scheduler->delay_microseconds(2500);

        // raise throttle
        motors.throttle_pass_through(motorFilter.apply(g.rc_3.radio_min+100));
    }
    cliSerial->printf_P(PSTR("warmed up\n"));

	// reset the time;
	elapsed = 0;
	

	// raise throttle to throttle_min
    while(elapsed < (1 * 1000000)) {
        elapsed += 2500;
        hal.scheduler->delay_microseconds(2500);

        // raise throttle
        hal.rcout->write(0, 1725); // front right
        hal.rcout->write(1, 1725); // back left
        hal.rcout->write(2, 1600); // front left
        hal.rcout->write(3, 1100); // back right
        
    }
    //cliSerial->printf_P(PSTR("yaw up\n"));

	// reset the time;
	elapsed = 0;
	
	// raise throttle to throttle_min
    while(elapsed < (1 * 1000000)) {
        elapsed += 2500;
        hal.scheduler->delay_microseconds(2500);

        // raise throttle
        hal.rcout->write(0, 1100); // front right
        hal.rcout->write(1, 1100); // back left
        hal.rcout->write(2, 1100); // front left
        hal.rcout->write(3, 1600); // back right   
    }
    //cliSerial->printf_P(PSTR("warmed up\n"));

	// reset the time;
	elapsed = 0;
	
	
    motors.throttle_pass_through(g.rc_3.radio_min);

    cliSerial->printf_P(PSTR("\nComplete\n"));
    return (0);
}


static int8_t test_gimbal(uint8_t argc, const Menu::arg *argv)
{
 
    int16_t ch_out = argv[1].i;
    float angle_out = argv[2].f;
       
    //RC_Channel_aux::move_servo(RC_Channel_aux::k_gimbal_tilt, angle_out, 0, 9000);
    //RC_Channel_aux::set_servo_out(RC_Channel_aux::k_gimbal_tilt, angle_out);
    
    // val 13 = Aux 6
    // val 12 = Aux 5
    // val 11 = Aux 4
    // val 10 = Aux 3
    // val 9 = Aux 2
    // val 8 = Aux 1
    
    angle_out = constrain_float(angle_out, 0, 9000);
    angle_out = 1520 - (angle_out * 520)/9000;

    hal.rcout->enable_ch(ch_out);
    hal.rcout->write(ch_out, (int16_t)angle_out); //
 
    cliSerial->printf_P(PSTR("\nGimbal!, %d\n"), angle_out);
    return (0);
}

static int8_t test_gimbal2(uint8_t argc, const Menu::arg *argv)
{

    print_hit_enter();
    hal.rcout->enable_ch(8);

    delay(1000);

    while(1) {
        delay(20);
        read_radio();
        if(cliSerial->available() > 0) {
            return (0);
        }
        gimbal_run(g.rc_2.control_in);

    }
}



static int8_t test_relay(uint8_t argc, const Menu::arg *argv)
{
    print_hit_enter();
    delay(1000);

    while(1) {
        cliSerial->printf_P(PSTR("Relay on\n"));
        relay.on(0);
        delay(3000);
        if(cliSerial->available() > 0) {
            return (0);
        }

        cliSerial->printf_P(PSTR("Relay off\n"));
        relay.off(0);
        delay(3000);
        if(cliSerial->available() > 0) {
            return (0);
        }
    }
}


#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
/*
 *  run a debug shell
 */
static int8_t
test_shell(uint8_t argc, const Menu::arg *argv)
{
    hal.util->run_debug_shell(cliSerial);
    return 0;
}
#endif

#if HIL_MODE == HIL_MODE_DISABLED
/*
 *  test the rangefinders
 */
static int8_t
test_sonar(uint8_t argc, const Menu::arg *argv)
{
#if CONFIG_SONAR == ENABLED
	sonar.init();

    cliSerial->printf_P(PSTR("RangeFinder: %d devices detected\n"), sonar.num_sensors());

    print_hit_enter();
    while(1) {
        delay(100);
        sonar.update();

        cliSerial->printf_P(PSTR("Primary: health %d distance_cm %d \n"), (int)sonar.healthy(), sonar.distance_cm());
        cliSerial->printf_P(PSTR("All: device_0 type %d health %d distance_cm %d, device_1 type %d health %d distance_cm %d\n"), 
        (int)sonar._type[0], (int)sonar.healthy(0), sonar.distance_cm(0), (int)sonar._type[1], (int)sonar.healthy(1), sonar.distance_cm(1));

        if(cliSerial->available() > 0) {
            return (0);
        }
    }
#endif
    return (0);
}
#endif

static void print_hit_enter()
{
    cliSerial->printf_P(PSTR("Hit Enter to exit.\n\n"));
}

#endif // CLI_ENABLED
