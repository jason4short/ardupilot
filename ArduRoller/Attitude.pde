/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

static bool
check_attitude()
{
    uint32_t _time = millis();

    if(labs(ahrs.pitch_sensor) > 4000){
    	//remember time of pitchover
        balance_timer = _time;
        if(ap.armed){
            init_disarm_motors();
            reset_I_all();
            current_speed       = 0;
            nav_yaw             = ahrs.yaw_sensor;
            current_encoder_x   = 0;
            current_encoder_y   = 0;
            pitch_out_right     = 0;
            pitch_out_left      = 0;
            yaw_out             = 0;
            encoder_nav.set_current_position(g_gps->longitude, g_gps->latitude);
            set_destination(encoder_nav.get_position());
        }
        return false;
    }

	//no output until we have been upright for 2 seconds
    if((_time - balance_timer) < 2000){
        pitch_out_right = 0;
        pitch_out_left  = 0;
        yaw_out         = 0;
        // reset nav_yaw to be whatever
        nav_yaw = ahrs.yaw_sensor;
        return false;

    }else{
	    if(!ap.armed){
	        set_destination(encoder_nav.get_position());
	        init_arm_motors();
	    }
    }
    // we're OK to run motors
    return true;
}

static int16_t
get_stabilize_pitch(int32_t target_angle)
{
    int32_t angle_error = wrap_180_cd(target_angle - (ahrs.pitch_sensor + balance_offset));  //balance_offset

    // dynamically adjust the CG when we are supposed to be at vertical
    if(target_angle == 0){
        balance_offset = g.pid_balance.get_i(angle_error, G_Dt);
    }
    
    int32_t rate_error  = 0 - (omega.y * DEGX100);

    int16_t bal_P = g.pid_balance.kP() * (float)angle_error;
    int16_t bal_D = g.pid_balance.kD() * (float)rate_error;

    //cliSerial->printf_P(PSTR("a:%d, P:%d, D:%d\n"), (int16_t)ahrs.pitch_sensor, bal_P, bal_D);
   // cliSerial->printf_P(PSTR("a:%d\te:%d\n"), (int16_t)ahrs.pitch_sensor, angle_error);

    return bal_P + bal_D;
}

static int16_t
get_velocity_pitch()
{
    //cliSerial->printf_P(PSTR("ws:%d, se:%d\n"), wheel.speed, speed_error);
    return g.p_vel * wheel.speed; /// 1.0 * 1000;
}

static int16_t
get_stabilize_yaw(int32_t target_angle)
{
    int32_t angle_error;

    // angle error
    angle_error         = wrap_180_cd(target_angle - ahrs.yaw_sensor);

    // limit the error we're feeding to the PID
    angle_error         = constrain(angle_error, -1500, 1500);
    int16_t output      = (float)g.pid_yaw.get_pid(angle_error, G_Dt) / wheel_ratio;
    return output;
}



/*************************************************************
 * yaw controllers
 *************************************************************/

 // get_look_at_yaw - updates bearing to look at center of circle or do a panorama
// should be called at 100hz
static void get_circle_yaw()
{
    static uint8_t look_at_yaw_counter = 0;     // used to reduce update rate to 10hz

    // if circle radius is zero do panorama
    if( g.circle_radius == 0 ) {
        // slew yaw towards circle angle
        nav_yaw = get_yaw_slew(nav_yaw, ToDeg(circle_angle)*100, AUTO_YAW_SLEW_RATE);
    }else{
        look_at_yaw_counter++;
        if( look_at_yaw_counter >= 10 ) {
            look_at_yaw_counter = 0;
            yaw_look_at_WP_bearing = pv_get_bearing_cd(encoder_nav.get_position(), yaw_look_at_WP);
        }
        // slew yaw
        nav_yaw = get_yaw_slew(nav_yaw, yaw_look_at_WP_bearing, AUTO_YAW_SLEW_RATE);
    }

    // call stabilize yaw controller
    get_stabilize_yaw(nav_yaw);
}



/*
 *  reset all I integrators
 */
static void reset_I_all(void)
{
    balance_offset = 0;
    g.pid_balance.reset_I();
    g.pid_yaw.reset_I();
    reset_nav_I();
}

static void reset_nav_I()
{
    g.pid_nav_right.reset_I();
    g.pid_nav_left.reset_I();
}

