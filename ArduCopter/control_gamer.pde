/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
 * control_drift.pde - init and run calls for drift flight mode
 */

// drift_init - initialise drift controller
static bool gamer_init(bool ignore_checks)
{
    if (GPS_ok() || ignore_checks) {
        hal.rcout->enable_ch(8);
       return true;
    }else{
        return false;
    }
}

// gamer_run - runs the gamer controller
// should be called at 100hz or more
static void gamer_run()
{
    float target_yaw_rate;
    float target_climb_rate = 0;
    static int16_t yaw_in;


    if (ap.new_radio_frame) {
        
        // gimbal control:
        gimbal_run(g.rc_2.control_in);
        g.rc_2.control_in = 0;

        // yaw control
        yaw_in  = g.rc_1.control_in / 2;

        // switch roll and yaw input
        g.rc_1.control_in = g.rc_4.control_in;        
        


        // apply SIMPLE mode transform to pilot inputs
        update_simple_mode();
    }
    
    
    // if not armed or landed and throttle at zero, set throttle to zero and exit immediately
    if(!motors.armed() || (ap.land_complete && g.rc_3.control_in <= 0)) {
        attitude_control.relax_bf_rate_controller();
        attitude_control.set_yaw_target_to_current_heading();
        attitude_control.set_throttle_out(0, false);
        pos_control.set_alt_target_to_current_alt();
        return;
    }
    
    // process pilot inputs
    if (!failsafe.radio) {

        // process pilot's roll and pitch input
        wp_nav.set_pilot_desired_acceleration(g.rc_1.control_in, g.rc_2.control_in);

        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(yaw_in);

        // get pilot desired climb rate
        target_climb_rate = get_pilot_desired_climb_rate(g.rc_3.control_in);

        // check for pilot requested take-off
        if (ap.land_complete && target_climb_rate > 0) {
            // indicate we are taking off
            set_land_complete(false);
            // clear i term when we're taking off
            set_throttle_takeoff();
        }
    } else {
        // clear out pilot desired acceleration in case radio failsafe event occurs and we do not switch to RTL for some reason
        wp_nav.clear_pilot_desired_acceleration();
    }


    // relax loiter target if we might be landed
    if (land_complete_maybe()) {
        wp_nav.loiter_soften_for_landing();
    }

    // when landed reset targets and output zero throttle
    if (ap.land_complete) {
        wp_nav.init_loiter_target();
        attitude_control.relax_bf_rate_controller();
        attitude_control.set_yaw_target_to_current_heading();
        // move throttle to between minimum and non-takeoff-throttle to keep us on the ground
        attitude_control.set_throttle_out(get_throttle_pre_takeoff(g.rc_3.control_in), false);
        pos_control.set_alt_target_to_current_alt();
    }else{
        // run loiter controller
        wp_nav.update_loiter();

        // call attitude controller
        attitude_control.angle_ef_roll_pitch_rate_ef_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), target_yaw_rate);


        // update altitude target and call position controller
        pos_control.set_alt_target_from_climb_rate(target_climb_rate, G_Dt);
        pos_control.update_z_controller();
    }
}

