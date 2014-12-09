/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
 * control_drift.pde - init and run calls for drift flight mode
 */

static bool gamer_roi;

// drift_init - initialise drift controller
static bool gamer_init(bool ignore_checks)
{
    gamer_roi = false;
    
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
    float target_yaw_rate = 0;
    float target_climb_rate = 0;
    static int16_t yaw_in = 0;
    
    if (ap.new_radio_frame) {

        // remapping
        g.rc_1.control_in = g.rc_2.control_in;
        // clear pitch input to make the rails simulation work
        g.rc_2.control_in = 0;

        // yaw control
        yaw_in  = g.rc_4.control_in / 2;

        // switch roll and yaw input
        //g.rc_1.control_in = g.rc_4.control_in;        
        
        // apply SIMPLE mode transform to pilot inputs
        update_simple_mode();

        float tilt_input = (float)(g.rc_3.radio_in - g.rc_3.radio_trim) / 500.0;

        // look for CH7 to go high to enter ROI lock mode
        //if(g.rc_7.radio_in > 1500){
        if((g.rc_4.control_in == 0) && (fabs(tilt_input) < .022)){
            if(!gamer_roi){
                gamer_roi = true;
                calc_roi_from_gimbal();
            }
        }else{
            gamer_roi = false;
        }
        
                


        // this is a big Maybe!
        // we may want to try a version where we let the user
        // maintain manual gimbal control, but Yaw is given to ROI controller.
        if(gamer_roi){
        //if(fabs(tilt_input) < .02){
            // tilt is run by the gimbal
            gimbal_run_roi();
        }else{
            // gimbal control:
            gimbal_run_manual(tilt_input * -4500.0);
        }
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
        //target_climb_rate = get_pilot_desired_climb_rate(g.rc_3.control_in);
        target_climb_rate = 0;

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
        if (gamer_roi) {
            // roll, pitch from waypoint controller, yaw heading from auto_heading()
            attitude_control.angle_ef_roll_pitch_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), get_roi_yaw(), true);
        }else{
            // roll & pitch from waypoint controller, yaw rate from pilot
            attitude_control.angle_ef_roll_pitch_rate_ef_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), target_yaw_rate);
        }


        // update altitude target and call position controller
        pos_control.set_alt_target_from_climb_rate(0, G_Dt);
        pos_control.update_z_controller();
    }
}


