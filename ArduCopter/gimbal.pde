#if GIMBAL == ENABLED

static void
update_gimbal_control()
{
    if (manual_flight_mode(control_mode)) {
        // we're in manual
        if(g.radio_tuning == 0){
            int16_t angle_out = g.rc_6.control_in * 9 ;
            angle_out = constrain_int16(angle_out, 20, 9000);
            
            RC_Channel_aux::move_servo(RC_Channel_aux::k_gimbal_tilt, angle_out, 0, 9000);
            //RC_Channel_aux::set_servo_out(RC_Channel_aux::k_gimbal_tilt, angle_out);            
        }else{
            // do nothing
        }
        
    }else{
        // we're in an AP mode
        if(auto_yaw_mode == AUTO_YAW_ROI){
            output_gimbal_pwm();
        }
    }
}

static void
output_gimbal_pwm()
{
	Vector3f position = inertial_nav.get_position();

    float deltaX = position.x - roi_WP.x;
    float deltaY = position.y - roi_WP.y;
    float deltaZ = position.z - roi_WP.z;
    
	float wp_distance   = safe_sqrt(deltaX * deltaX + deltaY * deltaY);
	float angle_out     = fast_atan2(wp_distance, deltaZ);
	
	angle_out = constrain_float(angle_out, .01, 1.571); // 0 to 90
    RC_Channel_aux::move_servo(RC_Channel_aux::k_gimbal_tilt, RadiansToCentiDegrees(angle_out), 0, 9000);
    //RC_Channel_aux::set_servo_out(RC_Channel_aux::k_gimbal_tilt, RadiansToCentiDegrees(angle_out));
}

#endif