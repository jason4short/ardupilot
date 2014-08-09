#if GIMBAL == ENABLED

static void
update_gimbal_control()
{
    if(gimbal_mode == GIMBAL_TILT_ROI){
        output_gimbal_pwm();
        
    }else{
        //gimbal_mode == GIMBAL_MANUAL 
        if(g.radio_tuning == 0){
            gimbal_angle = constrain_int16((g.rc_6.control_in * 9), 20, 9000);
            RC_Channel_aux::move_servo(RC_Channel_aux::k_gimbal_tilt, gimbal_angle, 0, 9000);
            //RC_Channel_aux::set_servo_out(RC_Channel_aux::k_gimbal_tilt, angle_out);            
            //cliSerial->printf_P(PSTR("IN: 6: %d\t    PWMOut 9: %d\t RadioOut 9: %d\t   angle out: %1.1f\n"), g.rc_6.radio_in, g.rc_9.pwm_out, g.rc_9.radio_out, gimbal_angle);
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
	gimbal_angle        = fast_atan2(wp_distance, deltaZ);
	
	gimbal_angle = constrain_float(gimbal_angle, .01, 1.571); // 0 to 90
    RC_Channel_aux::move_servo(RC_Channel_aux::k_gimbal_tilt, RadiansToCentiDegrees(gimbal_angle), 0, 9000);
    //RC_Channel_aux::set_servo_out(RC_Channel_aux::k_gimbal_tilt, RadiansToCentiDegrees(gimbal_angle));
}



static void calc_roi_from_gimbal()
{
    
	Vector3f position = inertial_nav.get_position();
	float _gimbal_angle = constrain_float(gimbal_angle, 500, 8000);

    // calc distance
    float distance = (1/tan(_gimbal_angle * .000174533f)) * position.z;
    
    // rotate distance to world frame
    //Lat N/S:
    roi_WP.x = position.x + (ahrs.cos_yaw() * distance);
    //lon E/W
    roi_WP.y = position.y + (ahrs.sin_yaw() * distance);
    
    // defaulting to 2m heigh (a person's face)
    roi_WP.z = 0;
    
    //cliSerial->printf_P(PSTR("ROI: an:%1.3f, yaw:%1.2f, sin:%1.3f, cos:%1.3f, dis:%1.3f, x:%1.3f, y:%1.3f\n"), _gimbal_angle, (float)ahrs.yaw_sensor, ahrs.sin_yaw(), ahrs.cos_yaw(), distance, roi_WP.x, roi_WP.y);
}
//ROI: an:9000.000, yaw:35932.00, sin:-0.011, cos:0.999, dis:-1.281, x:0.015, y:-1.281
.000174533f

#endif