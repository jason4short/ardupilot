// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_Param.h>
#include <AC_Gimbal.h>

// uncomment for debuging
//#include <AP_HAL.h>
//extern const AP_HAL::HAL& hal;

// Just so that it's completely clear...
#define ENABLED                 1
#define DISABLED                0

#if defined( __AVR_ATmega1280__ )
#else
#endif

#define rc_ch(i) RC_Channel::rc_channel(i-1)

const AP_Param::GroupInfo AC_Gimbal::var_info[] PROGMEM = {

    // @Param: RC_IN_TILT
    // @DisplayName: tilt (pitch) RC input channel
    // @Description: 0 for none, any other for the RC channel to be used to control tilt (pitch) movements
    // @Values: 0:Disabled,5:RC5,6:RC6,7:RC7,8:RC8
    // @User: Standard
    AP_GROUPINFO("RC_IN_TILT",  10, AC_Gimbal, _tilt_rc_in,    0),

    // @Param: ANGMIN_TIL
    // @DisplayName: Minimum tilt angle
    // @Description: Minimum physical tilt (pitch) angular position of mount.
    // @Units: Centi-Degrees
    // @Range: -18000 17999
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("ANGMIN_TIL", 11, AC_Gimbal, _tilt_angle_min, -4500),

    // @Param: ANGMAX_TIL
    // @DisplayName: Maximum tilt angle
    // @Description: Maximum physical tilt (pitch) angular position of the mount
    // @Units: Centi-Degrees
    // @Range: -18000 17999
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("ANGMAX_TIL", 12, AC_Gimbal, _tilt_angle_max, 4500),

    AP_GROUPEND
};

AC_Gimbal::AC_Gimbal(const AP_InertialNav& inav, const AP_AHRS &ahrs, uint8_t id) :
                _inav(inav),
                _ahrs(ahrs),
                _mount_mode(MAV_MOUNT_MODE_RC_TARGETING)
{
	AP_Param::setup_object_defaults(this, var_info);
}

/// This one should be called periodically
void AC_Gimbal::update_gimbal()
{
    //hal.console->printf_P(PSTR("\n update_gimbal: _mount_mode:%d\n"), _mount_mode);
    switch((enum MAV_MOUNT_MODE)_mount_mode) {

        // RC radio manual angle control
        case MAV_MOUNT_MODE_RC_TARGETING:
        {            
            //hal.console->printf_P(PSTR("\n MAV_MOUNT_MODE_RC_TARGETING\n"));
            // allow pilot position input to come directly from an RC_Channel
            if (_tilt_rc_in && (rc_ch(_tilt_rc_in))) {
                _tilt_angle = angle_input(rc_ch(_tilt_rc_in), _tilt_angle_min, _tilt_angle_max);
                //_tilt_angle = angle_input(rc_ch(_tilt_rc_in), -9000, 0); // for testing
                
            }else{
                _tilt_angle = 8000;
            }
            break;
        }

        // point mount to a GPS point given by the mission planner
        case MAV_MOUNT_MODE_GPS_POINT:
        {
            //hal.console->printf_P(PSTR("\n MAV_MOUNT_MODE_GPS_POINT\n"));
            if(_ahrs.get_gps().status() >= AP_GPS::GPS_OK_FIX_2D) {
                calc_gimbal_ROI();
            }
            break;
        }

        default:
            //do nothing
            break;
    }

    // output servo commands
    RC_Channel_aux::move_servo(RC_Channel_aux::k_mount_tilt, _tilt_angle, _tilt_angle_min, _tilt_angle_max);
}

void AC_Gimbal::set_mode(enum MAV_MOUNT_MODE mode)
{
    _mount_mode = (int8_t)mode;
    // right now only accepting 2 modes, GPS tracking and RC control
    if(_mount_mode != MAV_MOUNT_MODE_GPS_POINT){
        _mount_mode = MAV_MOUNT_MODE_RC_TARGETING;
    }
}

/// Change the configuration of the mount
/// triggered by a MavLink packet.
void AC_Gimbal::configure_msg(mavlink_message_t* msg)
{}

/// Control the mount (depends on the previously set mount configuration)
/// triggered by a MavLink packet.
void AC_Gimbal::control_msg(mavlink_message_t *msg)
{}

/// Return mount status information (depends on the previously set mount configuration)
/// triggered by a MavLink packet.
void AC_Gimbal::status_msg(mavlink_message_t *msg, mavlink_channel_t chan)
{}


/// Set mount configuration, triggered by mission script commands
void AC_Gimbal::configure_cmd()
{
}


/// Control the mount (depends on the previously set mount configuration), triggered by mission script commands
void AC_Gimbal::control_cmd()
{
}


/// returns the angle (degrees*100) that the RC_Channel input is receiving
int32_t
AC_Gimbal::angle_input(RC_Channel* rc, int16_t angle_min, int16_t angle_max)
{
    float temp;
    temp = (float)(rc->radio_in - rc->radio_min) / (float)(rc->radio_max - rc->radio_min) ;    
    temp *= (float)(angle_max - angle_min);
    if(rc->get_reverse()) temp = -temp;
    temp += (rc->get_reverse() ? angle_max : angle_min);
    return temp;
}


/// Set mount point/region of interest, triggered by mission script commands
void AC_Gimbal::set_ROI(Vector3f roi_WP)
{
    // set the target gps location
    _roi_WP = roi_WP;
    //hal.console->printf_P(PSTR("\nset_ROI %1.0f, %1.0f, %1.0f\n"), _roi_WP.x, _roi_WP.y, _roi_WP.z);

    // set the mode to GPS tracking mode
    set_mode(MAV_MOUNT_MODE_GPS_POINT);
}


// Calculates a new tilt angle based on the given ROI
void
AC_Gimbal::calc_gimbal_ROI()
{
    //hal.console->printf_P(PSTR("\ncalc_gimbal_ROI\n"));
	Vector3f position = _inav.get_position();

    float deltaX = position.x - _roi_WP.x;
    float deltaY = position.y - _roi_WP.y;
    float deltaZ = position.z - _roi_WP.z;

    //hal.console->printf_P(PSTR("\ndelta %1.0f, %1.0f, %1.0f\n"), deltaX, deltaY, deltaZ);
    
	float wp_distance   = safe_sqrt(deltaX * deltaX + deltaY * deltaY);
    #if defined( __AVR_ATmega1280__ )
	_tilt_angle         = fast_atan2(wp_distance, deltaZ); // not so accurate
    #else
	_tilt_angle         = atan2(wp_distance, deltaZ);   // way more accurate
    #endif
    //hal.console->printf_P(PSTR("\nwp_distance %1.0f, tilt_rad:%1.6f\n"), wp_distance, _tilt_angle);
	
	_tilt_angle = constrain_float(_tilt_angle, .01, 1.571); // 0 to 90
	// make it negative
	_tilt_angle = -(RadiansToCentiDegrees(_tilt_angle));
    //hal.console->printf_P(PSTR("\n_tilt_angle deg %1.2f\n"),_tilt_angle);
}


// returns a new ROI vector based on the tilt angle of the camera
// Assumes an altitude of 0
Vector3f 
AC_Gimbal::get_ROI_from_gimbal()
{
    //hal.console->printf_P(PSTR("\nget_ROI_from_gimbal\n"));
    //hal.console->printf_P(PSTR("\ncos_yaw: %1.4f, sin_yaw:%1.4f\n"), _ahrs.cos_yaw(), _ahrs.sin_yaw());

	Vector3f position   = _inav.get_position();
	
	float _gimbal_angle = constrain_float(_tilt_angle, -8000, -500);
    float _distance      = position.z / tan(_gimbal_angle * .000174533f);
    //hal.console->printf_P(PSTR("\n_gimbal_angle: %1.4f, _distance:%1.4f\n"), _gimbal_angle, _distance);
    
    // rotate _distance to world frame
    _roi_WP.x = position.x + (_ahrs.cos_yaw() * _distance);  // X = Lat N/S:
    _roi_WP.y = position.y + (_ahrs.sin_yaw() * _distance);  //Y = lon E/W
    _roi_WP.z = 0;
    
    //hal.console->printf_P(PSTR("\nget_ROI %1.0f, %1.0f, %1.0f\n"), _roi_WP.x, _roi_WP.y, _roi_WP.z);
    
    return _roi_WP;
}

