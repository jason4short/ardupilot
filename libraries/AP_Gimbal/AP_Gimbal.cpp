// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_Param.h>
#include <AP_Gimbal.h>

// Just so that it's completely clear...
#define ENABLED                 1
#define DISABLED                0

#if defined( __AVR_ATmega1280__ )
#else
#endif

const AP_Param::GroupInfo AP_Gimbal::var_info[] PROGMEM = {

    // @Param: RC_IN_TILT
    // @DisplayName: tilt (pitch) RC input channel
    // @Description: 0 for none, any other for the RC channel to be used to control tilt (pitch) movements
    // @Values: 0:Disabled,5:RC5,6:RC6,7:RC7,8:RC8
    // @User: Standard
    AP_GROUPINFO("RC_IN_TILT",  10, AP_Gimbal, _tilt_rc_in,    0),

    // @Param: ANGMIN_TIL
    // @DisplayName: Minimum tilt angle
    // @Description: Minimum physical tilt (pitch) angular position of mount.
    // @Units: Centi-Degrees
    // @Range: -18000 17999
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("ANGMIN_TIL", 11, AP_Gimbal, _tilt_angle_min, -4500),

    // @Param: ANGMAX_TIL
    // @DisplayName: Maximum tilt angle
    // @Description: Maximum physical tilt (pitch) angular position of the mount
    // @Units: Centi-Degrees
    // @Range: -18000 17999
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("ANGMAX_TIL", 12, AP_Gimbal, _tilt_angle_max, 4500),

    AP_GROUPEND
};

AP_Gimbal::AP_Gimbal(const struct Location *current_loc, const AP_AHRS &ahrs, uint8_t id) :
    _ahrs(ahrs)
{
	AP_Param::setup_object_defaults(this, var_info);
    _mount_mode = MAV_MOUNT_MODE_RC_TARGETING;
    _tilt_idx = RC_Channel_aux::k_mount_tilt;
}


/// This one should be called periodically
void AP_Gimbal::update_mount_position()
{
    switch((enum MAV_MOUNT_MODE)_mount_mode) {
        // point to the angles given by a mavlink message
        //case MAV_MOUNT_MODE_MAVLINK_TARGETING:
        //    break;

        // RC radio manual angle control
        case MAV_MOUNT_MODE_RC_TARGETING:
        {
            #define rc_ch(i) RC_Channel::rc_channel(i-1)
            // allow pilot position input to come directly from an RC_Channel
            //if (_tilt_rc_in && (rc_ch(_tilt_rc_in))) {
            //    _tilt_control_angle = angle_input_rad(rc_ch(_tilt_rc_in), _tilt_angle_min, _tilt_angle_max);
            //}
            break;
        }

        // point mount to a GPS point given by the mission planner
        case MAV_MOUNT_MODE_GPS_POINT:
        {
            if(_ahrs.get_gps().status() >= AP_GPS::GPS_OK_FIX_2D) {
                //calc_GPS_target_angle(&_target_GPS_location);
                // XXX switch to ROI
            }
            break;
        }

        default:
            //do nothing
            break;
    }

    // write the results to the servos
    move_servo(_tilt_idx, _tilt_angle*10, _tilt_angle_min*0.1f, _tilt_angle_max*0.1f);
}

void AP_Gimbal::set_mode(enum MAV_MOUNT_MODE mode)
{
    _mount_mode = (int8_t)mode;
    // right now only accepting 2 modes, GPS tracking and RC control
    if(_mount_mode != MAV_MOUNT_MODE_GPS_POINT){
        _mount_mode = MAV_MOUNT_MODE_RC_TARGETING;
    }
}

/// Change the configuration of the mount
/// triggered by a MavLink packet.
void AP_Gimbal::configure_msg(mavlink_message_t* msg)
{}

/// Control the mount (depends on the previously set mount configuration)
/// triggered by a MavLink packet.
void AP_Gimbal::control_msg(mavlink_message_t *msg)
{}

/// Return mount status information (depends on the previously set mount configuration)
/// triggered by a MavLink packet.
void AP_Gimbal::status_msg(mavlink_message_t *msg, mavlink_channel_t chan)
{}

/// Set mount point/region of interest, triggered by mission script commands
void AP_Gimbal::set_roi_cmd(Vector3f _wp_roi)
{
    // set the target gps location
    wp_roi = _wp_roi;

    // set the mode to GPS tracking mode
    set_mode(MAV_MOUNT_MODE_GPS_POINT);
}

/// Set mount configuration, triggered by mission script commands
void AP_Gimbal::configure_cmd()
{
    // TODO get the information out of the mission command and use it
}

/// Control the mount (depends on the previously set mount configuration), triggered by mission script commands
void AP_Gimbal::control_cmd()
{
    // TODO get the information out of the mission command and use it
}

/// returns the angle (degrees*100) that the RC_Channel input is receiving
int32_t
AP_Gimbal::angle_input(RC_Channel* rc, int16_t angle_min, int16_t angle_max)
{
    return (rc->get_reverse() ? -1 : 1) * (rc->radio_in - rc->radio_min) * (int32_t)(angle_max - angle_min) / (rc->radio_max - rc->radio_min) + (rc->get_reverse() ? angle_max : angle_min);
}

/// returns the angle (radians) that the RC_Channel input is receiving
float
AP_Gimbal::angle_input_rad(RC_Channel* rc, int16_t angle_min, int16_t angle_max)
{
    return radians(angle_input(rc, angle_min, angle_max)*0.01f);
}

void
AP_Gimbal::calc_GPS_target_angle(const struct Location *target)
{
    /*
    float GPS_vector_x = (target->lng-_current_loc->lng)*cosf(ToRad((_current_loc->lat+target->lat)*0.00000005f))*0.01113195f;
    float GPS_vector_y = (target->lat-_current_loc->lat)*0.01113195f;
    float GPS_vector_z = (target->alt-_current_loc->alt);                 // baro altitude(IN CM) should be adjusted to known home elevation before take off (Set altimeter).
    float target_distance = 100.0f*pythagorous2(GPS_vector_x, GPS_vector_y);      // Careful , centimeters here locally. Baro/alt is in cm, lat/lon is in meters.

    _tilt_control_angle  = atan2f(GPS_vector_z, target_distance);
    */
    
    
}

/*
 *  /// For testing and development. Called in the medium loop.
 *  void
 *  AP_Gimbal::debug_output()
 *  {   Serial3.print("current   -     ");
 *       Serial3.print("lat ");
 *       Serial3.print(_current_loc->lat);
 *       Serial3.print(",lon ");
 *       Serial3.print(_current_loc->lng);
 *       Serial3.print(",alt ");
 *       Serial3.println(_current_loc->alt);
 *
 *       Serial3.print("gps       -     ");
 *       Serial3.print("lat ");
 *       Serial3.print(_gps->latitude);
 *       Serial3.print(",lon ");
 *       Serial3.print(_gps->longitude);
 *       Serial3.print(",alt ");
 *       Serial3.print(_gps->altitude);
 *       Serial3.println();
 *
 *       Serial3.print("target   -      ");
 *       Serial3.print("lat ");
 *       Serial3.print(_target_GPS_location.lat);
 *       Serial3.print(",lon ");
 *       Serial3.print(_target_GPS_location.lng);
 *       Serial3.print(",alt ");
 *       Serial3.print(_target_GPS_location.alt);
 *       Serial3.print(" hdg to targ ");
 *       Serial3.print(degrees(_pan_control_angle));
 *       Serial3.println();
 *  }
 */

/// all angles are degrees * 10 units
void
AP_Gimbal::move_servo(uint8_t function_idx, int16_t angle, int16_t angle_min, int16_t angle_max)
{
	// saturate to the closest angle limit if outside of [min max] angle interval
	//int16_t servo_out = closest_limit(angle, &angle_min, &angle_max);
	int16_t servo_out = 0;
	RC_Channel_aux::move_servo((RC_Channel_aux::Aux_servo_function_t)function_idx, servo_out, angle_min, angle_max);
}
