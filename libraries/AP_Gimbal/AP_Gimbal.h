// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/************************************************************
* AP_Gimbal -- library to control a 2 or 3 axis mount.		*
*															*
* Author:  Jason Short                                      *
*          Joe Holdsworth;									*
*		   Ritchie Wilson;									*
*		   Amilcar Lucas;									*
*		   Gregory Fletcher;								*
*															*
* Purpose:  Move a 2 or 3 axis mount attached to vehicle,	*
*			Used for mount to track targets or stabilise	*
*			camera plus	other modes.						*
*															*
* Usage:	Use in main code to control	mounts attached to	*
*			vehicle.										*
*															*
* Comments: All angles in degrees * 100, distances in meters*
*			unless otherwise stated.						*
************************************************************/
#ifndef __AP_Gimbal_H__
#define __AP_Gimbal_H__

#include <AP_Math.h>
#include <AP_Common.h>
#include <AP_GPS.h>
#include <AP_AHRS.h>
#include <GCS_MAVLink.h>
#include <RC_Channel.h>

class AP_Gimbal
{
public:
    //Constructor
    AP_Gimbal(const struct Location *current_loc, const AP_AHRS &ahrs, uint8_t id);

    // get_mode - return current mount mode
    enum MAV_MOUNT_MODE             get_mode() const { return (enum MAV_MOUNT_MODE)_mount_mode; }

    // set_mode_to_default - restores the mode to it's default held in the MNT_MODE parameter
    //      this operation requires 2ms on an APM2, 0.7ms on a Pixhawk/PX4
    void                            set_mode_to_default() { _mount_mode; }

    // MAVLink methods
    void                            configure_msg(mavlink_message_t* msg);
    void                            control_msg(mavlink_message_t* msg);
    void                            status_msg(mavlink_message_t* msg, mavlink_channel_t chan);
    void                            set_roi_cmd(Vector3f _wp_roi);
    void                            configure_cmd();
    void                            control_cmd();

    // should be called periodically
    void                            update_mount_position();
    void                            debug_output();      ///< For testing and development. Called in the medium loop.

    // hook for eeprom variables
    static const struct AP_Param::GroupInfo        var_info[];

    void                            set_mode(enum MAV_MOUNT_MODE mode);

private:

    //methods

    // internal methods
    void                            calc_GPS_target_angle(const struct Location *target);
    void                            move_servo(uint8_t rc, int16_t angle, int16_t angle_min, int16_t angle_max);
    int32_t                         angle_input(RC_Channel* rc, int16_t angle_min, int16_t angle_max);
    float                           angle_input_rad(RC_Channel* rc, int16_t angle_min, int16_t angle_max);

    //members
    const AP_AHRS                   &_ahrs; ///< Rotation matrix from earth to plane.
    Vector3f                        wp_roi; ///< GCS controlled position for mount, vector.x = roll vector.y = tilt, vector.z=pan

    uint8_t                         _tilt_idx; ///< RC_Channel_aux mount tilt function index
    float                           _tilt_control_angle; ///< radians
    float                           _tilt_angle; ///< degrees

    // EEPROM parameters
    AP_Int8                         _stab_roll; ///< (1 = yes, 0 = no)
    AP_Int8                         _stab_tilt; ///< (1 = yes, 0 = no)
    AP_Int8                         _stab_pan;  ///< (1 = yes, 0 = no)

    int8_t                          _mount_mode;
    // RC_Channel for providing direct angular input from pilot
    AP_Int8                         _tilt_rc_in;

    AP_Int16                        _roll_angle_min; ///< min angle limit of actuated surface in 0.01 degree units
    AP_Int16                        _roll_angle_max; ///< max angle limit of actuated surface in 0.01 degree units
    AP_Int16                        _tilt_angle_min; ///< min angle limit of actuated surface in 0.01 degree units
    AP_Int16                        _tilt_angle_max; ///< max angle limit of actuated surface in 0.01 degree units
    AP_Int16                        _pan_angle_min;  ///< min angle limit of actuated surface in 0.01 degree units
    AP_Int16                        _pan_angle_max;  ///< max angle limit of actuated surface in 0.01 degree units

    AP_Int8                         _joystick_speed;

    AP_Vector3f                     _control_angles; ///< GCS controlled position for mount, vector.x = roll vector.y = tilt, vector.z=pan
};

#endif // __AP_Gimbal_H__
