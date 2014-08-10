// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/************************************************************
* AC_Gimbal -- library to control a 2 or 3 axis mount.		*
*															*
* Author:   Jason Short                                     *
*           based on AP_mount                               *
*															*
* Purpose:  Move a brushless gimbal attached to vehicle 	*
*															*
* Usage:	Use in main code to control	mounts attached to	*
*			vehicle.										*
*															*
************************************************************/
#ifndef __AC_Gimbal_H__
#define __AC_Gimbal_H__

#include <AP_Math.h>
#include <AP_Common.h>
#include <AP_GPS.h>
#include <AP_InertialNav.h>
#include <AP_AHRS.h>
#include <GCS_MAVLink.h>
#include <RC_Channel.h>

class AC_Gimbal
{
public:
    //Constructor
    AC_Gimbal(const AP_InertialNav& inav, const AP_AHRS &ahrs, uint8_t id);

    // get_mode - return current mount mode
    enum MAV_MOUNT_MODE             get_mode() const { return (enum MAV_MOUNT_MODE)_mount_mode; }

    // set_mode_to_default - restores the mode to it's default held in the MNT_MODE parameter
    //      this operation requires 2ms on an APM2, 0.7ms on a Pixhawk/PX4
    void                            set_mode_to_default() { _mount_mode = MAV_MOUNT_MODE_RC_TARGETING; }

    // MAVLink methods
    void                            configure_msg(mavlink_message_t* msg);
    void                            control_msg(mavlink_message_t* msg);
    void                            status_msg(mavlink_message_t* msg, mavlink_channel_t chan);
    
    void                            set_ROI(Vector3f roi_WP);
    Vector3f                        get_ROI_from_gimbal();
    
    void                            configure_cmd();
    void                            control_cmd();

    // should be called periodically
    void                            update_gimbal();
    void                            debug_output();      ///< For testing and development. Called in the medium loop.

    // hook for eeprom variables
    static const struct AP_Param::GroupInfo        var_info[];

    void                            set_mode(enum MAV_MOUNT_MODE mode);

// move to private after testing
    void                            calc_gimbal_ROI();
    float                           _tilt_angle; ///< radians

private:

    // pointers to other objects we depend upon
    const AP_InertialNav&           _inav;

    // internal methods 
    // change to float
    int32_t                         angle_input(RC_Channel* rc, int16_t angle_min, int16_t angle_max);

    //members
    const AP_AHRS                   &_ahrs; ///< Rotation matrix from earth to plane.
    Vector3f                        _roi_WP; ///< GCS controlled position for mount, vector.x = roll vector.y = tilt, vector.z=pan

    uint8_t                         _tilt_idx; ///< RC_Channel_aux mount tilt function index

    int8_t                          _mount_mode;
    // RC_Channel for providing direct angular input from pilot
    AP_Int8                         _tilt_rc_in;

    AP_Int16                        _tilt_angle_min; ///< min angle limit of actuated surface in 0.01 degree units
    AP_Int16                        _tilt_angle_max; ///< max angle limit of actuated surface in 0.01 degree units
};

#endif // __AC_Gimbal_H__
