/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#ifndef AC_WPNAV_H
#define AC_WPNAV_H

#include <inttypes.h>
#include <AP_Common.h>
#include <AP_Param.h>
#include <AP_Math.h>
#include <AC_PID.h>             // PID library
#include <APM_PI.h>             // PID library
#include <AR_InertialNav.h>     // Inertial Navigation library

// loiter maximum velocities and accelerations
#define MAX_LOITER_POS_VELOCITY         500         // maximum velocity that our position controller will request.  should be 1.5 ~ 2.0 times the pilot input's max velocity.  To-Do: make consistent with maximum velocity requested by pilot input to loiter
#define MAX_LOITER_POS_ACCEL            500         // defines the velocity vs distant curve.  maximum acceleration in cm/s/s that loiter position controller asks for from acceleration controller
#define MAX_LOITER_VEL_ACCEL            800         // max acceleration in cm/s/s that the loiter velocity controller will ask from the lower accel controller.
                                                    // should be 1.5 times larger than MAX_LOITER_POS_ACCEL.
                                                    // max acceleration = max lean angle * 980 * pi / 180.  i.e. 23deg * 980 * 3.141 / 180 = 393 cm/s/s

#define MAX_LEAN_ANGLE                  4500        // default maximum lean angle

#define MAX_LOITER_OVERSHOOT            531        // maximum distance (in cm) that we will allow the target loiter point to be from the current location when switching into loiter
#define WPNAV_WP_SPEED                  500.0f      // default horizontal speed betwen waypoints in cm/s
#define WPNAV_WP_RADIUS                 200.0f      // default waypoint radius in cm


class AR_WPNav
{
public:

    /// Constructor
    AR_WPNav(AR_InertialNav* inav, AC_PID* pid_wp_nav);


    ///
    ///
    ///

    /// update_loiter - run the loiter controller - should be called at 10hz
    void update_loiter();

    /// update_wp - update waypoint controller
    void update_wpnav();

    ///
    ///
    ///

    /// get_destination waypoint using position vector (distance from home in cm)
    const Vector3f &get_destination() const { return _destination; }

    /// set_destination waypoint using position vector (distance from home in cm)
    void set_destination(const Vector3f& destination);

    /// set_origin_and_destination - set origin and destination waypoints using position vectors (distance from home in cm)
    void set_origin_and_destination(const Vector3f& origin, const Vector3f& destination);

    /// get_distance_to_destination - get horizontal distance to destination in cm
    float get_distance_to_destination();

    /// get_bearing_to_destination - get bearing to next waypoint in centi-degrees
    int32_t get_bearing_to_destination();

	int32_t get_bearing_to_target() const;
	float get_distance_to_target() const;

    /// reached_destination - true when we have come within RADIUS cm of the waypoint
    bool reached_destination() const { return _reached_destination; }

    /// get desired roll, pitch which should be fed into stabilize controllers
    int32_t get_desired_roll() const { return _desired_roll; };
    int32_t get_desired_pitch() const { return _desired_pitch; };

    /// set_cos_sin_yaw - short-cut to save on calculations to convert from roll-pitch frame to lat-lon frame
    void set_cos_sin_yaw(float cos_yaw, float sin_yaw, float cos_pitch) {
        _cos_yaw = cos_yaw;
        _sin_yaw = sin_yaw;
        _cos_pitch = cos_pitch;
    }

    /// set_horizontal_velocity - allows main code to pass target horizontal velocity for wp navigation
    void set_horizontal_velocity(float velocity_cms) { _speed_xy_cms = velocity_cms; };

    /// get_waypoint_radius - access for waypoint radius in cm
    float get_waypoint_radius() const { return _wp_radius_cm; }

    static const struct AP_Param::GroupInfo var_info[];

protected:

    /// get_loiter_position_to_velocity - loiter position controller
    ///     converts desired position held in _target vector to desired velocity
    void get_position_to_velocity(float dt);

    /// get_velocity_to_acceleration - loiter velocity controller
    ///    converts desired velocities in lat/lon directions to accelerations in lat/lon frame
    void get_velocity_to_acceleration(float vel_lat_cms, float vel_lon_cms, float dt);

    /// get_acceleration_to_lean_angles - loiter acceleration controller
    ///    converts desired accelerations provided in lat/lon frame to roll/pitch angles
    void get_acceleration_to_lean_angles(float accel_lat_cmss, float accel_lon_cmss);

    /// get_bearing_cd - return bearing in centi-degrees between two positions
    float get_bearing_cd(const Vector3f &origin, const Vector3f &destination) const;


    /// reset_I - clears I terms from loiter PID controller
    void reset_I();

    // pointers to inertial nav library
    AR_InertialNav*	_inav;

    // pointers to pid controllers
    AC_PID*		_pid_wp_nav;

    // parameters
    AP_Float    _speed_xy_cms;          // horizontal speed target in cm/s
    AP_Float    _wp_radius_cm;          // distance from a waypoint in cm that, when crossed, indicates the wp has been reached
    uint32_t	_loiter_last_update;    // time of last update_loiter call
    uint32_t	_wpnav_last_update;     // time of last update_wpnav call
    float       _cos_yaw;               // short-cut to save on calcs required to convert roll-pitch frame to lat-lon frame
    float       _sin_yaw;
    float       _cos_pitch;

    // output from controller
    int32_t     _desired_roll;          // fed to stabilize controllers at 50hz
    int32_t     _desired_pitch;         // fed to stabilize controllers at 50hz

    // waypoint controller internal variables
    Vector3f    _origin;                // starting point of trip to next waypoint in cm from home (equivalent to next_WP)
    Vector3f    _destination;           // target destination in cm from home (equivalent to next_WP)
    float       _distance_to_target;    // distance to loiter target
    bool        _reached_destination;   // true if we have reached the destination

public:
    // for logging purposes
    Vector2f dist_error;                // distance error calculated by loiter controller
    Vector2f desired_vel;               // loiter controller desired velocity
    Vector2f desired_accel;             // the resulting desired acceleration
};
#endif	// AC_WPNAV_H
