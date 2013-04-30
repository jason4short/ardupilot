/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#include <AP_HAL.h>
#include <AR_WPNav.h>

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AR_WPNav::var_info[] PROGMEM = {
    // index 0 was used for the old orientation matrix

    // @Param: SPEED
    // @DisplayName: Waypoint Horizontal Speed Target
    // @Description: Defines the speed in cm/s which the aircraft will attempt to maintain horizontally during a WP mission
    // @Units: Centimeters/Second
    // @Range: 0 1000
    // @Increment: 50
    // @User: Standard
    AP_GROUPINFO("SPEED",   0, AR_WPNav, _speed_xy_cms, WPNAV_WP_SPEED),

    // @Param: RADIUS
    // @DisplayName: Waypoint Radius
    // @Description: Defines the distance from a waypoint, that when crossed indicates the wp has been hit.
    // @Units: Centimeters
    // @Range: 100 1000
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("RADIUS",  1, AR_WPNav, _wp_radius_cm, WPNAV_WP_RADIUS),

    AP_GROUPEND
};

// Default constructor.
// Note that the Vector/Matrix constructors already implicitly zero
// their values.
//
AR_WPNav::AR_WPNav(AR_InertialNav* inav, AC_PID* pid_wp_nav) :
    _inav(inav),
    _pid_wp_nav(pid_wp_nav)
{
    AP_Param::setup_object_defaults(this, var_info);
}

///
/// simple loiter controller
///

/// update_loiter - run the loiter controller - should be called at 10hz
void AR_WPNav::update_loiter()
{
    uint32_t now = hal.scheduler->millis();
    float dt = (now - _loiter_last_update) / 1000.0f;
    _loiter_last_update = now;

    // catch if we've just been started
    if( dt >= 1.0 ) {
        dt = 0.0;
        reset_I();
    }

    // translate any adjustments from pilot to loiter target
    //translate_loiter_target_movements(dt);

    // run loiter position controller
    //get_loiter_position_to_velocity(dt);
}

/// update_wpnav - run the wp controller - should be called at 10hz
void AR_WPNav::update_wpnav()
{
    uint32_t now = hal.scheduler->millis();
    float dt = (now - _wpnav_last_update) / 1000.0f;
    _wpnav_last_update = now;

    // catch if we've just been started
    if( dt >= 1.0 ) {
        dt = 0.0;
        reset_I();
    }else{
    }

    // run loiter position controller
}




/// get_position_to_velocity - loiter position controller
///     converts desired position held in _target vector to desired velocity
void AR_WPNav::get_position_to_velocity(float dt)
{}

/// get_loiter_velocity_to_acceleration - loiter velocity controller
///    converts desired velocities in lat/lon directions to accelerations in lat/lon frame
void AR_WPNav::get_velocity_to_acceleration(float vel_lat, float vel_lon, float dt)
{}

/// get_velocity_to_acceleration - loiter acceleration controller
///    converts desired accelerations provided in lat/lon frame to roll/pitch angles
void AR_WPNav::get_acceleration_to_lean_angles(float accel_lat, float accel_lon)
{}


///
/// shared methods
///


/// set_destination - set destination using cm from home
void AR_WPNav::set_destination(const Vector3f& destination)
{
    // if waypoint controlls is active and copter has reached the previous waypoint use it for the origin
    if( _reached_destination && ((hal.scheduler->millis() - _wpnav_last_update) < 1000) ) {
        _origin = _destination;
    }else{
        // otherwise calculate origin from the current position and velocity
        //project_stopping_point(_inav->get_position(), _inav->get_velocity(), _origin);
    }

    // set origin and destination
    set_origin_and_destination(_origin, destination);
}

/// set_origin_and_destination - set origin and destination using lat/lon coordinates
void AR_WPNav::set_origin_and_destination(const Vector3f& origin, const Vector3f& destination)
{
    _origin = origin;
    _destination = destination;
    _reached_destination = false;
}
// get_bearing_cd - return bearing in centi-degrees between two positions
// To-Do: move this to math library
float AR_WPNav::get_bearing_cd(const Vector3f &origin, const Vector3f &destination) const
{
    float bearing = 9000 + atan2f(-(destination.x-origin.x), destination.y-origin.y) * 5729.57795f;
    if (bearing < 0) {
        bearing += 36000;
    }
    return bearing;
}

/// get_distance_to_destination - get horizontal distance to destination in cm
float AR_WPNav::get_distance_to_destination()
{
    // get current location
    Vector3f curr = _inav->get_position();
    return pythagorous2(_destination.x-curr.x,_destination.y-curr.y);
}

/// get_bearing_to_destination - get bearing to next waypoint in centi-degrees
int32_t AR_WPNav::get_bearing_to_destination()
{
    return get_bearing_cd(_inav->get_position(), _destination);
}

/// get_distance_to_target - get horizontal distance to loiter target in cm
float AR_WPNav::get_distance_to_target() const
{
    return _distance_to_target;
}

/// get_bearing_to_target - get bearing to loiter target in centi-degrees
int32_t AR_WPNav::get_bearing_to_target() const
{
    return get_bearing_cd(_inav->get_position(), _destination);
}

/// reset_I - clears I terms from loiter PID controller
void AR_WPNav::reset_I()
{}
