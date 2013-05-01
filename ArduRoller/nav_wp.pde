


/// update_loiter - should be called at 10hz
void update_loiter()
{
    uint32_t now = hal.scheduler->millis();
    float dt = (now - _loiter_last_update) / 1000.0f;
    _loiter_last_update = now;

    // catch if we've just been started
    if( dt >= 1.0 ) {
        dt = 0.0;
        reset_I_all();
    }


}

/// update_wpnav should be called at 10hz
void update_wpnav()
{
    uint32_t now = hal.scheduler->millis();
    float dt = (now - _wpnav_last_update) / 1000.0f;
    _wpnav_last_update = now;

    // catch if we've just been started
    if( dt >= 1.0 ) {
        dt = 0.0;
        reset_I_all();
    }


}



void set_destination(const Vector3f& destination)
{
    // if waypoint controlls is active and copter has reached the previous waypoint use it for the origin
    if( _reached_destination && ((hal.scheduler->millis() - _wpnav_last_update) < 1000) ) {
        _origin = _destination;
    }else{
        // otherwise calculate origin from the current position and velocity
        //project_stopping_point(inertial_nav.get_position(), inertial_nav.get_velocity(), _origin);
    }

    // set origin and destination
    set_origin_and_destination(_origin, destination);
}

/// set_origin_and_destination - set origin and destination using lat/lon coordinates
void set_origin_and_destination(const Vector3f& origin, const Vector3f& destination)
{
    _origin = origin;
    _destination = destination;
    _reached_destination = false;
}



//
void set_horizontal_velocity(float velocity_cms)
{
	//_speed_xy_cms = velocity_cms;
};



// get_bearing_cd - return bearing in centi-degrees between two positions
// To-Do: move this to math library
float get_bearing_cd(const Vector3f &origin, const Vector3f &destination)
{
    float bearing = 9000 + atan2f(-(destination.x-origin.x), destination.y-origin.y) * 5729.57795f;
    if (bearing < 0) {
        bearing += 36000;
    }
    return bearing;
}

/// get_distance_to_destination - get horizontal distance to destination in cm
float get_distance_to_destination()
{
    // get current location
    Vector3f curr = inertial_nav.get_position();
    return pythagorous2(_destination.x-curr.x,_destination.y-curr.y);
}

/// get_bearing_to_destination - get bearing to next waypoint in centi-degrees
int32_t get_bearing_to_destination()
{
    return get_bearing_cd(inertial_nav.get_position(), _destination);
}

/// get_distance_to_target - get horizontal distance to loiter target in cm
float get_distance_to_target()
{
    return _distance_to_target;
}

/// get_bearing_to_target - get bearing to loiter target in centi-degrees
int32_t get_bearing_to_target()
{
    return get_bearing_cd(inertial_nav.get_position(), _destination);
}
