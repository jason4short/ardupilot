// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// position_vector.pde related utility functions

// position vectors are Vector2f
//    .x = latitude from home in cm
//    .y = longitude from home in cm
//    .z = altitude above home in cm

// pv_latlon_to_vector - convert lat/lon coordinates to a position vector
const Vector3f pv_latlon_to_vector(int32_t lat, int32_t lon, int32_t alt)
{
    Vector3f tmp((lat-home.lat) * LATLON_TO_CM, (lon-home.lng) * LATLON_TO_CM * scaleLongDown, alt);
    return tmp;
}

// pv_latlon_to_vector - convert lat/lon coordinates to a position vector
const Vector3f pv_location_to_vector(Location loc)
{
    Vector3f tmp((loc.lat-home.lat) * LATLON_TO_CM, (loc.lng-home.lng) * LATLON_TO_CM * scaleLongDown, loc.alt);
    return tmp;
}

// pv_get_lon - extract latitude from position vector
const int32_t pv_get_lat(const Vector3f pos_vec)
{
    return home.lat + (int32_t)(pos_vec.x / LATLON_TO_CM);
}

// pv_get_lon - extract longitude from position vector
const int32_t pv_get_lon(const Vector3f pos_vec)
{
    return home.lng + (int32_t)(pos_vec.y / LATLON_TO_CM * scaleLongUp);
}

// pv_get_bearing_cd - return bearing in centi-degrees between two positions
const float pv_get_bearing_cd(const Vector3f origin, const Vector3f destination)
{
    float bearing = 9000 + atan2f(-(destination.x-origin.x), destination.y-origin.y) * DEGX100;
    if (bearing < 0) {
        bearing += 36000;
    }
    return bearing;
}


/// get_distance_to_destination - get horizontal distance to destination in cm
float get_distance_to_destination()
{
    // get current location
    Vector3f curr = encoder_nav.get_position();
    return pythagorous2(_destination.x - curr.x, _destination.y - curr.y);
}

/// get_bearing_to_destination - get bearing to next waypoint in centi-degrees
int32_t get_bearing_to_destination()
{
    return pv_get_bearing_cd(encoder_nav.get_position(), _destination);
}

