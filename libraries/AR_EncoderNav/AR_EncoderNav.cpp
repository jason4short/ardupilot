/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#include <AP_HAL.h>
#include <AR_EncoderNav.h>

extern const AP_HAL::HAL& hal;

// table of user settable parameters
const AP_Param::GroupInfo AR_EncoderNav::var_info[] PROGMEM = {
    // start numbering at 1 because 0 was previous used for body frame accel offsets
    // @Param: TC
    // @DisplayName: Horizontal Time Constant
    // @Description: Time constant for GPS and accel mixing. Higher TC decreases GPS impact on position estimate
    // @Range: 0 10
    // @Increment: 0.1
    AP_GROUPINFO("TC",   1, AR_EncoderNav, _time_constant, AR_ENCODERNAV_TC),
    
    AP_GROUPEND
};

// init - initialise library
void AR_EncoderNav::init()
{
    // recalculate the gains
    update_gains();
}

// update - updates velocities and positions using latest info from ahrs,
void AR_EncoderNav::update(float dt)
{
    // discard samples where dt is too large
    if( dt > 0.1f ) {
        return;
    }

    // check gps
    check_gps();

    float tmp = _k1 * dt;
    _position_correction.x += _position_error.x * tmp;
    _position_correction.y += _position_error.y * tmp;

    // calculate new estimate of position
    _position_estimation += _velocity * dt;

    // store 3rd order estimate (i.e. horizontal position) for future use at 10hz
    _historic_counter++;
    if( _historic_counter >= AR_ENCODERNAV_SAVE_POS_AFTER_ITERATIONS ) {
        _historic_counter = 0;
        _hist_position_estimate_x.add(_position_estimation.x);
        _hist_position_estimate_y.add(_position_estimation.y);
    }
}

// set time constant - set timeconstant used by complementary filter
void AR_EncoderNav::set_time_constant( float time_constant_in_seconds )
{
    // ensure it's a reasonable value
    if( time_constant_in_seconds > 0 && time_constant_in_seconds < 30 ) {
        _time_constant = time_constant_in_seconds;
        update_gains();
    }
}

// position_ok - return true if position has been initialised and have received gps data within 3 seconds
bool AR_EncoderNav::position_ok() const
{
    return _enabled;
}

// check_gps - check if new gps readings have arrived and use them to correct position estimates
void AR_EncoderNav::check_gps()
{
    uint32_t gps_time;
    uint32_t now = hal.scheduler->millis();

    if( _gps_ptr == NULL || *_gps_ptr == NULL )
        return;

    // get time according to the gps
    gps_time = (*_gps_ptr)->time;

    // compare gps time to previous reading
    if( gps_time != _gps_last_time ) {

        // calculate time since last gps reading
        float dt = (float)(now - _gps_last_update) * 0.001f;

        // call position correction method
        correct_with_gps((*_gps_ptr)->longitude, (*_gps_ptr)->latitude, dt);

        // record gps time and system time of this update
        _gps_last_time = gps_time;
        _gps_last_update = now;
    }

    // clear position error if GPS updates stop arriving
    if( now - _gps_last_update > AR_ENCODERNAV_GPS_TIMEOUT_MS ) {
        _position_error.x = 0;
        _position_error.y = 0;
    }
}

// correct_with_gps - modifies accelerometer offsets using gps.  dt is time since last gps update
void AR_EncoderNav::correct_with_gps(int32_t lon, int32_t lat, float dt)
{
    float x,y;
    float hist_position_est_x, hist_position_est_y;

    // discard samples where dt is too large
    if( dt > 1.0f || dt == 0 || !_enabled) {
        return;
    }

    // calculate distance from estimated location
    x = (float)(lat - _home_lat) * AP_INERTIALNAV_LATLON_TO_CM;
    y = (float)(lon - _home_lon) * _lon_to_m_scaling;

    // ublox gps positions are delayed by 400ms
    // we store historical position at 10hz so 4 iterations ago
    if( _hist_position_estimate_x.num_items() >= AR_ENCODERNAV_GPS_LAG_IN_10HZ_INCREMENTS ) {
        hist_position_est_x = _hist_position_estimate_x.peek(AR_ENCODERNAV_GPS_LAG_IN_10HZ_INCREMENTS-1);
        hist_position_est_y = _hist_position_estimate_y.peek(AR_ENCODERNAV_GPS_LAG_IN_10HZ_INCREMENTS-1);
    }else{
        hist_position_est_x = _position_estimation.x;
        hist_position_est_y = _position_estimation.y;
    }

    // calculate error in position from gps with our historical estimate
    _position_error.x = x - (hist_position_est_x + _position_correction.x); // latitude
    _position_error.y = y - (hist_position_est_y + _position_correction.y); // longitude
}

// get accel based latitude
int32_t AR_EncoderNav::get_latitude() const
{
    // make sure we've been initialised
    if( !_enabled ) {
        return 0;
    }

    return _home_lat + (int32_t)((_position_estimation.x + _position_correction.x)/AP_INERTIALNAV_LATLON_TO_CM);
}

// get accel based longitude
int32_t AR_EncoderNav::get_longitude() const
{
    // make sure we've been initialised
    if( !_enabled ) {
        return 0;
    }

    return _home_lon + (int32_t)((_position_estimation.y+_position_correction.y) / _lon_to_m_scaling);
}

// set_current_position - all internal calculations are recorded as the distances from this point
void AR_EncoderNav::set_current_position(int32_t lon, int32_t lat)
{
    // set base location
    _home_lon = lon;
    _home_lat = lat;

    // set longitude->meters scaling
    // this is used to offset the shrinking longitude as we go towards the poles
    _lon_to_m_scaling = cosf((fabsf((float)lat)*1.0e-7f) * 0.0174532925f) * AP_INERTIALNAV_LATLON_TO_CM;

    // reset corrections to base position to zero
    _position_estimation.x = 0;
    _position_estimation.y = 0;
    _position_correction.x = 0;
    _position_correction.y = 0;

    // clear historic estimates
    _hist_position_estimate_x.clear();
    _hist_position_estimate_y.clear();

    // set as enabled
    _enabled = true;
}

// get accel based latitude
float AR_EncoderNav::get_latitude_diff() const
{
    // make sure we've been initialised
    if( !_enabled ) {
        return 0;
    }

    return ((_position_estimation.x + _position_correction.x) / AP_INERTIALNAV_LATLON_TO_CM);
}

// get accel based longitude
float AR_EncoderNav::get_longitude_diff() const
{
    // make sure we've been initialised
    if( !_enabled ) {
        return 0;
    }

    return (_position_estimation.y + _position_correction.y) / _lon_to_m_scaling;
}

// get velocity in latitude & longitude directions
float AR_EncoderNav::get_latitude_velocity() const
{
    // make sure we've been initialised
    if( !_enabled ) {
        return 0;
    }

    return _velocity.x;
    // Note: is +_velocity.x the output velocity in logs is in reverse direction from accel lat
}

float AR_EncoderNav::get_longitude_velocity() const
{
    // make sure we've been initialised
    if( !_enabled ) {
        return 0;
    }

    return _velocity.y;
}

// set_velocity - set velocity in latitude & longitude directions (in cm/s)
void AR_EncoderNav::set_velocity(float x, float y)
{
    _velocity.x = x;
    _velocity.y = y;
}

//
// Private methods
//

// update_gains - update gains from time constant (given in seconds)
void AR_EncoderNav::update_gains()
{
    // X & Y axis time constant
    if( _time_constant == 0 ) {
        _k1 = _k2 = _k3 = 0;
    }else{
        _k1 = 3 / _time_constant;
        _k2 = 3 / (_time_constant * _time_constant);
        _k3 = 1 / (_time_constant * _time_constant * _time_constant);
    }
}
