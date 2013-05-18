/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AR_ENCODERNAV_H__
#define __AR_ENCODERNAV_H__

#include <AP_AHRS.h>
#include <AP_Buffer.h>                  // FIFO buffer library

#define AR_ENCODERNAV_TC   8.0f 			// default time constant for complementary filter's X & Y axis
#define AR_ENCODERNAV_GPS_LAG  4			// must not be larger than size of _hist_position_estimate_x and _hist_position_estimate_y
#define AR_ENCODERNAV_GPS_TIMEOUT_MS 300    // timeout after which position error from GPS will fall to zero

#define AP_ENCODERNAV_LATLON_TO_CM 1.1113175f

/*
 * AR_EncoderNav is an attempt to use accelerometers to augment other sensors to improve altitud e position hold
 */
class AR_EncoderNav
{
public:

    // Constructor
    AR_EncoderNav( GPS** gps_ptr ) :
        _gps_ptr(gps_ptr),
        _enabled(true),
        _gps_last_update(0),
        _gps_last_time(0)
        {
            AP_Param::setup_object_defaults(this, var_info);
        }

    // Initialisation
    void        init();

    // update - updates velocities and positions using latest info from accelerometers;
    void        update(float dt);

    // set time constant - set timeconstant used by complementary filter
    void        set_time_constant( float time_constant_in_seconds );

    // check_gps - check if new gps readings have arrived and use them to correct position estimates
    void        check_gps();

    // correct_with_gps - modifies accelerometer offsets using gps.  dt is time since last gps update
    void        correct_with_gps(int32_t lon, int32_t lat, float dt);

    // get_position - returns current position from home in cm
    Vector3f    get_position() const { return _position_estimation + _position_correction; }

    // get latitude & longitude positions
    int32_t     get_latitude() const;
    int32_t     get_longitude() const;

    // set_current_position - all internal calculations are recorded as the distances from this point
    void        set_current_position(int32_t lon, int32_t lat);

    // get latitude & longitude positions from base location (in cm)
    float       get_latitude_diff() const;
    float       get_longitude_diff() const;

    // get velocity in latitude & longitude directions (in cm/s)
    float       get_latitude_velocity() const;
    float       get_longitude_velocity() const;

    // get_velocity - returns current velocity in cm/s
    Vector3f    get_velocity() const { return _velocity; }

    // set velocity in latitude & longitude directions (in cm/s)
    void        set_velocity(float x, float y);


    // class level parameters
    static const struct AP_Param::GroupInfo var_info[];

    // public variables
    //Vector3f                accel_correction_ef;        // earth frame accelerometer corrections. here for logging purposes only

protected:

    void                    update_gains();             // update_gains - update gains from time constant (given in seconds)

    GPS**                   _gps_ptr;                   // pointer to pointer to gps

    bool                    _enabled;                	//  position estimates enabled
    AP_Float                _time_constant; 	        // time constant for horizontal corrections
    float                   _k1;        	            // gain for horizontal position correction
    uint32_t                _gps_last_update;           // system time of last gps update
    uint32_t                _gps_last_time;             // time of last gps update according to the gps itself
    uint8_t                 _historic_counter;       	// counter used to slow saving of position estimates for later comparison to gps
    AP_BufferFloat_Size5    _hist_position_estimate_x;  // buffer of historic accel based position to account for lag
    AP_BufferFloat_Size5    _hist_position_estimate_y;  // buffer of historic accel based position to account for lag
    int32_t                 _home_lat;                  // base latitude
    int32_t                 _home_lon;                  // base longitude
    float                   _lon_to_m_scaling;          // conversion of longitude to meters

    // general variables
    Vector3f                _position_estimation;             // position estimate
    Vector3f                _position_correction;       // sum of correction to _comp_h from delayed 1st order samples
    Vector3f                _velocity;                  // latest velocity estimate (integrated from accelerometer values)
    Vector3f                _position_error;
};

#endif // __AR_ENCODERNAV_H__
