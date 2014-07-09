// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// run_nav_updates - top level call for the autopilot
// ensures calculations such as "distance to waypoint" are calculated before autopilot makes decisions
// To-Do - rename and move this function to make it's purpose more clear
static void run_nav_updates(void)
{
    // fetch position from inertial navigation
	current_loc.lng = encoder_nav.get_longitude();
	current_loc.lat = encoder_nav.get_latitude();


    // calculate distance and bearing for reporting and autopilot decisions
	wp_distance = get_distance_to_destination();
	wp_bearing 	= get_bearing_to_destination();

    loiter_distance = get_distance_to_loiter();

	// obstacle avoidance
	sonar_distance += read_sonar();
	sonar_distance >>= 1;

    // calculate home distance and bearing
    if( ap.home_is_set ) {
	    Vector3f curr = encoder_nav.get_position();
        home_distance = pythagorous2(curr.x, curr.y);
        home_bearing = pv_get_bearing_cd(curr, Vector3f(0, 0, 0));
    }else{
        home_distance = 0;
        home_bearing = 0;
    }

	// only check if we have not reached so we don't "un-reach"
	if(_reached_destination == false){
	    // check we're not orbiting    
		_reached_destination = (wp_distance < g.waypoint_radius) || check_missed_wp();
	}

    //cliSerial->printf("y:%d, lat %ld, lon %ld", (int16_t)(ahrs.yaw_sensor / 100), current_loc.lat, current_loc.lng);
    //cliSerial->printf("\tdis:%d, be %d\n", (int16_t)loiter_distance, (int16_t)(wp_bearing / 100));
    //cliSerial->printf_P(PSTR("desitination  %1.4f, %1.4f\n"), _destination.y, _destination.x);

    // run autopilot to make high level decisions about control modes
    switch( control_mode ) {
        case AUTO:
            // load the next command if the command queues are empty
            update_commands();

            // process the active navigation and conditional commands
            verify_commands();
            break;

        case GUIDED:
            // no need to do anything - wp_nav should take care of getting us to the desired location
            break;

        case RTL:
        	if(verify_RTL()){
        		set_mode(FBW);
        	}
            break;

        case CIRCLE:
            break;
    }

	if(ap.armed)
	    Log_Write_NTUN();
}


void set_destination(const Vector3f& destination)
{
    _reached_destination = false;
    _destination = destination;
}


// calc_pitch_out
static void calc_pitch_out(float speed)
{
	// slow down in front of obstacles
	if(ap.obstacle)
		speed = speed/2;

    int16_t bal_out = 0;
    int16_t vel_out = 0;
    int16_t nav_right_out = 0;
    int16_t nav_left_out = 0;
    float wheel_speed_error;
    int16_t ff_out;

	// switch units to encoder ticks
	desired_ticks   = convert_velocity_to_encoder_speed(speed);

	// grab the wheel speed error
	wheel_speed_error 	= wheel.speed - desired_ticks;
	//                     1000       - 1500 		= -500 too low

	// 4 components of stability and navigation
	bal_out  = get_stabilize_pitch(0);           // hold as vertical as possible
	vel_out  = get_velocity_pitch();             // wheel.speed * 1.0
	ff_out   = (-desired_ticks) * g.throttle;    // desired_ticks * 1.0 

	if(ap.position_hold){
		g.pid_nav_right.reset_I();
		g.pid_nav_left.reset_I();
		nav_right_out = 0;
		nav_left_out = 0;
		
	}else{
		nav_right_out = g.pid_nav_right.get_i(wheel_speed_error, G_Dt);
		nav_left_out  = g.pid_nav_left.get_i(wheel_speed_error, G_Dt);
	}

	// sum the output
	pitch_out_right = bal_out + vel_out + nav_right_out + ff_out;
	pitch_out_left  = bal_out + vel_out + nav_left_out  + ff_out;
}

// speed 0, s 81	acc 0, s 0

float limit_acceleration(float _speed, float acc)
{
	static float _speed_old = 0;
	
	// scale speed by delta time
	acc *= G_Dt;

	if(_speed < _speed_old){ // slow down
		float temp_speed = _speed_old - acc;
		_speed = max(temp_speed, _speed);

	}else if (_speed > _speed_old){ // speed up
		float temp_speed = _speed_old + acc;
		_speed = min(temp_speed, _speed);
	}

	_speed_old = _speed;
	return _speed;
}

    //int16_t speed_limit = g.waypoint_speed;
    //return constrain(_speed, -speed_limit, speed_limit);            // units = cm/s



static bool check_missed_wp()
{
    int32_t temp;
    temp = wp_bearing - original_wp_bearing;
    temp = wrap_180_cd(temp);
    return (labs(temp) > 7500);         // we passed the waypoint by a certain angle in deg * 100
}


static float
get_desired_wp_speed()
{
    /*
    Based on Equation by Bill Premerlani & Robert Lefebvre
    	(sq(V2)-sq(V1))/2 = A(X2-X1)
        derives to:
        V1 = sqrt(sq(V2) - 2*A*(X2-X1))
     */
     
	float _speed;

	if(wp_distance < 4000){ // limit the size of numbers we're dealing with to avoid overflow
		// go slower
		float temp 	= 2 * 100.0 * (wp_distance - g.waypoint_radius);
		temp += (WAYPOINT_SPEED_MIN * WAYPOINT_SPEED_MIN);
		if( temp < 0    ) temp = 0;                // check to ensure we don't try to take the sqrt of a negative number
		_speed = sqrt(temp);
		// limit to max velocity
		_speed = min(_speed, g.waypoint_speed);
		_speed = max(_speed, WAYPOINT_SPEED_MIN); 	// don't go too slow
	}else{
		_speed = g.waypoint_speed;
	}

	return limit_acceleration(_speed, 60.0);
}

static float
get_loiter_speed()
{
	//loiter_distance
	float temp = loiter_distance * g.loiter_gain;
	temp = max(temp, -30);
	temp = min(temp, 30);
	return temp;
}

// Keeps old data out of our calculation / logs
static void reset_nav_params(void)
{
    // Will be set by new command
    wp_bearing                      = 0;

    // Will be set by new command
    wp_distance                     = 0;
    loiter_distance                 = 0;
}

// get_yaw_slew - reduces rate of change of yaw to a maximum
// assumes it is called at 100hz so centi-degrees and update rate cancel each other out
static int32_t get_yaw_slew(int32_t current_yaw, int32_t desired_yaw, int16_t deg_per_sec)
{
    return wrap_360_cd(current_yaw + constrain_int16(wrap_180_cd(desired_yaw - current_yaw), -deg_per_sec, deg_per_sec));
}


//////////////////////////////////////////////////////////
// circle navigation controller
//////////////////////////////////////////////////////////

// circle_set_center -- set circle controller's center position and starting angle
static void
circle_set_center(const Vector3f current_position, float heading_in_radians)
{

}

// update_circle - circle position controller's main call which in turn calls loiter controller with updated target position
static void
update_circle()
{

}


// Crosstrack Error
// ----------------
static int32_t
get_crosstrack(int32_t _bearing)
{
	static int8_t _counter = 0;
	_counter++;

	// called at 100hz, calc at 10hz
	if(_counter >= 10){
		_counter = 0;

		// If we are too far off or too close we don't do track following
		if (labs(wrap_180_cd(wp_bearing - original_wp_bearing)) < 4500){
			// convert angle_cd error to radians
			float temp = (wp_bearing - original_wp_bearing) * RADX100;
			// Meters we are off track line
			_crosstrack_fix = (sin(temp) * (float)wp_distance) * g.crosstrack_gain;
			// scale to degrees
			_crosstrack_fix = constrain(_crosstrack_fix * g.crosstrack_gain, -1000, 1000); // 10 deg change max
		}
	}

	// don't mess with crosstrack beyond 2 meters
	if(wp_distance < 200)
		_crosstrack_fix = 0;

	return wrap_360_cd(_bearing + _crosstrack_fix);
}

#define MIN_DIST 40
#define MAX_DIST 80
static int32_t avoid_obstacle(int32_t _bearing)
{
	static int8_t _counter = 0;

	_counter++;

	if(_counter >= 10){
		_counter = 0;

		if(sonar_distance >= MAX_DIST){
			_avoid_obstacle = 0;
		}else{
			sonar_distance = max(MIN_DIST, sonar_distance);
			float _scale = (float)(sonar_distance - MIN_DIST) / (float)(MAX_DIST - MIN_DIST);
			_avoid_obstacle = 9000.0 * (1.0 - _scale);
		}
	}

	return wrap_360_cd(_bearing + _avoid_obstacle);
}



static void
reset_stall_checker()
{
    obstacle_counter = 0;
}

static void
check_stall()
{
	if(abs(wheel.speed) < 40){
		obstacle_counter++;
	}

	if(abs(wheel.speed) > 500){ // 1 rev every 2 seconds
		//clear counter
		obstacle_counter = 0;
	}

	if(obstacle_counter > 300){ // 3 seconds
	    distance = 0;
		obstacle_counter = 0;
		nav_mode = NAV_AVOID_BACK;
	}
}

