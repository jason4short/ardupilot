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
	//target_bearing 	= wp_bearing;

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
}


void set_destination(const Vector3f& destination)
{
    _reached_destination = false;
    _destination = destination;
}


// calc_pitch_out
static void calc_pitch_out(int16_t speed)
{
    int16_t bal_out = 0;
    int16_t vel_out = 0;
    int16_t nav_out = 0;
    int16_t wheel_speed_error, ff_out;

	// limit speed
	speed   = limit_acceleration(speed, 50); // cm/s

	// switch units to encoder ticks
	desired_ticks   = convert_velocity_to_encoder_speed(speed);

	// grab the wheel speed error
	wheel_speed_error 	= wheel.speed - desired_ticks;

	// 4 components of stability and navigation
	bal_out         = get_stabilize_pitch(0);                           // hold as vertical as possible
	vel_out         = get_velocity_pitch();                             // magic
	ff_out          = (float)desired_ticks * g.throttle;                       // allows us to roll while vertical
	nav_out      	= g.pid_nav.get_pid(wheel_speed_error, G_Dt);

	// sum the output
	pitch_out = (bal_out + vel_out + nav_out - ff_out);
}

int16_t limit_acceleration(int16_t speed, int16_t acc)
{
	// scale speed by delta time
	acc *= G_Dt;

	if(speed < ground_speed){ // slow down
		int16_t temp_speed = ground_speed - acc;
		speed = max(temp_speed, speed);

	}else if (speed > ground_speed){ // speed up
		int16_t temp_speed = ground_speed + acc;
		speed = min(temp_speed, speed);
	}

    int16_t speed_limit = g.waypoint_speed;
    return constrain(speed, -speed_limit, speed_limit);            // units = cm/s
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


// ------------------------

static int32_t avoid_obstacle(int32_t bearing)
{
	if(sonar_distance > 220){
		return bearing;
	}

	float scale = (float)(sonar_distance - 20) / 200.0;
	bearing += 9000 * scale;
	return wrap_360_cd(bearing);
}

