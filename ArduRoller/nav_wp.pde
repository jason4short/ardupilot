
void set_destination(const Vector3f& destination)
{
    _reached_destination = false;
    _destination = destination;
}


/// update_loiter - should be called at 10hz
void update_loiter()
{
    uint32_t now = hal.scheduler->millis();
    float dt = (now - _loiter_last_update) / 1000.0f;
    _loiter_last_update = now;

    // catch if we've just been started
    if(dt >= 1.0){
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
    if(dt >= 1.0){
        dt = 0.0;
        reset_I_all();
    }

}



static int16_t
get_nav_pitch(int16_t minSpeed, float dist_err)
{
	static float desired_speed_old = 0;
	int16_t nav_out, ff_out, speed_error;

	//dist_err		= (float)long_error * cos_yaw_x + (float)lat_error * sin_yaw_y;
    //dist_err 		= constrain(dist_err, 0, 45);

    // this is the speed of the wheels: 1000 = 1 rotation of the wheels.
    // We convert cm to rpm based on wheel diamter and encoder ticks per revolution
    desired_speed 	= convert_distance_to_encoder_speed(dist_err);

	// accleration is limited to prevent wobbly starts towards waypoints
	desired_speed 	= min(desired_speed, desired_speed_old + 10);// limit going faster
	desired_speed 	= max(desired_speed, minSpeed);
	desired_speed_old = desired_speed;

	// We simply rotated the wheels at the desired speed times a proportional value
	// stabilizer manages uprightness
    ff_out          = (float)desired_speed * g.throttle;

	// grab the wheel speed error
	speed_error 	= wheel.speed - desired_speed;
	nav_out      	= g.pid_nav.get_pid(speed_error, G_Dt);

    return constrain((nav_out - ff_out), -2000, 2000);
}
