void init_balance()
{
	wheel_ratio 		= 1000.0 / g.wheel_encoder_speed;
	tilt_start 			= false;
    g_gps->longitude 	= 0;
    g_gps->latitude 	= 0;
    fail 				= 0;
    init_home();

    // init Yaw hold
    nav_yaw     		= ahrs.yaw_sensor;
}
