// Camera Control
// --------------------------
static uint32_t gimbal_timer;
static float camera_angle; // 0° to 90°
static float camera_accel = 50;
static float camera_rate;
static float camera_input_rate; // -1 tp 1

static float max_camera_angle = 90;
static float min_camera_angle;
static float roi_distance;


static void 
gimbal_run_manual(int16_t stick_input)
{
    camera_input_rate = (float)stick_input / 4500.0;
    
    // calc camera rotation
    calc_camera_rate();
    camera_angle += camera_rate * .02; // 50 hz

    // Constrain limits of camera
    if(camera_angle < min_camera_angle){
        camera_rate = 0;
        camera_angle = min_camera_angle;
        
    }else if (camera_angle > max_camera_angle){
        camera_rate = 0;
        camera_angle = max_camera_angle;
    }
    //gimbal_update();
}

static void 
gimbal_run_roi()
{
    float deltaX, deltaY;
    Vector3f _position;
	
	// grab the relative location of the vehicle
	_position       = inertial_nav.get_position();
	//_position.x     = 0; // XXX
	//_position.y     = 0; // XXX
    //_position.z     = 1000; // XXX
    deltaX          = _position.x - roi_WP.x;
    deltaY          = _position.y - roi_WP.y;


	// Calc distance to ROI
	roi_distance     = safe_sqrt(deltaX * deltaX + deltaY * deltaY);

	// Calc Angle of camera (0 = level, 90 = straight down)
	camera_angle    = degrees(fast_atan2((_position.z - roi_WP.z), roi_distance)) ;
	camera_angle    = constrain_float(camera_angle, 0, 90); // 0 to 90

    //gimbal_update();
}

static void
gimbal_update()
{
    if(control_mode == GAMER){
        // Super hack
        g.rc_9.function = 0;
        
        // 0 = level
        // 90 = straight down
        //cliSerial->printf_P(PSTR("%1.2f\n"), camera_angle);
        hal.rcout->write(8, (int16_t)(1000 + (1 - (camera_angle / 90.0)) * 520));

    }else{

        g.rc_9.function = 7;
        
        // pass through CH6 to Gimbal
        //hal.rcout->write(8, g.rc_6.radio_in);
        
        // update camera mount's position
        camera_mount.update_mount_position();
    }

}


static void
calc_camera_rate()
{
    float desired_rate = calc_max_rate() * camera_input_rate * camera_accel;
    
    //limit acceleration
    if (desired_rate > camera_rate){
        desired_rate = camera_rate + 3;
    }  else if (desired_rate < camera_rate){
        desired_rate = camera_rate - 3;
    }            
    
    camera_rate = desired_rate;
}

#define MARGIN 30
static float
calc_max_rate()
{
    camera_angle = constrain_float(camera_angle, 0, 90);
    float new_speed;
                
    if(camera_angle < MARGIN && camera_input_rate < 0){
        new_speed = ease_user_input(camera_angle / MARGIN);
        
    }else if(camera_angle > 90 - MARGIN && camera_input_rate > 0){
        new_speed = ease_user_input((90 - camera_angle) / MARGIN);
        
    }else{
        new_speed = 1;
    }
    
    return new_speed;
}


static float 
ease_user_input (float _delta)
{
    return sqrt(1 - ((_delta - 1) * (_delta - 1)));
}

static float 
get_camera_angle ()
{
    return camera_angle;
}

static float 
get_roi_distance()
{
    return roi_distance;
}


static void
calc_roi_from_angle(float _camera_angle)
{
    Vector3f _position;
    float _distance;
    
	// grab the relative location of the vehicle
	_position = inertial_nav.get_position();
	
	//_position.z = 1000; // XXX
	//_position.x = 0; // XXX
	//_position.y = 0; // XXX
	//camera_angle = 63.4349488; // XXX
	
	// limit the range of the camera angle so we don't gimbal lock or look too far out.
	_camera_angle = constrain_float(_camera_angle, 1, 80);

    // calc _distance
    _distance = (1/tan(_camera_angle * .0174533f)) * _position.z;
    
    // limit distance to 3x the altitude (18°)
    _distance = min(_distance, (_position.z * 3.0));
    
    // rotate _distance to world frame
    //Lat N/S:
    roi_WP.x = _position.x + (ahrs.cos_yaw() * _distance);

    //lon E/W
    roi_WP.y = _position.y + (ahrs.sin_yaw() * _distance);
    
    // defaulting to 2m heigh (a person's face)
    //roi_WP.z = 0;
    
    //cliSerial->printf_P(PSTR("ROI: an:%1.3f, yaw:%1.2f, sin:%1.3f, cos:%1.3f, dis:%1.3f, x:%1.3f, y:%1.3f\n"), _camera_angle, (float)ahrs.yaw_sensor, ahrs.sin_yaw(), ahrs.cos_yaw(), _distance, roi_WP.x, roi_WP.y);
}
//ROI: an:9000.000, yaw:35932.00, sin:-0.011, cos:0.999, dis:-1.281, x:0.015, y:-1.281

