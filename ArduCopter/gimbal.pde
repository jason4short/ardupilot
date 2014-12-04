// Camera Control
// --------------------------
static uint32_t gimbal_timer;
static float camera_angle;        
static float camera_accel = 1;
static float camera_rate;
static float camera_rate_old;
static float camera_input_rate; // -1 tp 1

static float max_camera_angle = 90;
static float min_camera_angle;


static void 
gimbal_run(int16_t stick_input)
{
    // output to the gimbal

    camera_input_rate = (float)stick_input / 4500.0;

    
    //cliSerial->printf("%1.2f\n", camera_input_rate);
    
    // calc camera rotation
    calc_camera_rate();
    //camera_angle += camera_rate * .01; // 50 hz
    camera_angle += camera_rate * .015; // 50 hz
    //camera_angle += camera_rate * .0025; // 400 hz

    // Constrain limits of camera
    if(camera_angle < min_camera_angle){
        camera_rate = 0;
        camera_angle = min_camera_angle;
        
    }else if (camera_angle > max_camera_angle){
        camera_rate = 0;
        camera_angle = max_camera_angle;
    }
    // hal write
    //pwm_output[CH_6] = 1000 + (camera_angle / 90.0) * 520;    
    hal.rcout->write(8, (int16_t)(1000 + (camera_angle / 90.0) * 520)); //
}




static void
calc_camera_rate()
{
    float desired_rate = calc_max_rate() * (camera_input_rate * camera_accel);
    
    desired_rate -= (desired_rate - camera_rate_old) * .3;
    camera_rate_old = camera_rate;
    camera_rate = desired_rate;
}

static float
calc_max_rate()
{
    camera_angle = constrain_float(camera_angle, .1, 89.9);
    float new_speed;
                
    if(camera_angle < 45){
        // 0-45
        if(camera_input_rate < 0){  // up
            new_speed = ease_user_input(camera_angle, 0, 45, 45);
        }else{
            new_speed = 45;
        }
    
    }else{
        // 45-90
        if(camera_input_rate > 0){  // down
            new_speed = ease_user_input(90 - camera_angle, 0, 45, 45);
        }else{
            new_speed = 45;
        }
    }
    return new_speed;
}


static float 
ease_user_input (float _delta, float _start, float _change, float _duration)
{
    _delta = (_delta / _duration) - 1;
    return _change * sqrt(1 - (_delta  * _delta)) + _start;
}

