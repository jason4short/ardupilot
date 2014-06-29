static void init_balance()
{
    // wheel ratio is used to convert ticks into 1000 t/s for 1 wheel rotation
	wheel_ratio 		= 1000.0 / (float)g.wheel_encoder_speed;
    
    // for use without GPS unit.
    g_gps->longitude 	= 0;
    g_gps->latitude 	= 0;
    I2Cfail 			= 0;
	_i2c_sem = hal.i2c->get_semaphore();
	
    if (!_i2c_sem->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        hal.scheduler->panic(PSTR("Failed to get Encoder semaphore"));
    }
    
   _i2c_sem->give();

    // we will not spin the wheels
	set_armed(false);

    // init Yaw hold
    nav_yaw     		= ahrs.yaw_sensor;
}

// read_inertia - read inertia in from accelerometers
static void encoder_nav_update()
{
    static uint8_t log_counter_inav = 0;
    static float last_GDT = 0;

    // inertial altitude estimates
    encoder_nav.update(G_Dt + last_GDT); // 50 hz
    last_GDT = G_Dt;

    if(ap.armed && g.log_bitmask & MASK_LOG_INAV) {
        log_counter_inav++;
        if( log_counter_inav >= 10 ) {
            log_counter_inav = 0;
            Log_Write_INAV();
        }
    }
}

static bool update_wheel_encoders()
{
	uint8_t buff[12];

   if (!_i2c_sem->take(5)) {
       // the bus is busy - try again later
       return false;
   }
	if (hal.i2c->read(ENCODER_ADDRESS, sizeof(buff), buff) != 0) {
		I2Cfail++;
	   _i2c_sem->give();
		return false;
	}
	_i2c_sem->give();


	memcpy(bytes_union.bytes, &buff[1], 2);
	wheel.left_distance = bytes_union.int_value * WHEEL_ENCODER_DIR_LEFT;

	memcpy(bytes_union.bytes, &buff[3], 2);
	wheel.right_distance = bytes_union.int_value * WHEEL_ENCODER_DIR_RIGHT;

	// raw ticks per sec
	memcpy(bytes_union.bytes, &buff[5], 2);
	wheel.left_speed = bytes_union.int_value * WHEEL_ENCODER_DIR_LEFT;

	memcpy(bytes_union.bytes, &buff[7], 2);
	wheel.right_speed = bytes_union.int_value * WHEEL_ENCODER_DIR_RIGHT;

	// convert wheel.speed to 1 rotation per second = 1000 ticks
	float tmp 	= ((float)(wheel.left_speed + wheel.right_speed) * wheel_ratio);

	// divide tmp speeds by number of wheels
	tmp /= 2;

	// small averaging
	// Wheel speed is in 1000 ticks per rotation
	wheel.speed 	= (wheel.speed + tmp) /2;
	//cliSerial->printf_P("left: %ld, right: %ld, lsp: %d, rsp: %d\n", wheel.left_distance, wheel.right_distance, wheel.left_speed, wheel.right_speed);

	// convert to CM/s
	ground_speed 	= convert_encoder_speed_to_velocity(wheel.speed);
	encoder_nav.set_velocity(sin_yaw * ground_speed, cos_yaw * ground_speed);
	
	// sum distnace traveled
	// can be reset by nav code, used for obstacle avoidance
	distance += fabs(ground_speed) * .02;
	
	
	return true;
}

// ------------------


static float convert_velocity_to_encoder_speed(float _wheel_velocity)
{
	return (_wheel_velocity * 1000.0 ) / g.wheel_diameter;
}

static float convert_encoder_speed_to_velocity(float encoder_speed)
{
	return (encoder_speed * g.wheel_diameter) / 1000.0;
	//1 RPS = 307.87
	
	//return (1000 * 31.11) / 1000.0;

}

//

// ------------------

/*
static int16_t convert_speed_to_PWM(int16_t lut[], int16_t encoder_speed){
	int8_t flip_sign = (encoder_speed < 0) ? -1 : 1;

	uint16_t input;
	uint8_t index;

	if(abs(encoder_speed) > (PWM_LUT_SIZE * 100)){
		return lut[PWM_LUT_SIZE-1] * flip_sign;
	}

	input 	= constrain(abs(encoder_speed), 0, (PWM_LUT_SIZE * 100));
	index	= input / 100;   //

	int16_t output = lut[index] + ((input - index * 100) * (lut[index + 1] - lut[index])) / 100;
	return output * flip_sign;
}
*/