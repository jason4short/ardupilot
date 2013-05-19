static void init_balance()
{
	wheel_ratio 		= 1000.0 / (float)g.wheel_encoder_speed;
    g_gps->longitude 	= 0;
    g_gps->latitude 	= 0;
    I2Cfail 			= 0;
	_i2c_sem = hal.i2c->get_semaphore();
    if (!_i2c_sem->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        hal.scheduler->panic(PSTR("Failed to get Encoder semaphore"));
    }
   _i2c_sem->give();

	set_armed(false);

    // init Yaw hold
    nav_yaw     		= ahrs.yaw_sensor;
}

// 815 ticks per revolution

static int16_t get_pwm_from_speed_wheel_mixer_left()  // left motor
{
	// mix output speeds
	wheel.left_speed_output = (pitch_out + yaw_out);

	// Lookup our PWM output:
	int16_t wheel_ff		= convert_speed_to_PWM(pwm_LUT_L, wheel.left_speed_output);
	int16_t speed_err 		= wheel.left_speed_output - wheel.left_speed;

    int16_t wheel_P 		= g.pid_wheel_left_mixer.get_p(speed_err);
    int16_t wheel_I 		= g.pid_wheel_left_mixer.get_i(speed_err, .02);
    int16_t wheel_D 		= g.pid_wheel_left_mixer.get_d(speed_err, .02);

	/*cliSerial->printf_P(PSTR("%d, %d, %d, %d\n"),
				wheel.left_speed,
				speed_err,
				wheel_ff,
				wheel_pid);
	//*/

	return (wheel_ff + wheel_P + wheel_I + wheel_D);
}


static int16_t get_pwm_from_speed_wheel_mixer_right()  // right motor
{
	// mix output speeds
	wheel.right_speed_output 	= (pitch_out - yaw_out);

	// Lookup our PWM output:
	int16_t wheel_ff			= convert_speed_to_PWM(pwm_LUT_R, wheel.right_speed_output);	// Lookup our PWM output:
	int16_t speed_err 			= wheel.right_speed_output - wheel.right_speed;

	int16_t wheel_P 			= g.pid_wheel_right_mixer.get_p(speed_err);
    int16_t wheel_I 			= g.pid_wheel_right_mixer.get_i(speed_err, .02);
    int16_t wheel_D 			= g.pid_wheel_right_mixer.get_d(speed_err, .02);

	int16_t output = (wheel_ff + wheel_P + wheel_I + wheel_D);


	/*
							//1   2   3   4   5   6
	cliSerial->printf_P(PSTR("%d, %d, %d, %d, %d, %d\n"),
					(int16_t)ahrs.pitch_sensor,			// 1
					pitch_out,						// 2
					wheel.right_speed,					// 3
					speed_err,							// 4
					wheel_ff,							// 5
					output);							// 6
	//*/
	return output;
}

// read_inertia - read inertia in from accelerometers
static void encoder_nav_update()
{
    static uint8_t log_counter_inav = 0;
    static float last_GDT = 0;

    // inertial altitude estimates
    encoder_nav.update(G_Dt + last_GDT); // 50 hz
    last_GDT = G_Dt;

    if(g.log_bitmask & MASK_LOG_INAV) {
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

	// convert wheel.speed to 1rps = 1000
	int16_t tmp 	= ((float)(wheel.left_speed + wheel.right_speed) * wheel_ratio);

	// divide tmp speeds by number of wheels
	tmp 		 	= tmp >> 1;

	// small averaging
	wheel.speed 	= (wheel.speed + tmp) >> 1;

	// convert to CM/s
	ground_speed 	= convert_encoder_speed_to_ground_speed(wheel.speed);
	encoder_nav.set_velocity(cos_yaw * ground_speed, sin_yaw * ground_speed);


	if(gps_available == false){
		// scaling the mm accuracy to cm
		current_loc.lng	 = current_encoder_x / 10;
		current_loc.lat  = current_encoder_y / 10;
	} else {
		current_loc.lng	 = ((float)g_gps->longitude * .01) + (current_encoder_x / 10) * .99;
		current_loc.lat  = ((float)g_gps->latitude  * .01) + (current_encoder_y / 10) * .99;
	}
	return true;
	//cliSerial->printf_P("left: %ld, right: %ld, lsp: %d, rsp: %d\n", wheel.left, wheel.right, wheel.left_speed, wheel.right_speed);
}

// ------------------

//static float convert_groundspeed_to_encoder_speed(float _ground_speed)
//{
//	return (_ground_speed * (float)g.wheel_encoder_speed ) / WHEEL_DIAMETER_CM;
//}

static float convert_distance_to_encoder_speed(float _distance)
{
	//return (_distance * (float)g.wheel_encoder_speed ) / WHEEL_DIAMETER_CM;
	return (_distance * 1000.0 ) / WHEEL_DIAMETER_CM;
}

static float convert_encoder_speed_to_ground_speed(float encoder_speed)
{
	//return (encoder_speed * WHEEL_DIAMETER_CM) / (float)g.wheel_encoder_speed;
	return (encoder_speed * WHEEL_DIAMETER_CM) / 1000.0;
}

// ------------------

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
