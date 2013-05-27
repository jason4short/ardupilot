// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
 *       This event will be called when the failsafe changes
 *       boolean failsafe reflects the current state
 */

static void low_battery_event(void)
{
    // failsafe check
    if (g.failsafe_battery_enabled && !ap.low_battery && ap.armed) {
        switch(control_mode) {
            case STABILIZE:
                // if throttle is zero disarm motors
                if (g.rc_3.control_in == 0) {
                    init_disarm_motors();
                }
                break;
            case AUTO:
                if(ap.home_is_set == true && home_distance > g.waypoint_radius) {
                    set_mode(RTL);
                }
                break;
        }
    }

    // set the low battery flag
    set_low_battery(true);

    // warn the ground station and log to dataflash
    gcs_send_text_P(SEVERITY_LOW,PSTR("Low Battery!"));
    Log_Write_Error(ERROR_SUBSYSTEM_FAILSAFE_BATT, ERROR_CODE_FAILSAFE_OCCURRED);

#if COPTER_LEDS == ENABLED
    // Only Activate if a battery is connected to avoid alarm on USB only
    piezo_on();
#endif // COPTER_LEDS
}

// failsafe_gps_check - check for gps failsafe
/*
static void failsafe_gps_check()
{
    uint32_t last_gps_update_ms = millis() - g_gps->last_fix_time;

    // check if all is well
    if( last_gps_update_ms > FAILSAFE_GPS_TIMEOUT_MS) {
		// do nothing if gps failsafe already triggered or motors disarmed
		if(!ap.failsafe_gps){
			set_failsafe_gps(true);
			gcs_send_text_P(SEVERITY_LOW,PSTR("Lost GPS!"));
			Log_Write_Error(ERROR_SUBSYSTEM_FAILSAFE_GPS, ERROR_CODE_FAILSAFE_OCCURRED);
		}
    }else{
        // check for recovery from gps failsafe
        if(ap.failsafe_gps){
            set_failsafe_gps(false);
            Log_Write_Error(ERROR_SUBSYSTEM_FAILSAFE_GPS, ERROR_CODE_FAILSAFE_RESOLVED);
        }
	}
}
*/

static void update_events()     // Used for MAV_CMD_DO_REPEAT_SERVO and MAV_CMD_DO_REPEAT_RELAY
{
    if(event_repeat == 0 || (millis() - event_timer) < event_delay)
        return;

    if(event_repeat != 0) {             // event_repeat = -1 means repeat forever
        event_timer = millis();

        if (event_id >= CH_5 && event_id <= CH_8) {
            if(event_repeat%2) {
                hal.rcout->write(event_id, event_value);                 // send to Servos
            } else {
                hal.rcout->write(event_id, event_undo_value);
            }
        }

        if  (event_id == RELAY_TOGGLE) {
            relay.toggle();
        }
        if (event_repeat > 0) {
            event_repeat--;
        }
    }
}

