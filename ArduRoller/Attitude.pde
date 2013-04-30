/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

static void
get_stabilize_roll(int32_t target_angle)
{}

static void
get_stabilize_pitch(int32_t target_angle)
{}

static void
get_stabilize_yaw(int32_t target_angle)
{}



/*************************************************************
 * yaw controllers
 *************************************************************/

 // get_look_at_yaw - updates bearing to look at center of circle or do a panorama
// should be called at 100hz
static void get_circle_yaw()
{
    static uint8_t look_at_yaw_counter = 0;     // used to reduce update rate to 10hz

    // if circle radius is zero do panorama
    if( g.circle_radius == 0 ) {
        // slew yaw towards circle angle
        nav_yaw = get_yaw_slew(nav_yaw, ToDeg(circle_angle)*100, AUTO_YAW_SLEW_RATE);
    }else{
        look_at_yaw_counter++;
        if( look_at_yaw_counter >= 10 ) {
            look_at_yaw_counter = 0;
            yaw_look_at_WP_bearing = pv_get_bearing_cd(inertial_nav.get_position(), yaw_look_at_WP);
        }
        // slew yaw
        nav_yaw = get_yaw_slew(nav_yaw, yaw_look_at_WP_bearing, AUTO_YAW_SLEW_RATE);
    }

    // call stabilize yaw controller
    get_stabilize_yaw(nav_yaw);
}



/*
 *  reset all I integrators
 */
static void reset_I_all(void)
{
    reset_rate_I();
    reset_stability_I();

    // This is the only place we reset Yaw
    g.pi_stabilize_yaw.reset_I();
}

static void reset_rate_I()
{
    g.pid_rate_roll.reset_I();
    g.pid_rate_pitch.reset_I();
    g.pid_rate_yaw.reset_I();
}


static void reset_stability_I(void)
{
    // Used to balance a quad
    // This only needs to be reset during Auto-leveling in flight
    g.pi_stabilize_roll.reset_I();
    g.pi_stabilize_pitch.reset_I();
}
