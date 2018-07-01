#include "Sub.h"

// stabilize_init - initialise stabilize controller
bool Sub::stabilize_init()
{
    // set target altitude to zero for reporting
    pos_control.set_alt_target(0);

    // attitude hold inputs become thrust inputs in manual mode
    // set to neutral to prevent chaotic behavior (esp. roll/pitch)
    set_neutral_controls();

    return true;
}

// stabilize_run - runs the main stabilize controller
// should be called at 100hz or more
void Sub::stabilize_run()
{
     // if not armed set throttle to zero and exit immediately
    if (!motors.armed()) {
        motors.set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);
        return;
    }

    motors.set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    //motors.rc_write(uint8_t chan, uint16_t pwm)
    
    unsigned short* rc_pwm_input;
    unsigned short rc_input_values[8];

    for (int i=0; i<8; i++) {
        rc_input_values[i] = RC_Channels::rc_channel(i)->read();
        //rc_input_values[i] = (unsigned short)(*(RC_Channels::rc_channel(i)));

    }

    rc_pwm_input = rc_input_values;

    motors.direct_thruster_control(rc_pwm_input);

    /*
    motors.rc_write(1, (*(RC_Channels::rc_channel(1))));
    motors.rc_write(2, (*(RC_Channels::rc_channel(2))));
    motors.rc_write(3, (*(RC_Channels::rc_channel(3))));
    motors.rc_write(4, (*(RC_Channels::rc_channel(4))));
    motors.rc_write(5, (*(RC_Channels::rc_channel(5))));
    motors.rc_write(6, (*(RC_Channels::rc_channel(6))));
    motors.rc_write(7, (*(RC_Channels::rc_channel(7))));
    */


    float dt =  ((float)(micros() - last_control_mode_update_us)) / 1e6;
    motors.limit_demand_slew_rate(g.manual_slew_rate, dt);
}
