#include "Copter.h"

#include <AP_Motors/AP_MotorsMulticopter.h>
#include <AP_Motors/AP_Motors.h>

/*
 * Init and run calls for stabilize flight mode
 */

// COMMANDS: To get roll pitch yaw, throttle
// channel_roll->get_control_in();   channel_pitch->get_control_in();   channel_yaw->get_control_in();  channel_throttle->get_control_in();   get_pilot_desired_throttle()

// COMMANDS: To set roll pitch
// channel_roll->set_control_in(<float value>);   channel_pitch->set_control_in(<float value>);   channel_yaw->set_control_in(<float value>);
	

// REMOVE LATER
float target_roll, target_pitch, target_yaw_rate;
float cur_roll, cur_pitch, cur_yaw, cur_yaw_rate;
float err_roll, err_pitch, err_yaw, err_yaw_rate, err_throt;

// new_control_run - runs the main stabilize controller
// should be called at 100hz or more
void ModeNewControl::run()
{
    // apply simple mode transform to pilot inputs
	// Converts from global pitch roll to drone pitch roll using yaw
    update_simple_mode();

    // convert pilot input to lean angles
	// from mode.cpp, returns lean angles in centi degrees
    // float target_roll, target_pitch;
    get_pilot_desired_lean_angles(target_roll, target_pitch, copter.aparm.angle_max, copter.aparm.angle_max);

    // get pilot's desired yaw rate
	// from Attitude.cpp, returns yaw rate in centi-degrees per second
    target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

    if (!motors->armed()) {
        // Motors should be Stopped
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
    } else if (copter.ap.throttle_zero) {
        // Attempting to Land
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
    } else {
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    }

	// To change
    switch (motors->get_spool_state()) {
    case AP_Motors::SpoolState::SHUT_DOWN:
        // Motors Stopped
        attitude_control->set_yaw_target_to_current_heading();
        attitude_control->reset_rate_controller_I_terms();
        break;

    case AP_Motors::SpoolState::GROUND_IDLE:
        // Landed
        attitude_control->set_yaw_target_to_current_heading();
        attitude_control->reset_rate_controller_I_terms();
        break;

    case AP_Motors::SpoolState::THROTTLE_UNLIMITED:
        // clear landing flag above zero throttle
        if (!motors->limit.throttle_lower) {
            set_land_complete(false);
        }
        break;

    case AP_Motors::SpoolState::SPOOLING_UP:
    case AP_Motors::SpoolState::SPOOLING_DOWN:
        // do nothing
        break;
    }

    // // call attitude controller
    // attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);

	

    // // output pilot's throttle
    // attitude_control->set_throttle_out(get_pilot_desired_throttle(),
                                       // true,
                                       // g.throttle_filt)
	
	// Custom Controller Loop
	update_motors();
}

void ModeNewControl::update_motors()
{
	//// Get AHRS reading (all angles in radians)
	Vector3f gyro_latest = ahrs.get_gyro_latest();
	cur_roll = gyro_latest.x; 		// ahrs.get_roll();
	cur_pitch = gyro_latest.y; 		// ahrs.get_pitch();
	cur_yaw = gyro_latest.z; // absolute yaw using compass (not sure)
	cur_yaw_rate = ahrs.get_yaw_rate_earth(); 	// in rads/sec
	
	printf("Target R: %f, P: %f, Yr: %f\n", target_roll, target_pitch, target_yaw_rate);
	printf("Curren R: %f, P: %f, Yr: %f, Y: %f\n", cur_roll, cur_pitch, cur_yaw_rate, cur_yaw);
	
	// Error Roll Pitch Yaw
	// err_roll = cur_roll - target_roll;
	// err_pitch = cur_pitch - target_pitch;
	// err_yaw_rate = cur_yaw_rate - target_yaw_rate;
	
	
	// // Get Gyro Reading
	// Vector3f gyro_latest = ahrs.get_gyro_latest();
	// ahrs.get_yaw_rate_earth() --> returns float

	
	// Vector3f _attitude_target_euler_angle;
	
	// attitude_target_euler_angle.x = euler_roll_angle;
	// attitude_target_euler_angle.y = euler_pitch_angle;
	// attitude_target_euler_angle.z += euler_yaw_rate * _dt;
	// float dt = 1/400;
	// Quaternion attitude_target_quat;
	// attitude_target_quat.from_euler(t_roll, t_pitch, t_yaw_rate * dt);
	// attitude_target_quat.from_euler(_attitude_target_euler_angle.x, _attitude_target_euler_angle.y, _attitude_target_euler_angle.z);
	
	// Convert Gyro Readings to Roll Pitch Yaw
	
	// Get Error
	
	// Apply Controller loop on Error
	
	// Output to motors
	SRV_Channels::set_output_pwm_chan(0, 1622); // uint16_t val		// Motor Pos:
	SRV_Channels::set_output_pwm_chan(1, 1624); // uint16_t val 	// Motor Pos:
	SRV_Channels::set_output_pwm_chan(2, 1624); // uint16_t val		// Motor Pos: 
	SRV_Channels::set_output_pwm_chan(3, 1622); // uint16_t val		// Motor Pos:
    SRV_Channels::cork();				// cork now, so that all channel outputs happen at once
    SRV_Channels::output_ch_all();		// update output on any aux channels, for manual passthru
    SRV_Channels::push();				// push all channels	
}