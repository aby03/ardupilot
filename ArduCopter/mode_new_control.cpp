#include "Copter.h"

#include <AP_Motors/AP_MotorsMulticopter.h>
#include <AP_Motors/AP_Motors.h>

// COMMANDS: To get roll pitch yaw, throttle
// channel_roll->get_control_in();   channel_pitch->get_control_in();   channel_yaw->get_control_in();  channel_throttle->get_control_in();   get_pilot_desired_throttle()

// COMMANDS: To set roll pitch
// channel_roll->set_control_in(<float value>);   channel_pitch->set_control_in(<float value>);   channel_yaw->set_control_in(<float value>);
	

// REMOVE LATER
int roll_i = 0, pitch_i = 1, yaw_i = 2, yaw_rate_i = 3, throttle_i = 4;
int A = 0, B = 1, C = 2, D = 3;
float target[5];
float current[5];
float pulse_width[4];
float pulse_width_max[4];
float pulse_width_min[4];
float error[5];
float error_sum[5];
float error_min[5];
float error_max[5];
float delta_err[5], prev_error[5];
float pid[5];
float pid_max[5];
float pid_min[5];

// PID
float kp[5];
float ki[5];
float kd[5];

float target_roll, target_pitch, target_yaw_rate, target_throt;
float cur_roll, cur_pitch, cur_yaw, cur_yaw_rate, cur_throt;
float err_roll, err_pitch, err_yaw, err_yaw_rate, err_throt;


/*
 * Init and run calls for stabilize flight mode
 */
bool ModeNewControl::init(bool ignore_checks)
{
	// current[throttle_i] = 1500;
	// return true;
	// PID Parameters
	kp[roll_i] = 1;
	kp[pitch_i] = 1;
	kp[yaw_rate_i] = 1;
	
	kd[roll_i] = 1;
	kd[pitch_i] = 1;
	kd[yaw_rate_i] = 1;
	
	ki[roll_i] = 1;
	ki[pitch_i] = 1;
	ki[yaw_rate_i] = 1;
	
	pid_max[roll_i] = 400;
	pid_max[pitch_i] = 400;
	pid_max[yaw_rate_i] = 400;
	
	pid_min[roll_i] = -400;
	pid_min[pitch_i] = -400;
	pid_min[yaw_rate_i] = -400;
	
	for (int i = 0; i<4; i++)
	{
		pulse_width_max[i] = 2000;
		pulse_width_min[i] = 1000;
	}
	// Error Range
	error_min[roll_i] = -400;
	error_min[pitch_i] = -400;
	error_min[yaw_rate_i] = -400;
	
	error_max[roll_i] = 400;
	error_max[pitch_i] = 400;
	error_max[yaw_rate_i] = 400;
	
	prev_error[roll_i] = 0;
	prev_error[pitch_i] = 0;
	prev_error[yaw_rate_i] = 0;
	
	return true;
}

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
	// target[roll_i] = target_roll;
	// target[pitch_i] = target_pitch;
	// target[yaw_rate_i] = target_yaw_rate;
	// target[throttle_i] = channel_throttle->get_control_in();
	

	//// 1. Get AHRS reading (all angles in radians/seconds)
	Vector3f gyro_latest = ahrs.get_gyro_latest();
	current[roll_i] = gyro_latest.x; 		// ahrs.get_roll();
	current[pitch_i] = gyro_latest.y; 		// ahrs.get_pitch();
	current[yaw_rate_i] = gyro_latest.z; // absolute yaw using compass (not sure)
	current[yaw_rate_i] = ahrs.get_yaw_rate_earth(); 	// in rads/sec
	
	cur_roll = ahrs.get_roll();
	cur_pitch = ahrs.get_pitch();
	cur_yaw = ahrs.get_yaw();

	// printf("Target R: %f, P: %f, Yr: %f\n", target_roll, target_pitch, target_yaw_rate);
	printf("Rate R: %f, P: %f, Y: %f\n", ToDeg(gyro_latest.x), ToDeg(gyro_latest.y), ToDeg(gyro_latest.z) );
	printf("Position R: %f, P: %f, Y: %f\n", ToDeg(cur_roll), ToDeg(cur_pitch), ToDeg(cur_yaw) );
	// printf("Curren R: %f, P: %f, Yr: %f, Y: %f\n", ToDeg(cur_roll), ToDeg(cur_pitch), ToDeg(cur_yaw_rate), cur_yaw);
	printf("Channels: %d %d\n", SRV_Channel::k_motor1, SRV_Channels::channel_function(1));
	
	
	//// 2. Error Roll Pitch Yaw
	error[roll_i]     = target[roll_i]     - current[roll_i];
	error[pitch_i]    = target[pitch_i]    - current[pitch_i];
	error[yaw_rate_i] = target[yaw_rate_i] - current[yaw_rate_i];
	error[throttle_i] = target[throttle_i] - current[throttle_i];
	//// Error Sum
	error_sum[roll_i]     += error[roll_i];
	error_sum[pitch_i]    += error[pitch_i];
	error_sum[yaw_rate_i] += error[yaw_rate_i];
	error_sum[throttle_i] += error[throttle_i];
	//// Limit err_correction
	if(error_sum[pitch_i] < (error_min[pitch_i]/ki[pitch_i]) )
	{	error_sum[pitch_i] = error_min[pitch_i]/ki[pitch_i];	}
	if(error_sum[pitch_i] > (error_max[pitch_i]/ki[pitch_i]))
	{	error_sum[pitch_i] = error_max[pitch_i]/ki[pitch_i];	}
	if(error_sum[roll_i] < (error_min[roll_i]/ki[roll_i]))
	{	error_sum[roll_i] = error_min[roll_i]/ki[roll_i];	}
	if(error_sum[roll_i] > (error_max[roll_i]/ki[roll_i]))
	{	error_sum[roll_i] = error_max[roll_i]/ki[roll_i];	}
	///// Delta Error
	delta_err[yaw_rate_i] = error[yaw_rate_i] - prev_error[yaw_rate_i];
	delta_err[pitch_i] = error[pitch_i] - prev_error[pitch_i];
	delta_err[roll_i] = error[roll_i] - prev_error[roll_i];
	
	prev_error[yaw_rate_i] = error[yaw_rate_i];
	prev_error[pitch_i] = error[pitch_i];
	prev_error[roll_i] = error[roll_i];
	
	
	
	// // Vector3f attitude_target_euler_angle;
	
	// // attitude_target_euler_angle.x = euler_roll_angle;
	// // attitude_target_euler_angle.y = euler_pitch_angle;
	// // attitude_target_euler_angle.z += euler_yaw_rate * _dt;
	// // float dt = 1/400;
	// // Quaternion attitude_target_quat;
	// // attitude_target_quat.from_euler(t_roll, t_pitch, t_yaw_rate * dt);
	// // attitude_target_quat.from_euler(_attitude_target_euler_angle.x, _attitude_target_euler_angle.y, _attitude_target_euler_angle.z);
	
	
	//// 3. Apply Controller loop on Error
	pid[yaw_rate_i] = (error[yaw_rate_i]*kp[yaw_rate_i]) + (error_sum[yaw_rate_i]*ki[yaw_rate_i]) + (delta_err[yaw_rate_i]*kd[yaw_rate_i]);
	pid[pitch_i] = (error[pitch_i]*kp[pitch_i]) + (error_sum[pitch_i]*ki[pitch_i]) + (delta_err[pitch_i]*kd[pitch_i]);
	pid[roll_i] = (error[roll_i]*kp[roll_i]) + (error_sum[roll_i]*ki[roll_i]) + (delta_err[roll_i]*kd[roll_i]);
		
	if(pid[yaw_rate_i] > pid_max[yaw_rate_i])
	{	pid[yaw_rate_i] = pid_max[yaw_rate_i];	}
	if(pid[yaw_rate_i] < pid_min[yaw_rate_i])
	{	pid[yaw_rate_i] = pid_min[yaw_rate_i];	}
	
	if(pid[pitch_i] > pid_max[pitch_i])
	{	pid[pitch_i] = pid_max[pitch_i];	}
	if(pid[pitch_i] < pid_min[pitch_i])
	{	pid[pitch_i] = pid_min[pitch_i];	}
	
	if(pid[roll_i] > pid_max[roll_i])
	{	pid[roll_i] = pid_max[roll_i];	}
	if(pid[roll_i] < pid_min[roll_i])
	{	pid[roll_i] = pid_min[roll_i];	}
	
	pulse_width[A] = current[throttle_i] - pid[roll_i]-pid[pitch_i]+pid[yaw_rate_i];		// Front Right	()	
	pulse_width[B] = current[throttle_i] + pid[roll_i]-pid[pitch_i]-pid[yaw_rate_i];		// Front Left
	pulse_width[C] = current[throttle_i] - pid[roll_i]+pid[pitch_i]-pid[yaw_rate_i];		// Back Right
	pulse_width[D] = current[throttle_i] + pid[roll_i]+pid[pitch_i]+pid[yaw_rate_i];		// Back Left

	if(pulse_width[A] > pulse_width_max[A])
	{	pulse_width[A] = pulse_width_max[A];	}
	if(pulse_width[A] < pulse_width_min[A])
	{	pulse_width[A] = pulse_width_min[A];	}
		
	if(pulse_width[B] > pulse_width_max[B])
	{	pulse_width[B] = pulse_width_max[B];	}
	if(pulse_width[B] < pulse_width_min[B])
	{	pulse_width[B] = pulse_width_min[B];	}

	if(pulse_width[C] > pulse_width_max[C])
	{	pulse_width[C] = pulse_width_max[C];	}
	if(pulse_width[C] < pulse_width_min[C])
	{	pulse_width[C] = pulse_width_min[C];	}
		
	if(pulse_width[D] > pulse_width_max[D])
	{	pulse_width[D] = pulse_width_max[D];	}
	if(pulse_width[D] < pulse_width_min[D])
	{	pulse_width[D] = pulse_width_min[D];	}
		
	//// 4. Output to motors
	SRV_Channels::set_output_pwm_chan(0, 1600); // uint16_t val		// Motor Pos: Back Right(Anticlockwise Roll)
	SRV_Channels::set_output_pwm_chan(1, 1610); // uint16_t val 	// Motor Pos: Back Left (Clockwise Roll)
	SRV_Channels::set_output_pwm_chan(2, 1600); // uint16_t val		// Motor Pos: Front --Left (Pitch)
	SRV_Channels::set_output_pwm_chan(3, 1600); // uint16_t val		// Motor Pos: Back Left
    SRV_Channels::cork();				// cork now, so that all channel outputs happen at once
    SRV_Channels::output_ch_all();		// update output on any aux channels, for manual passthru
    SRV_Channels::push();				// push all channels	
}