#include "Copter.h"

#include <AP_Motors/AP_MotorsMulticopter.h>
#include <AP_Motors/AP_Motors.h>
#include <stdio.h> // For printf statements
// #include <fstream>
// #include <iostream>

// COMMANDS: To get roll pitch yaw, throttle
// channel_roll->get_control_in();   channel_pitch->get_control_in();   channel_yaw->get_control_in();  channel_throttle->get_control_in();   get_pilot_desired_throttle()

// COMMANDS: To set roll pitch
// channel_roll->set_control_in(<float value>);   channel_pitch->set_control_in(<float value>);   channel_yaw->set_control_in(<float value>);
	
//const AP_InertialNav&       _inav;
//const AP_InertialNav&       inav;
//_inav(inav);


/*
./Tools/autotest/autotest.py build.ArduCopter fly.ArduCopter --map --viewerip=192.168.184.1./Tools/autotest/autotest.py build.ArduCopter fly.ArduCopter --map --viewerip=127.0.0.1
*/
FILE *fptr;
// ofstream outfile;

// REMOVE LATER
int HOVER_THROTTLE_OFFSET = 100;
int roll_i = 0, pitch_i = 1, yaw_i = 2, throttle_i = 3;
int roll_rate_i = 4, pitch_rate_i = 5, yaw_rate_i = 6;
int A = 0, B = 1, C = 2, D = 3;
float target[7];
float current[7]; // , current_rate[7];
float pulse_width[4];
float pulse_width_max[4];
float pulse_width_min[4];
float error[7];
float error_sum[7];
float error_min[7];
float error_max[7];
float delta_err[7], prev_error[7];
float pid[7];
float pid_max[7];
float pid_min[7];

// PID
float kp[7], kp_r[4];
float ki[7], kd_r[4];
float kd[7], ki_r[4];

float target_roll, target_pitch, target_yaw_rate, target_throttle;
float cur_roll, cur_pitch, cur_yaw, cur_yaw_rate, cur_throt;
float err_roll, err_pitch, err_yaw, err_yaw_rate, err_throt;

// Throttle
// Throttle constants set
float THR_ALT_P = 1.0f;
float RATE_THR_P = 1.0f;
float RATE_THR_I = 0.0f;
float RATE_THR_D = 0.0f;
float ACCEL_THR_P = 1.0f;
float ACCEL_THR_I = 0.0f;
float ACCEL_THR_D = 0.0f;

float accel_lim = 250.0f;
float pid_accel_max = accel_lim;
float pid_accel_min = -accel_lim;
float _dt = 0.01;

float throttle_in = 0.0;
float pid_accel = 0.0;
float vel_max = 250;
float vel_min = -250;
//pos_control->set_alt_target(target_z); // Line not able to call
// Altitude target set
float target_z = 3000.0f; //in cm according to inbuilt code 

float curr_alt = 0.0;
float error_z = 0.0;

float vel_target = 0.0;

float error_vel = 0.0;
float error_sum_vel = 0.0;
float delta_err_vel = 0.0;
float prev_vel = 0.0;
float pid_vel = 0.0;

float accel_target = 0.0;
float accel_cur = 0.0;
float error_accel = 0.0;
float error_sum_accel = 0.0;
float delta_err_accel = 0.0;
float prev_accel = 0.0;

float thr_out = 0.0;
float accel_gnd = 0;

int loop = 0;
/*
 * Init and run calls for stabilize flight mode
 */
bool ModeNewControl::init(bool ignore_checks)
{
	loop = 0;
	// outfile.open("custom.log");	
	fptr = fopen("throttle.txt","w"); // change
	// current[throttle_i] = 1500;
	// return true;

	// PID Parameters
	kp[roll_i] = 0.005;//0.005
	kp[pitch_i] = 0.005;//0.005
	kp[yaw_i] = 0.001;//0.001
	kp[roll_rate_i] = 0;
	kp[pitch_rate_i] = 0;
	kp[yaw_rate_i] = 0;
	
	kd[roll_i] = 1.05;//1.05
	kd[pitch_i] = 1.01;//1.01
	kd[yaw_i] = 1.01; // 1.01
	kd[roll_rate_i] = 0;
	kd[pitch_rate_i] = 0;
	kd[yaw_rate_i] = 0;
	
	ki[roll_i] = 0;
	ki[pitch_i] = 0;
	ki[yaw_rate_i] = 0;
	ki[roll_rate_i] = 0;
	ki[pitch_rate_i] = 0;
	ki[yaw_rate_i] = 0;
	
	// Write to file PID parameters
	// fprintf(fptr,"Kp: %f Kd: %f Ki: %f\n", kp[pitch_i], kd[pitch_i], ki[pitch_i]); // change

	// PID Variables threshold
	for (int i = 0; i < 7; i ++ ){
		pid_max[i] = 400;
		pid_min[i] = -400;
		error_min[i] = -400;
		error_max[i] = 400;
		prev_error[i] = 0;
	}

	// PWM Range
	for (int i = 0; i<4; i++)
	{
		pulse_width_max[i] = 2000;
		pulse_width_min[i] = 1000;
	}


	for (int i = 0; i<7; i++)
	{
		target[i] = 0;
		pid[i] = 0;
	}

	current[throttle_i] = 1000; // 1800
	
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
	
	// // Throttle from remote control
	// target_throttle = 1000 + channel_throttle->get_control_in() + HOVER_THROTTLE_OFFSET; // For getting throttle from remote control

	// Throttle from Altitude Control
	get_custom_throttle();	// Get Final throttle PWM value in target_throttle
	// if (loop % 100 == 0){
	// 	printf("Run Working %d\n", loop);
	// }
	PID_motors();
}

// RANGES:
// target_yaw_rate -20,250 - 20,250
// target_roll -3,000 - 3,000
// target_pitch -3,000 - 3,000
// throttle - 0-1	
// channel_throttle->get_control_in() - 0 - 1000 479

void ModeNewControl::PID_motors()
{
	loop += 1;	// loop counter for print statements
	if (loop % 100 == 0){
		printf("PID Motors Loop Count %d\n", loop);
	}

	//// 1. Get AHRS reading (all angles in radians/seconds)
	Vector3f gyro_latest = ahrs.get_gyro_latest();
	current[roll_rate_i] = ToDeg(gyro_latest.x); 		// ahrs.get_roll();
	current[pitch_rate_i] = ToDeg(gyro_latest.y); 		// ahrs.get_pitch();
	current[yaw_rate_i] = ToDeg(gyro_latest.z); // absolute yaw using compass (not sure)
	// current_rate[yaw_rate_i] = ahrs.get_yaw_rate_earth(); 	// in rads/sec
	current[roll_i] = ToDeg(ahrs.get_roll());
	current[pitch_i] = ToDeg(ahrs.get_pitch());
	current[yaw_i] = ToDeg(ahrs.get_yaw());

	//// 1.2. Get Target Attitude (from remote) (COMMENT WHEN NOT USING REMOTE AND USE ALTERNATE)
	// target[roll_i] = target_roll / 100;
	// target[pitch_i] = target_pitch / 100;
	// target[yaw_rate_i] = target_yaw_rate / 1000;
	// target[throttle_i] = target_throttle;
	// target[yaw_i] = current[yaw_i] + target_yaw_rate / 400;
	
	//// 1.2. ALTERNATE Auto Set Target
	target[throttle_i] = target_throttle;
	if (loop <= 5000){
		target[roll_i] = 0;
		target[pitch_i] = 0;
		target[yaw_i] = 0;
	}
	if(loop > 5000){
	
	}
	if (loop > 10000){
		target[pitch_i] = 20; // change
		target[roll_i] = 0;
	}
	if(loop > 11000){
		target[yaw_i] = 0; 
	}
	if(loop > 12000){
		target[pitch_i] = 0;
	}


	current[throttle_i] = target[throttle_i];	// This value used directly in PWM throttle output
	
	//// 2. Error Roll Pitch Yaw
	error[roll_i]     = target[roll_i]     - current[roll_i];
	error[pitch_i]    = target[pitch_i]    - current[pitch_i];
	error[yaw_i]      = target[yaw_i]      - current[yaw_i];

	error[yaw_rate_i] = target[yaw_rate_i] - current[yaw_rate_i];
	error[throttle_i] = target[throttle_i] - current[throttle_i];
	
	//// Error Sum
	error_sum[roll_i]     += error[roll_i];
	error_sum[pitch_i]    += error[pitch_i];
	error_sum[yaw_i]    += error[yaw_i];

	error_sum[yaw_rate_i] += error[yaw_rate_i];
	error_sum[throttle_i] += error[throttle_i];

	//// Limit err_correction
	if (abs(ki[pitch_i]) > 0.0000001)
	{ 
		if(error_sum[pitch_i] < (error_min[pitch_i]/ki[pitch_i]) )
		{	error_sum[pitch_i] = error_min[pitch_i]/ki[pitch_i];	}
		if(error_sum[pitch_i] > (error_max[pitch_i]/ki[pitch_i]))
		{	error_sum[pitch_i] = error_max[pitch_i]/ki[pitch_i];	}
	}else{
		// error_sum[pitch_i] = 0;
	}
	if (abs(ki[roll_i]) > 0.0000001)
	{
		if(error_sum[roll_i] < (error_min[roll_i]/ki[roll_i]))
		{	error_sum[roll_i] = error_min[roll_i]/ki[roll_i];	}
		if(error_sum[roll_i] > (error_max[roll_i]/ki[roll_i]))
		{	error_sum[roll_i] = error_max[roll_i]/ki[roll_i];	}
	}else{
		// error_sum[roll_i] = 0;
	}
	if (abs(ki[yaw_i]) > 0.0000001)
	{
		if(error_sum[yaw_i] < (error_min[yaw_i]/ki[yaw_i]))
		{	error_sum[yaw_i] = error_min[yaw_i]/ki[yaw_i];	}
		if(error_sum[yaw_i] > (error_max[yaw_i]/ki[yaw_i]))
		{	error_sum[yaw_i] = error_max[yaw_i]/ki[yaw_i];	}
	}else{
		// error_sum[yaw_i] = 0;
	}
	///// Delta Error
	delta_err[yaw_i] = error[yaw_i] - prev_error[yaw_i];
	delta_err[pitch_i] = error[pitch_i] - prev_error[pitch_i];
	delta_err[roll_i] = error[roll_i] - prev_error[roll_i];
	
	prev_error[yaw_i] = error[yaw_i];
	prev_error[pitch_i] = error[pitch_i];
	prev_error[roll_i] = error[roll_i];
	//-printf("Del Er   R: %f, P: %f, Y: %f\n", delta_err[roll_i], delta_err[pitch_i], delta_err[yaw_i]);
	
	
	// // // Vector3f attitude_target_euler_angle;
	
	// // // attitude_target_euler_angle.x = euler_roll_angle;
	// // // attitude_target_euler_angle.y = euler_pitch_angle;
	// // // attitude_target_euler_angle.z += euler_yaw_rate * _dt;
	// // // float dt = 1/400;
	// // // Quaternion attitude_target_quat;
	// // // attitude_target_quat.from_euler(t_roll, t_pitch, t_yaw_rate * dt);
	// // // attitude_target_quat.from_euler(_attitude_target_euler_angle.x, _attitude_target_euler_angle.y, _attitude_target_euler_angle.z);
	
	
	//// 3. Apply Controller loop on Error
	pid[yaw_i] = (error[yaw_i]*kp[yaw_i]) + (error_sum[yaw_i]*ki[yaw_i]) + (delta_err[yaw_i]*kd[yaw_i]);
	pid[pitch_i] = (error[pitch_i]*kp[pitch_i]) + (error_sum[pitch_i]*ki[pitch_i]) + (delta_err[pitch_i]*kd[pitch_i]);
	pid[roll_i] = (error[roll_i]*kp[roll_i]) + (error_sum[roll_i]*ki[roll_i]) + (delta_err[roll_i]*kd[roll_i]);
	//-printf("PID debug   A: %f, B: %f\n", error[roll_i], kp[roll_i]);

	//-printf("PID fb   R: %f, P: %f, Y: %f\n", pid[roll_i], pid[pitch_i], pid[yaw_i]);

	if(pid[yaw_i] > pid_max[yaw_i])
	{	pid[yaw_i] = pid_max[yaw_i];	}
	if(pid[yaw_i] < pid_min[yaw_i])
	{	pid[yaw_i] = pid_min[yaw_i];	}
	
	if(pid[pitch_i] > pid_max[pitch_i])
	{	pid[pitch_i] = pid_max[pitch_i];	}
	if(pid[pitch_i] < pid_min[pitch_i])
	{	pid[pitch_i] = pid_min[pitch_i];	}
	
	if(pid[roll_i] > pid_max[roll_i])
	{	pid[roll_i] = pid_max[roll_i];	}
	if(pid[roll_i] < pid_min[roll_i])
	{	pid[roll_i] = pid_min[roll_i];	}
	
	//// X Frame
	// pulse_width[A] = current[throttle_i] - pid[roll_i]-pid[pitch_i]+pid[yaw_rate_i];		// Front Right	()	
	// pulse_width[B] = current[throttle_i] + pid[roll_i]-pid[pitch_i]-pid[yaw_rate_i];		// Front Left
	// pulse_width[C] = current[throttle_i] - pid[roll_i]+pid[pitch_i]-pid[yaw_rate_i];		// Back Right
	// pulse_width[D] = current[throttle_i] + pid[roll_i]+pid[pitch_i]+pid[yaw_rate_i];		// Back Left

	//// Plus Frame
	pulse_width[A] = current[throttle_i] - pid[roll_i] + pid[yaw_i];		// Right	()	
	pulse_width[B] = current[throttle_i] + pid[roll_i] + pid[yaw_i];		// Left
	pulse_width[C] = current[throttle_i] + pid[pitch_i] - pid[yaw_i];		// Front
	pulse_width[D] = current[throttle_i] - pid[pitch_i] - pid[yaw_i];		// Back	

	//printf("PID Roll i %f, %f\n",pid[roll_i],error[roll_i]);

	// Limit PWM within range
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
	
	//-printf("PWM R: %f, L: %f, F: %f, B: %f\n", pulse_width[A], pulse_width[B], pulse_width[C], pulse_width[D] );
	//-printf("\n");

	//// 4. Output to motors
	SRV_Channels::set_output_pwm_chan(0, pulse_width[A]); // uint16_t val		// Motor Pos: Right
	SRV_Channels::set_output_pwm_chan(1, pulse_width[B]); // uint16_t val 	// Motor Pos: Left
	SRV_Channels::set_output_pwm_chan(2, pulse_width[C]); // uint16_t val		// Motor Pos: Front
	SRV_Channels::set_output_pwm_chan(3, pulse_width[D]); // uint16_t val		// Motor Pos: Back
	
	SRV_Channels::cork();				// cork now, so that all channel outputs happen at once
	SRV_Channels::output_ch_all();		// update output on any aux channels, for manual passthru
	SRV_Channels::push();				// push all channels	

}

void ModeNewControl::get_custom_throttle(){
	loop += 1;
	// ABHAY
	accel_gnd = -(ahrs.get_accel_ef_blended().z + GRAVITY_MSS)*100;
	curr_alt = inertial_nav.get_altitude();
	const Vector3f&  vel_current = inertial_nav.get_velocity() ;
	vel_target = AC_AttitudeControl::sqrt_controller(target_z - inertial_nav.get_altitude(), THR_ALT_P, accel_lim, _dt);
	if (vel_target < vel_min) {
        vel_target = vel_min;
        // _limit.vel_down = true;
    }
    if (vel_target > vel_max) {
        vel_target = vel_max;
        // _limit.vel_up = true;
    }
	accel_target = ((vel_target - vel_current.z)*RATE_THR_P);
	pid_accel = ((accel_target - accel_gnd)*ACCEL_THR_P);//(error_sum_accel*ACCEL_THR_I*_dt) + ((delta_err_accel*ACCEL_THR_D)/_dt);
	thr_out =  motors->get_throttle_hover() + pid_accel/250;
	if(thr_out>1)
	{	thr_out = 1;	}
	if(thr_out<0)
	{	thr_out = 0;	}

	float thr = 1000.0 + thr_out*(2000.0 - 1000.0);	
	target_throttle = thr;
	if(loop%100 == 0)
	{
		// printf("Current Altitude %f", curr_alt);
		// printf("| Current Acceleration %f", accel_gnd);
		// printf("| Current velocity %f",vel_current.z);
		// printf("| Velocity target %f", vel_target);
		// printf("| Acceleration Target %f", accel_target);
		// printf("| PID accel %f", pid_accel);	
		// printf("| throttle out %f", thr_out);
		// printf("| Current Throttle %f\n", target_throttle );
	}

	fprintf(fptr,"Target: %f Current: %f\n", target_z, curr_alt); // change

	// ABHAY
}