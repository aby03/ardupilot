#include "Copter.h"

#include <AP_Math/AP_Math.h>

#include <AP_Motors/AP_MotorsMulticopter.h>
#include <AP_Motors/AP_Motors.h>

// COMMANDS: To get roll pitch yaw, throttle
// channel_roll->get_control_in();   channel_pitch->get_control_in();   channel_yaw->get_control_in();  channel_throttle->get_control_in();   get_pilot_desired_throttle()
// COMMANDS: To set roll pitch
// channel_roll->set_control_in(<float value>);   channel_pitch->set_control_in(<float value>);   channel_yaw->set_control_in(<float value>);


/*
./Tools/autotest/autotest.py build.ArduCopter fly.ArduCopter --map --viewerip=192.168.184.1./Tools/autotest/autotest.py build.ArduCopter fly.ArduCopter --map --viewerip=127.0.0.1
*/
FILE *fptr;	// To open file and write it for plotting

// Attitude Control
int HOVER_THROTTLE_OFFSET = 100;
int roll_i = 0, pitch_i = 1, yaw_i = 2, throttle_i = 3;
int roll_rate_i = 4, pitch_rate_i = 5, yaw_rate_i = 6;
int A = 0, B = 1, C = 2, D = 3;
float target[7];
float current[7]; // , current_rate[7];
float pulse_width[4];
float pulse_width_max[4];
float pulse_width_min[4];
float error_rpyt[7];
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
float target_z = 0.0f; //in cm according to inbuilt code 

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

// Pos Control
// parameters
float    _wp_speed_cms = 500.0f;          // default maximum horizontal speed in cm/s during missions
float    _wp_speed_up_cms = 250.0f;       // default maximum climb rate in cm/s
float    _wp_speed_down_cms = 150.0f;     // default maximum descent rate in cm/s
float    _wp_radius_cm = 200.0f;          // distance from a waypoint in cm that, when crossed, indicates the wp has been reached
float    _wp_accel_cmss = 100.0f;          // horizontal acceleration in cm/s/s during missions
float    _wp_accel_z_cmss = 100.0f;        // vertical acceleration in cm/s/s during missions

Vector3f    _pos_target;            // target location in cm from home
Vector3f    _pos_error;             // error between desired and actual position in cm
Vector3f    _vel_desired;           // desired velocity in cm/s
Vector2f    _vel_error;             // error between desired and actual acceleration in cm/s
Vector2f    _prev_error_vel;             // error between desired and actual acceleration in cm/s
Vector3f    _vel_target;            // velocity target in cm/s calculated by pos_to_rate step
Vector2f    _vehicle_horiz_vel;     // velocity to use if _flags.vehicle_horiz_vel_override is set
Vector3f    _accel_desired;         // desired acceleration in cm/s/s (feed forward)
Vector3f    _accel_target;          // acceleration target in cm/s/s

enum SegmentType {
	SEGMENT_STRAIGHT = 0,
	SEGMENT_SPLINE = 1
};
// flags structure
struct wpnav_flags {
	uint8_t reached_destination     : 1;    // true if we have reached the destination
	uint8_t fast_waypoint           : 1;    // true if we should ignore the waypoint radius and consider the waypoint complete once the intermediate target has reached the waypoint
	uint8_t slowing_down            : 1;    // true when target point is slowing down before reaching the destination
	uint8_t recalc_wp_leash         : 1;    // true if we need to recalculate the leash lengths because of changes in speed or acceleration
	uint8_t new_wp_destination      : 1;    // true if we have just received a new destination.  allows us to freeze the position controller's xy feed forward
	SegmentType segment_type        : 1;    // active segment is either straight or spline
	uint8_t wp_yaw_set              : 1;    // true if yaw target has been set
} _flags;


// Gains
# define POSCONTROL_ACCEL_XY 				   100.0f
# define POSCONTROL_LEASH_LENGTH_MIN 		   100.0f
# define POSCONTROL_ACCEL_FILTER_HZ 		   2.0f
# define POSCONTROL_POS_XY_P                   1.0f    // horizontal position controller P gain default // 1.0f
# define POSCONTROL_VEL_XY_P                   2.0f    // horizontal velocity controller P gain default // 2.0f
# define POSCONTROL_VEL_XY_I                   1.0f    // horizontal velocity controller I gain default // 1.0f
# define POSCONTROL_VEL_XY_D                   0.5f  //0.5f    // horizontal velocity controller D gain default
# define POSCONTROL_VEL_XY_IMAX                1000.0f // horizontal velocity controller IMAX gain default
# define POSCONTROL_VEL_XY_FILT_HZ             10.0f    // horizontal velocity controller input filter	// 5.0f
# define POSCONTROL_VEL_XY_FILT_D_HZ           10.0f    // horizontal velocity controller input filter for D // 5.0f
// # define POSCONTROL_DT_50HZ					   0.01f  // Default: 0.02f

AC_PID_2D _pid_vel_xy = AC_PID_2D(POSCONTROL_VEL_XY_P, POSCONTROL_VEL_XY_I, POSCONTROL_VEL_XY_D, POSCONTROL_VEL_XY_IMAX, POSCONTROL_VEL_XY_FILT_HZ, POSCONTROL_VEL_XY_FILT_D_HZ, POSCONTROL_DT_50HZ);


// Debug
int loop = 0;
Vector2f print_accel;

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
	start_custom_pos();
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
    // get_pilot_desired_lean_angles(target_roll, target_pitch, copter.aparm.angle_max, copter.aparm.angle_max);

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

	Vector3f test_pos = inertial_nav.get_position();
	loop += 1;
	if (loop % 100 == 0){
		printf("Loop: %d X: %f Y: %f Z: %f Ax: %f Ay: %f\n", loop, test_pos.x, test_pos.y, test_pos.z, print_accel.x, print_accel.y);
		// printf("Loop: %d\n", loop);
	}
	if (loop <= 5000){
		target_z = 3000.0f;
	}
	if(loop == 20000){
		_pos_target.x = 10000.0f;
		_pos_target.y = 10000.0f;
	}

	// roll & pitch from pos_control
	// run_custom_pos();
	// Throttle from Altitude Control
	// get_custom_throttle();	// Get Final throttle PWM value in target_throttle
	// Attitude Control & Write PWM to motors
	// PID_motors();
}

// RANGES:
// target_yaw_rate -20,250 - 20,250
// target_roll -3,000 - 3,000
// target_pitch -3,000 - 3,000
// throttle - 0-1	
// channel_throttle->get_control_in() - 0 - 1000 479

void ModeNewControl::PID_motors()
{
	// loop += 1;	// loop counter for print statements
	// if (loop % 100 == 0){
	// 	printf("PID Motors Loop Count %d\n", loop);
	// }

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
	target[roll_i] = target_roll / 100;
	target[pitch_i] = target_pitch / 100;
	// target[yaw_rate_i] = target_yaw_rate / 1000;
	target[throttle_i] = target_throttle;
	// target[yaw_i] = current[yaw_i] + target_yaw_rate / 400;
	
	//// 1.2. ALTERNATE Auto Set Target
	// // target[throttle_i] = target_throttle;
	// if (loop <= 5000){
	// 	target[roll_i] = 0;
	// 	target[pitch_i] = 0;
	// 	target[yaw_i] = 0;
	// }
	// if(loop > 5000){
	
	// }
	// if (loop > 10000){
	// 	target[pitch_i] = 20; // change
	// 	target[roll_i] = 0;
	// }
	// if(loop > 11000){
	// 	target[yaw_i] = 0; 
	// }
	// if(loop > 20000){
	// 	target[pitch_i] = 0;
	// }


	current[throttle_i] = target[throttle_i];	// This value used directly in PWM throttle output
	
	//// 2. Error Roll Pitch Yaw
	error_rpyt[roll_i]     = target[roll_i]     - current[roll_i];
	error_rpyt[pitch_i]    = target[pitch_i]    - current[pitch_i];
	error_rpyt[yaw_i]      = target[yaw_i]      - current[yaw_i];

	error_rpyt[yaw_rate_i] = target[yaw_rate_i] - current[yaw_rate_i];
	error_rpyt[throttle_i] = target[throttle_i] - current[throttle_i];
	
	//// Error Sum
	error_sum[roll_i]     += error_rpyt[roll_i];
	error_sum[pitch_i]    += error_rpyt[pitch_i];
	error_sum[yaw_i]    += error_rpyt[yaw_i];

	error_sum[yaw_rate_i] += error_rpyt[yaw_rate_i];
	error_sum[throttle_i] += error_rpyt[throttle_i];

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
	delta_err[yaw_i] = error_rpyt[yaw_i] - prev_error[yaw_i];
	delta_err[pitch_i] = error_rpyt[pitch_i] - prev_error[pitch_i];
	delta_err[roll_i] = error_rpyt[roll_i] - prev_error[roll_i];
	
	prev_error[yaw_i] = error_rpyt[yaw_i];
	prev_error[pitch_i] = error_rpyt[pitch_i];
	prev_error[roll_i] = error_rpyt[roll_i];
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
	pid[yaw_i] = (error_rpyt[yaw_i]*kp[yaw_i]) + (error_sum[yaw_i]*ki[yaw_i]) + (delta_err[yaw_i]*kd[yaw_i]);
	pid[pitch_i] = (error_rpyt[pitch_i]*kp[pitch_i]) + (error_sum[pitch_i]*ki[pitch_i]) + (delta_err[pitch_i]*kd[pitch_i]);
	pid[roll_i] = (error_rpyt[roll_i]*kp[roll_i]) + (error_sum[roll_i]*ki[roll_i]) + (delta_err[roll_i]*kd[roll_i]);
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

	//printf("PID Roll i %f, %f\n",pid[roll_i],error_rpyt[roll_i]);

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

void ModeNewControl::start_custom_pos(){
    // check _wp_accel_cmss is reasonable
    // if (_wp_accel_cmss <= 0) {
    //     _wp_accel_cmss.set_and_save(WPNAV_ACCELERATION);
    // }

    // // initialise position controller
    // pos_control->set_desired_accel_xy(0.0f,0.0f);
    // pos_control->init_xy_controller();
    // pos_control->clear_desired_velocity_ff_z();

    // // initialise feed forward velocity to zero
    // pos_control->set_desired_velocity_xy(0.0f, 0.0f);

    // // initialise position controller speed and acceleration
    // pos_control->set_max_speed_xy(_wp_speed_cms);
    // pos_control->set_max_accel_xy(_wp_accel_cmss);
    // pos_control->set_max_speed_z(-_wp_speed_down_cms, _wp_speed_up_cms);
    // pos_control->set_max_accel_z(_wp_accel_z_cmss);
    // pos_control->calc_leash_length_xy();
    // pos_control->calc_leash_length_z();

    // // initialise yaw heading to current heading target
    // _flags.wp_yaw_set = false;

	// // ==

	// // Stopping Point
	// Vector3f stopping_point;
	// pos_control->get_stopping_point_xy(stopping_point);
    // pos_control->get_stopping_point_z(stopping_point);
	// _pid_vel_xy = AC_PID_2D(POSCONTROL_VEL_XY_P, POSCONTROL_VEL_XY_I, POSCONTROL_VEL_XY_D, POSCONTROL_VEL_XY_IMAX, POSCONTROL_VEL_XY_FILT_HZ, POSCONTROL_VEL_XY_FILT_D_HZ, POSCONTROL_DT_50HZ);

	_pid_vel_xy.reset_filter();
}


// limit flags structure
struct poscontrol_limit_flags {
	uint8_t pos_up      : 1;    // 1 if we have hit the vertical position leash limit while going up
	uint8_t pos_down    : 1;    // 1 if we have hit the vertical position leash limit while going down
	uint8_t vel_up      : 1;    // 1 if we have hit the vertical velocity limit going up
	uint8_t vel_down    : 1;    // 1 if we have hit the vertical velocity limit going down
	uint8_t accel_xy    : 1;    // 1 if we have hit the horizontal accel limit
} _limit;
// position controller internal variables
// Parameters
LowPassFilterVector2f _accel_target_filter; // acceleration target filter


/// run horizontal position controller correcting position and velocity
///     converts position (_pos_target) to target velocity (_vel_target)
///     desired velocity (_vel_desired) is combined into final target velocity
///     converts desired velocities in lat/lon directions to accelerations in lat/lon frame
///     converts desired accelerations provided in lat/lon frame to roll/pitch angles
void ModeNewControl::run_custom_pos(){
	// Get current position
    Vector3f curr_pos = inertial_nav.get_position();
    float kP = POSCONTROL_POS_XY_P; // scale gains to compensate for noisy optical flow measurement in the EKF

    // avoid divide by zero
    if (kP <= 0.0f) {
        _vel_target.x = 0.0f;
        _vel_target.y = 0.0f;
    } else {
        // calculate distance error
        _pos_error.x = _pos_target.x - curr_pos.x;
        _pos_error.y = _pos_target.y - curr_pos.y;

        _vel_target = sqrt_controller(_pos_error, kP, POSCONTROL_ACCEL_XY);
    }


    // the following section converts desired velocities in lat/lon directions to accelerations in lat/lon frame
    Vector2f accel_target_pos, vel_xy_p, vel_xy_i, vel_xy_d;

    // Current Velocity
	_vehicle_horiz_vel.x = inertial_nav.get_velocity().x;
	_vehicle_horiz_vel.y = inertial_nav.get_velocity().y;

    // calculate velocity error
    _vel_error.x = _vel_target.x - _vehicle_horiz_vel.x;
    _vel_error.y = _vel_target.y - _vehicle_horiz_vel.y;
    // TODO: constrain velocity error and velocity target

    // call pi controller
    // _pid_vel_xy.set_input(_vel_error);

    // get p
    vel_xy_p = _vel_error * POSCONTROL_VEL_XY_P;

    // update i term if we have not hit the accel or throttle limits OR the i term will reduce
    // TODO: move limit handling into the PI and PID controller
    // if (!_limit.accel_xy) {//abhay
    //     vel_xy_i = _pid_vel_xy.get_i();
    // } else {
    //     vel_xy_i = _pid_vel_xy.get_i_shrink();
    // }
	vel_xy_i.x = 0;
	vel_xy_i.y = 0;

    // get d
    vel_xy_d = (_prev_error_vel - _vel_error) * POSCONTROL_VEL_XY_D;
	_prev_error_vel = _vel_error;

    // acceleration to correct for velocity error and scale PID output to compensate for optical flow measurement induced EKF noise
    accel_target_pos.x = (vel_xy_p.x + vel_xy_i.x + vel_xy_d.x);//abhay
    accel_target_pos.y = (vel_xy_p.y + vel_xy_i.y + vel_xy_d.y);//abhay

    // // reset accel to current desired acceleration
    // if (_flags.reset_accel_to_lean_xy) {
    //     _accel_target_filter.reset(Vector2f(accel_target_pos.x, accel_target_pos.y));
    //     _flags.reset_accel_to_lean_xy = false;
    // }

    // filter correction acceleration
    // _accel_target_filter.set_cutoff_frequency(MIN(POSCONTROL_ACCEL_FILTER_HZ, 5.0f * ekfNavVelGainScaler)); //abhay
    // _accel_target_filter.apply(accel_target_pos, _dt);//abhay

    // pass the correction acceleration to the target acceleration output
    // _accel_target.x = _accel_target_filter.get().x;//abhay
    // _accel_target.y = _accel_target_filter.get().y;//abhay
	_accel_target.x = accel_target_pos.x;
	_accel_target.y = accel_target_pos.y;

    // Add feed forward into the target acceleration output
    // _accel_target.x += _accel_desired.x;//abhay
    // _accel_target.y += _accel_desired.y;//abhay

    // the following section converts desired accelerations provided in lat/lon frame to roll/pitch angles

    // limit acceleration using maximum lean angles
    float angle_max = 20.0f * 100.0f;
    float accel_max = MIN(GRAVITY_MSS * 100.0f * tanf(ToRad(angle_max * 0.01f)), POSCONTROL_ACCEL_XY_MAX);
    _limit.accel_xy = limit_vector_length(_accel_target.x, _accel_target.y, accel_max);

    // update angle targets that will be passed to stabilize controller
    accel_to_lean_angles(_accel_target.x, _accel_target.y, target_roll, target_pitch);
	// Debug
	print_accel.x = _accel_target.x;
	print_accel.y = _accel_target.y;
}

/// Proportional controller with piecewise sqrt sections to constrain second derivative
Vector3f ModeNewControl::sqrt_controller(const Vector3f& error, float p, float second_ord_lim)
{
    if (second_ord_lim < 0.0f || is_zero(second_ord_lim) || is_zero(p)) {
        return Vector3f(error.x * p, error.y * p, error.z);
    }

    float linear_dist = second_ord_lim / sq(p);
    float error_length = norm(error.x, error.y);
    if (error_length > linear_dist) {
        float first_order_scale = safe_sqrt(2.0f * second_ord_lim * (error_length - (linear_dist * 0.5f))) / error_length;
        return Vector3f(error.x * first_order_scale, error.y * first_order_scale, error.z);
    } else {
        return Vector3f(error.x * p, error.y * p, error.z);
    }
}

// get_lean_angles_to_accel - convert roll, pitch lean angles to lat/lon frame accelerations in cm/s/s
void ModeNewControl::accel_to_lean_angles(float accel_x_cmss, float accel_y_cmss, float& roll_target, float& pitch_target) const
{
    float accel_right, accel_forward;

    // rotate accelerations into body forward-right frame
    // todo: this should probably be based on the desired heading not the current heading
    accel_forward = accel_x_cmss * ahrs.cos_yaw() + accel_y_cmss * ahrs.sin_yaw();
    accel_right = -accel_x_cmss * ahrs.sin_yaw() + accel_y_cmss * ahrs.cos_yaw();

    // update angle targets that will be passed to stabilize controller
    pitch_target = atanf(-accel_forward / (GRAVITY_MSS * 100.0f)) * (18000.0f / M_PI);
    float cos_pitch_target = cosf(pitch_target * M_PI / 18000.0f);
    roll_target = atanf(accel_right * cos_pitch_target / (GRAVITY_MSS * 100.0f)) * (18000.0f / M_PI);
}

/// limit vector to a given length, returns true if vector was limited
bool ModeNewControl::limit_vector_length(float& vector_x, float& vector_y, float max_length)
{
    float vector_length = norm(vector_x, vector_y);
    if ((vector_length > max_length) && is_positive(vector_length)) {
        vector_x *= (max_length / vector_length);
        vector_y *= (max_length / vector_length);
        return true;
    }
    return false;
}