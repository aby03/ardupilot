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

// CONTROL PARAMETERS
# define POS_CONTROL_ENABLE 1
# define ALT_CONTROL_ENABLE 1

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
    // update_simple_mode();

    // convert pilot input to lean angles
	// from mode.cpp, returns lean angles in centi degrees
    // float target_roll, target_pitch;
    // get_pilot_desired_lean_angles(target_roll, target_pitch, copter.aparm.angle_max, copter.aparm.angle_max);

    // get pilot's desired yaw rate
	// from Attitude.cpp, returns yaw rate in centi-degrees per second
    // target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

    if (!motors->armed()) {
        // Motors should be Stopped
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
    } else if (copter.ap.throttle_zero) {
        // Attempting to Land
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
    } else {
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    }
	
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

}

// RANGES:
// target_yaw_rate -20,250 - 20,250
// target_roll -3,000 - 3,000
// target_pitch -3,000 - 3,000
// throttle - 0-1	
// channel_throttle->get_control_in() - 0 - 1000 479

void ModeNewControl::PID_motors()
{
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
	
	//// 1.2. ALTERNATE Auto Set Target (Only for debug) (add target based on loop count here)


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


	//// 3. Apply Controller loop on Error
	pid[yaw_i] = (error_rpyt[yaw_i]*kp[yaw_i]) + (error_sum[yaw_i]*ki[yaw_i]) + (delta_err[yaw_i]*kd[yaw_i]);
	pid[pitch_i] = (error_rpyt[pitch_i]*kp[pitch_i]) + (error_sum[pitch_i]*ki[pitch_i]) + (delta_err[pitch_i]*kd[pitch_i]);
	pid[roll_i] = (error_rpyt[roll_i]*kp[roll_i]) + (error_sum[roll_i]*ki[roll_i]) + (delta_err[roll_i]*kd[roll_i]);

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
	if (ALT_CONTROL_ENABLE){
		loop += 1;
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
	}
}

void ModeNewControl::start_custom_pos(){
	// For initialization if needed
}

/// run horizontal position controller correcting position and velocity
///     converts position (_pos_target) to target velocity (_vel_target)
///     desired velocity (_vel_desired) is combined into final target velocity
///     converts desired velocities in lat/lon directions to accelerations in lat/lon frame
///     converts desired accelerations provided in lat/lon frame to roll/pitch angles
void ModeNewControl::run_custom_pos(){
	if (POS_CONTROL_ENABLE){
		// Get current position
		Vector3f curr_pos = inertial_nav.get_position();

		float kP_pos = POSCONTROL_POS_XY_P; // Kp for P term on pos error

		// avoid divide by zero
		if (kP_pos <= 0.0f) {
			_vel_target.x = 0.0f;
			_vel_target.y = 0.0f;
		} else {
			// calculate distance error
			_pos_error.x = _pos_target.x - curr_pos.x;
			_pos_error.y = _pos_target.y - curr_pos.y;

			_vel_target = sqrt_controller(_pos_error, kP_pos, POSCONTROL_ACCEL_XY);
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

		// get p
		vel_xy_p = _vel_error * POSCONTROL_VEL_XY_P;

		// Not using I
		vel_xy_i.x = 0;
		vel_xy_i.y = 0;

		// get d
		vel_xy_d = (_prev_error_vel - _vel_error) * POSCONTROL_VEL_XY_D;
		_prev_error_vel = _vel_error;

		// acceleration to correct for velocity error and scale PID output to compensate for optical flow measurement induced EKF noise
		accel_target_pos.x = (vel_xy_p.x + vel_xy_i.x + vel_xy_d.x);//abhay
		accel_target_pos.y = (vel_xy_p.y + vel_xy_i.y + vel_xy_d.y);//abhay

		// Copy to accel target which will be limited based on max acc
		_accel_target.x = accel_target_pos.x;
		_accel_target.y = accel_target_pos.y;

		// the following section converts desired accelerations provided in lat/lon frame to roll/pitch angles
		// limit acceleration using maximum lean angles
		float angle_max = 20.0f * 100.0f;	// Max angle drone will tilt to go to position
		float accel_max = MIN(GRAVITY_MSS * 100.0f * tanf(ToRad(angle_max * 0.01f)), POSCONTROL_ACCEL_XY_MAX);
		limit_vector_length(_accel_target.x, _accel_target.y, accel_max);

		// update angle targets that will be passed to stabilize controller
		accel_to_lean_angles(_accel_target.x, _accel_target.y, target_roll, target_pitch);
		// Debug
		print_accel.x = _accel_target.x;
		print_accel.y = _accel_target.y;
	}
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