/*
	Motor.h - Library for the Motor class
	Created by Tianyang Cao, 04/07/2019.
	For IEEE Robotics Motor Abstraction Subteam.
*/

#ifndef MOTOR_H
#define MOTOR_H

#include "Arduino.h"

class Motor
{
    public:
    	// encoder variables
    	int32_t count;
	    int32_t prev_count;
	    int32_t curr_count;
	    double counts_per_rev;   // TO BE CHANGED WITH SPECIFIC MOTOR SPECS
	    unsigned long t_duration;
	    unsigned long curr_time;

	    // PID variables
	    double ref_angular_speed;  // 7 revolutions per second
	    double curr_angular_speed;
	    double curr_error_ang_speed;
	    double prev_error_ang_speed;
	    double curr_error;
	    double prev_error;
	    double total_error;
	    double pid_p_term;
	    double pid_i_term;
	    double pid_d_term;
	    double pid_output;
	    double kp;           
	    double ki;            
	    double kd;          

	    // general purpose variables
	    bool motor_id;
	    int8_t is_first_loop;
	    int8_t is_tracking_ang_speed;
	    double error_threshold_speed;
	    double pwm_output;

	    // constructor
	    Motor(bool motor_AorB, int PWM, int ENC_A, int ENC_B, int AIN1, int AIN2);
	    // class functions
	    void setNewSpeed(double new_speed);
	    void motorForward();
	    void motorBackward();
	    double pidControl(int32_t count);

	private:
		int _PWM;
		int _ENC_A;
		int _ENC_B;
		int _AIN1;
		int _AIN2;
        
};

#endif