/*
  Motor.cpp - Library for the Motor class.
  Created by Tianyang Cao, 04/07/2019.
  For IEEE Robotics Motor Abstraction Subteam.
*/ 

#include "Arduino.h"
#include "Motor.h"


Motor::Motor(bool motor_AorB, int PWM, int ENC_A, int ENC_B, int AIN1, int AIN2)
{
    // Pin assignment
    pinMode(PWM, OUTPUT);
    pinMode(ENC_A, INPUT);
    pinMode(ENC_B, INPUT);
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    _PWM = PWM;
    _ENC_A = ENC_A;
    _ENC_B = ENC_B;
    _AIN1 = AIN1;
    _AIN2 = AIN2;

    // object attributes initialization
    count = 0;   // encoder step count, 32 bit volatile integer (volatile ensures the value is fetched everytime, necessary for global variables modified by interrupt handlers)
    motor_id = motor_AorB;      // motor id: 0 - A, 1 - B
    prev_count = 0;
    curr_count = 0;
    counts_per_rev = 480;   // TO BE CHANGED WITH SPECIFIC MOTOR SPECS
    t_duration = 0;
    curr_angular_speed = 0;
    curr_error = 0;
    prev_error = 0;
    total_error = 0;
    kp = 8;           // 8    NEED MORE TUNING TO REDUCE INITIAL OVERSHOOT & SETTLING TIME
    ki = 9;            // 9
    kd = 0.1;          // 0.1
    is_first_loop = 1;
    error_threshold_speed = 0.01;
    curr_time = millis();
  
}

void Motor::setNewSpeed(double new_speed)
{
    ref_angular_speed = new_speed;
    //curr_error_ang_speed = ref_angular_speed - curr_angular_speed;
}

void Motor::motorForward()
{
    digitalWrite(_AIN1, LOW);
    digitalWrite(_AIN2, HIGH);
}

void Motor::motorBackward()
{
    digitalWrite(_AIN1, HIGH);
    digitalWrite(_AIN2, LOW);
}

double Motor::pidControl(int32_t count)
{
    prev_count = curr_count;
    curr_count = count;

    // TODO: might need counter overflow prevention
    // counter can store value ranging from -2,147,483,648 to 2,147,483,647

    // obtain the time duration of this past loop
    t_duration = millis() - curr_time;
    curr_time = millis();

    // calculate the current motor angular velocity
    curr_angular_speed = (curr_count - prev_count) / (t_duration*0.001);   // unit: counts per second
    curr_angular_speed = curr_angular_speed / counts_per_rev;             // unit: revolutions per second

    // obtain the current error in angular velocity, and store the last error
    prev_error_ang_speed = curr_error_ang_speed;
    curr_error_ang_speed = ref_angular_speed - curr_angular_speed;

    /* PID control starts here!! */
    curr_error = curr_error_ang_speed;
    prev_error = prev_error_ang_speed;
    total_error += curr_error;

    // check for the range of the current reference speed, and select proper set of PID parameters
    if (abs(ref_angular_speed) <= 2)
    {
        kp = 5;
        ki = 6;
        kd =0.1;  
    }
    else
    {
        kp = 8;
        ki = 9;
        kd = 0.1;
    }
//    if (abs(ref_angular_speed) <= 3)
//    {
//        if (abs(curr_angular_speed) < 0.1)
//        {
//          kp = 5;
//          ki = 6;        // this set neesd more tuning: speed start from 0 to a ref value less than 2
//          kd = 0.1;
//        }
//        else
//        {
//          kp = 4;
//          ki = 4.5;
//          kd = 0.1;
//        }
//    }
//    else
//    {
//        kp = 8;
//        ki = 9;
//        kd = 0.1;
//    }

    // Next, obtain the three terms of the PID controller output
    pid_p_term = kp * curr_error;
    pid_i_term = ki * total_error;
    pid_d_term = kd * (curr_error - prev_error);

    // Finally, set the PID output depending on if this is the very first loop
    if (is_first_loop == 1)
    {
        pid_output = pid_p_term;
        is_first_loop = 0;
    }
    else
    {
        pid_output = pid_p_term + pid_i_term + pid_d_term;
    }

    // Last step: convert PID output to PWM signal, send it to the motor together with the correct spinning directions

    // Added part to make sure PID integrator (I term) doesn't accumulate error signals too much
    // if saturate, then reset
    if (abs(pid_output) > 255)
    {
        //total_error = 0;
        total_error -= curr_error;
    }

    /* Finally, produce pwm command for the motor */
    if (abs(curr_error) <= error_threshold_speed)
    { 
        analogWrite(_PWM, pwm_output); 
    }
    else
    {
        if (ref_angular_speed >= 0) { motorForward(); }
        else { motorBackward(); }
        
        pwm_output = constrain(abs(pid_output), 0, 255);
        analogWrite(_PWM, pwm_output);
    }
    
    // print the current motor status
    Serial.print("Motor ID: ");
    Serial.println(motor_id);
    Serial.print(", Ang Speed: ");
    Serial.println(curr_angular_speed);
    Serial.print(", Error: ");
    Serial.println(curr_error);
    Serial.print(", PID output: ");
    Serial.println(pid_output);
    Serial.print(", PWM output: ");
    Serial.println(pwm_output);
    Serial.println("--------------------");

    // return the current PWM command
    return pwm_output;
}


