// Encoder + PID codes: perform motor angular position & velocity tracking
// Motor Model:
// Encoder Model:
// last revision: 04/02/2019

/* pin defintions */
#define PWMA (6)    // output PWM signal for motor A
#define PWMB (5)    // output PWM signal for motor B
#define ENC_A (2)   // Encoder Input A for motor A
#define ENC_B (3)   // Encoder Input B for motor A
#define ENC_C (8)   // Encoder Input A for motor B
#define ENC_D (9)   // Encoder Input A for motor B
#define AIN1 (10)
#define AIN2 (11)   // AIN1/2 for motor A
#define BIN1 (12)
#define BIN2 (13)   // AIN1/2 for motor B

/* Global Variables */
// Encoder variables
const int8_t encoder_table[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
volatile int32_t count_A = 0;   // encoder step count_A, 32 bit volatile integer (volatile ensures the value is fetched everytime, necessary for global variables modified by interrupt handlers)
volatile int32_t count_B = 0;   // encoder step count_B
int32_t prev_count_A = 0;
int32_t curr_count_A = 0;
int32_t prev_count_B = 0;
int32_t curr_count_B = 0;
const double counts_per_rev = 464.64;   // TO BE CHANGED WITH SPECIFIC MOTOR SPECS

// Time recording variables
unsigned long t_duration = 0;
unsigned long curr_time = 0;

// Angular Measurement Variables   
double curr_angular_pos_A = 0;
double curr_angular_speed_A = 0;


// PID variables
// Motor A:
double ref_angular_speed_A = 7;  // DESIRED SPEED FOR MOTOR A: TO BE SET
double ref_angular_pos_A = 20;
double curr_error_ang_pos_A = ref_angular_pos_A;       // error signals of angular measurements
double curr_error_ang_speed_A = ref_angular_speed_A;
double prev_error_ang_pos_A = ref_angular_pos_A;       
double prev_error_ang_speed_A = ref_angular_speed_A;
double curr_error_A = 0;
double prev_error_A = 0;
double total_error_A = 0;
double pid_p_term_A;
double pid_i_term_A;
double pid_d_term_A;
double pid_output_A;

// Motor B:
double ref_angular_speed_B = 7;  // DESIRED SPEED FOR MOTOR B: TO BE SET
double ref_angular_pos_B = 20;
double curr_error_ang_pos_B = ref_angular_pos_B;       // error signals of angular measurements
double curr_error_ang_speed_B = ref_angular_speed_B;
double prev_error_ang_pos_B = ref_angular_pos_B;       
double prev_error_ang_speed_B = ref_angular_speed_B;
double curr_error_B = 0;
double prev_error_B = 0;
double total_error_B = 0;
double pid_p_term_B;
double pid_i_term_B;
double pid_d_term_B;
double pid_output_B;


// PID Parameters
// Motor A:
double kp_A = 8;          // NEED MORE TUNING
double ki_A = 9;            
double kd_A = 0.1;
// Motor A:
double kp_B = 8;          // NEED MORE TUNING
double ki_B = 9;            
double kd_B = 0.1;

// general purpose variables
int8_t is_first_loop = 1;
const int8_t is_tracking_ang_pos = 0;
const int8_t is_tracking_ang_speed = 1;
const double error_threshold_pos = 0.01;
const double error_threshold_speed = 0.01;
double pwm_output_A = 0;
double pwm_output_B = 0;

// TESTING PURPOSE: A sequence of reference speeds
//double curr_sec = 0;
//double ref_angular_speed_A_sequence[] = {-8,-7,-6,-5,-4,-3,-2,0,2,3,4,5,6,7,8};
//int8_t ref_speed_seq_length = 15;
//int8_t curr_speed_seq_idx = 0;

void setup() {
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(BIN1,OUTPUT);
    pinMode(BIN2,OUTPUT);
    pinMode(PWMA, OUTPUT);
    pinMode(PWMB, OUTPUT);
    pinMode(ENC_A, INPUT);
    pinMode(ENC_B, INPUT);
    pinMode(ENC_C, INPUT);
    pinMode(ENC_D, INPUT);

    Serial.begin(9600);      // open the serial port at 9600bps

    // attach the encoder interrupt function to changes on both encoder input pins
    attachInterrupt(digitalPinToInterrupt(ENC_A), encoderA_isr, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_B), encoderA_isr, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_C), encoderB_isr, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_D), encoderB_isr, CHANGE);

    // record the current time
    curr_time = millis();
}

void loop() {

    // TESTING: SEQUENCE OF REF SPEED, FIXED DURATION
    //if (millis() / 1000 - curr_sec > 2)
    //{
    //    if (curr_speed_seq_idx < ref_speed_seq_length)
    //    {
    //        curr_speed_seq_idx = curr_speed_seq_idx + 1;
    //        ref_angular_speed_A = ref_angular_speed_A_sequence[curr_speed_seq_idx];
    //    }
    //    else
    //    {
    //        ref_angular_speed_A = 7;
    //    }
    //    curr_sec = millis() / 1000;
    //}
    
    // wait for 0.2 sec between loops
    delay(200);
    
    // Read and store the current encoder measurement
    // NOTE: when accessing the encoder count_A, prevent interrupts from modifying the variable
    noInterrupts();
    prev_count_A = curr_count_A;
    curr_count_A = count_A;
    prev_count_B = curr_count_B;
    curr_count_B = count_B;
    interrupts();

    // obtain the time duration of this past loop
    t_duration = millis() - curr_time;
    curr_time = millis();
    
    // calculate motor A's angular position & velocity
    curr_angular_pos_A = curr_count_A / counts_per_rev;                       // unit: # of positive revolutions from start
    curr_angular_speed_A = (curr_count_A - prev_count_A) / (t_duration*0.001);   // unit: counts per second
    curr_angular_speed_A = curr_angular_speed_A / counts_per_rev;             // unit: revolutions per second

    // calculate motor B's angular position & velocity
    curr_angular_pos_B = curr_count_B / counts_per_rev;                       
    curr_angular_speed_B = (curr_count_B - prev_count_B) / (t_duration*0.001);
    curr_angular_speed_B = curr_angular_speed_B / counts_per_rev;

    // obtain the current error in angular position & velocity, and store the last error
    // MOTOR A:
    prev_error_ang_pos_A = curr_error_ang_pos_A;       
    prev_error_ang_speed_A = curr_error_ang_speed_A;
    curr_error_ang_pos_A = ref_angular_pos_A - curr_angular_pos_A;
    curr_error_ang_speed_A = ref_angular_speed_A - curr_angular_speed_A;
    // MOTOR B:
    prev_error_ang_pos_B = curr_error_ang_pos_B;       
    prev_error_ang_speed_B = curr_error_ang_speed_B;
    curr_error_ang_pos_B = ref_angular_pos_B - curr_angular_pos_B;
    curr_error_ang_speed_B = ref_angular_speed_B - curr_angular_speed_B;

    /* PID control starts here!! */
    // First determine what we want to track: angular pos or speed, and set error signal accordingly
    if (is_tracking_ang_pos == 1)
    {
        curr_error_A = curr_error_ang_pos_A;
        prev_error_A = prev_error_ang_pos_A;
        curr_error_B = curr_error_ang_pos_B;
        prev_error_B = prev_error_ang_pos_B;
    }
    else
    {
        curr_error_A = curr_error_ang_speed_A;
        prev_error_A = prev_error_ang_speed_A;
        curr_error_B = curr_error_ang_speed_B;
        prev_error_B = prev_error_ang_speed_B;
    }
    total_error_A += curr_error_A;
    total_error_B += curr_error_B;

    // check for the range of the current reference speed, and select proper set of PID parameters
    // Motor A:
    if (abs(ref_angular_speed_A) <= 2 && abs(curr_angular_speed_A) <= 2)
    {
        kp_A = 5;
        ki_A = 6;        // this set neesd more tuning: speed start from 0 to a small ref value
        kd_A = 0.1;
    }
    else
    {
        kp_A = 8;
        ki_A = 9;
        kd_A = 0.1;
    }

    // Motor B:
    if (abs(ref_angular_speed_B) <= 2 && abs(curr_angular_speed_B) <= 2)
    {
        kp_B = 5;
        ki_B = 6;        // this set neesd more tuning: speed start from 0 to a small ref value
        kd_B = 0.1;
    }
    else
    {
        kp_B = 8;
        ki_B = 9;
        kd_B = 0.1;
    }    

    // Next, obtain the three terms of the PID controller output
    // Motor A:
    pid_p_term_A = kp * curr_error_A;
    pid_i_term_A = ki * total_error_A;
    pid_d_term_A = kd * (curr_error_A - prev_error_A);
    // Motor B:
    pid_p_term_B = kp * curr_error_B;
    pid_i_term_B = ki * total_error_B;
    pid_d_term_B = kd * (curr_error_B - prev_error_B);


    // Finally, set the PID output depending on if this is the very first loop
    if (is_first_loop == 1)
    {
        pid_output_A = pid_p_term_A;
        pid_output_B = pid_p_term_B;
    }
    else
    {
        pid_output_A = pid_p_term_A + pid_i_term_A + pid_d_term_A;
        pid_output_B = pid_p_term_B + pid_i_term_B + pid_d_term_B;
    }

    
    // Last step: convert PID output to PWM signal, send it to the motor together with the correct spinning directions

    // Added part to make sure PID integrator (I term) doesn't accumulate error signals too much
    // Integrator Anti-windup Strategy
    if (abs(pid_output_A) > 255)
    {
        total_error_A -= curr_error_A;
    }
    if (abs(pid_output_B) > 255)
    {
        total_error_B -= curr_error_B;
    }
    
    /* This part is for angular POSITION tracking */
    if (is_tracking_ang_pos == 1)
    {
        // Motor A:
        if (abs(curr_error_A) <= error_threshold_pos)
        { analogWrite(PWMA,0); }
        else
        {
            if (curr_error_A >= 0) { motorForward_A(); }
            else { motorBackward_A(); }
            pwm_output_A = constrain(abs(pid_output_A), 0, 255);
            analogWrite(PWMA, pwm_output_A);
        }
        // Motor B:
        if (abs(curr_error_B) <= error_threshold_pos)
        { analogWrite(PWMB,0); }
        else
        {
            if (curr_error_B >= 0) { motorForward_B(); }
            else { motorBackward_B(); }
            pwm_output_B = constrain(abs(pid_output_B), 0, 255);
            analogWrite(PWMB, pwm_output_B);
        }
    }
    /* This part is for angular SPEED tracking */
    else
    {
        // Motor A:
        if (abs(curr_error_A) <= error_threshold_speed)
        { analogWrite(PWMA,pwm_output_A); }
        else
        {
            if (ref_angular_speed_A >= 0) { motorForward_A(); }
            else { motorBackward_A(); }
            pwm_output_A = constrain(abs(pid_output_A), 0, 255);
            analogWrite(PWMA, pwm_output_A);
        }
        // Motor B:
        if (abs(curr_error_B) <= error_threshold_speed)
        { analogWrite(PWMB,pwm_output_B); }
        else
        {
            if (ref_angular_speed_B >= 0) { motorForward_B(); }
            else { motorBackward_B(); }
            pwm_output_B = constrain(abs(pid_output_B), 0, 255);
            analogWrite(PWMB, pwm_output_B);
        }
    }
    
    is_first_loop = 0;
    

    // print motors' status
    Serial.print("Speed A: ");
    Serial.println(curr_angular_speed_A);
    Serial.print("Speed B: ");
    Serial.println(curr_angular_speed_B);

    Serial.print("Error A: ");
    Serial.println(curr_error_A);
    Serial.print("Error B: ");
    Serial.println(curr_error_B);

    /* Additional Printing Info
    Serial.print("PID Output A: ");
    Serial.println(pid_output_A);
    Serial.print("PWM A: ");
    Serial.println(pwm_output_A);
    Serial.print("PID Output B: ");
    Serial.println(pid_output_B);
    Serial.print("PWM B: ");
    Serial.println(pwm_output_B);
    */
}
// End of the Main Loop




// Additional Functions, ISR:

// motor A's encoder interrupt function, fires on change of either encoder input signal
void encoderA_isr() {
    static uint8_t enc_val_A = 0; // static allows this value to persist across function calls

    enc_val_A = enc_val_A << 2; // shift the previous state to the left
    enc_val_A = enc_val_A | ((PIND & 0b1100) >> 2); // or the current state into the 2 rightmost bits

    count_A += encoder_table[enc_val_A & 0b1111];    // preform the table lookup and increment count_A accordingly
}

// motor B's encoder interrupt function
void encoderB_isr() {
    static uint8_t enc_val_B = 0; // static allows this value to persist across function calls

    enc_val_B = enc_val_B << 2; // shift the previous state to the left
    enc_val_B = enc_val_B | ((PIND & 0b1100) >> 2); // or the current state into the 2 rightmost bits

    count_B += encoder_table[enc_val_B & 0b1111];    // preform the table lookup and increment count_A accordingly
}


void motorBackward_A() {
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
}
void motorForward_A() {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
}
void motorBackward_B() {
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
}
void motorForward_B() {
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
}

