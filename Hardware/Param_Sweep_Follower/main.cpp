#include "mbed.h"
#include "rtos.h"
#include "EthernetInterface.h"
#include "ExperimentServer.h"
#include "QEI.h"
#include "BezierCurve.h"
#include "MotorShield.h" 
#include "HardwareSetup.h"

#define NUM_INPUTS 24
#define NUM_OUTPUTS 19

#define PULSE_TO_RAD (2.0f*3.14159f / 1200.0f)

// Initializations
Serial pc(USBTX, USBRX);    // USB Serial Terminal
ExperimentServer server;    // Object that lets us communicate with MATLAB
Timer t;                    // Timer to measure elapsed time of experiment


QEI encoderA(PE_9,PE_11, NC, 1200, QEI::X4_ENCODING);  // MOTOR A ENCODER (no index, 1200 counts/rev, Quadrature encoding)
QEI encoderB(PA_5, PB_3, NC, 1200, QEI::X4_ENCODING);  // MOTOR B ENCODER (no index, 1200 counts/rev, Quadrature encoding)
QEI encoderC(PC_6, PC_7, NC, 1200, QEI::X4_ENCODING);  // MOTOR C ENCODER (no index, 1200 counts/rev, Quadrature encoding)
QEI encoderD(PD_12, PD_13, NC, 1200, QEI::X4_ENCODING);// MOTOR D ENCODER (no index, 1200 counts/rev, Quadrature encoding)

MotorShield motorShield(24000); //initialize the motor shield with a period of 24000 ticks or ~10kHZ
Ticker currentLoop;


// Variables for q1
float current1;
float current_des1 = 0;
float prev_current_des1 = 0;
float current_int1 = 0;
float angle1;
float velocity1;
float duty_cycle1;
float angle1_init;

// Variables for q2
float current2;
float current_des2 = 0;
float prev_current_des2 = 0;
float current_int2 = 0;
float angle2;
float velocity2;
float duty_cycle2;
float angle2_init;

// Variables for q3
float current3;
float current_des3 = 0;
float prev_current_des3 = 0;
float current_int3 = 0;
float angle3;
float velocity3;
float duty_cycle3;
float angle3_init;

// variables for braking
float prev_angle_3_error = 0;
float angle_3_cum = 0;
float K3_brake;
float D3_brake;
float I3_brake;


float tau1_weight = 1.0;
float tau2_weight = 1.0;
float tau3_weight = 1.0;

// Fixed kinematic parameters
const float l_OA=.011; 
const float l_OB=.042; 
const float l_AC=.096; 
const float l_DE=.091;

// Timing parameters
float current_control_period_us = 200.0f;     // 5kHz current control loop
float torque_control_period_us = 500.0f;  // 1kHz torque control loop
float start_period, traj_period, end_period;

// Current control parameters
float current_Kp = 4.0f;         
float current_Ki = 0.4f;           
float current_int_max = 3.0f;       
float duty_max;

// Model parameters
float supply_voltage = 12;     // motor supply voltage
float R = 2.0f;                // motor resistance
float k_t = 0.18f;             // motor torque constant
float nu = 0.0005;             // motor viscous friction


float fric_comp_torque = 0.020; 
float deadzone_radius = 0.1; //0.5;


float q1_max = 1.73;
float q2_min = -2.2;

bool q2_retract_based_on_q1_angle = true;
float q2_retract_q1_angle = 1.5;
float T_q2_in = 0.16;

bool brake_based_on_q1_angle = true;
float brake_release_q1_angle = 1.725;
float T_brake = 0.18 * 3;

float T_end = 3.0;


int phase = 1;

// float T1 = T_brake;
// float T2 = 0.31727828827 * 2.75;
// float T3;

// Current control interrupt function
void CurrentLoop() {
    // This loop sets the motor voltage commands using PI current controllers with feedforward terms.
    
    //use the motor shield as follows:
    //motorShield.motorAWrite(DUTY CYCLE, DIRECTION), DIRECTION = 0 is forward, DIRECTION =1 is backwards.
        
    current1 = -(((float(motorShield.readCurrentA())/65536.0f)*30.0f)-15.0f);           // measure current
    velocity1 = encoderA.getVelocity() * PULSE_TO_RAD;                                  // measure velocity        
    float err_c1 = current_des1 - current1;                                             // current errror
    current_int1 += err_c1;                                                             // integrate error
    current_int1 = fmaxf( fminf(current_int1, current_int_max), -current_int_max);      // anti-windup
    float ff1 = R*current_des1 + k_t*velocity1;                                         // feedforward terms
    duty_cycle1 = (ff1 + current_Kp*err_c1 + current_Ki*current_int1)/supply_voltage;   // PI current controller
    
    float absDuty1 = abs(duty_cycle1);
    if (absDuty1 > duty_max) {
        duty_cycle1 *= duty_max / absDuty1;
        absDuty1 = duty_max;
    }    

    if (current_des1 == 0){
        motorShield.motorAWrite(0.0, 1);
    } else {
        if (duty_cycle1 < 0) { // backwards
            motorShield.motorAWrite(absDuty1, 1);
        } else { // forwards
            motorShield.motorAWrite(absDuty1, 0);
        } 
    }            
    prev_current_des1 = current_des1; 
    
    current2 = -(((float(motorShield.readCurrentB())/65536.0f)*30.0f)-15.0f);           // measure current
    velocity2 = encoderB.getVelocity() * PULSE_TO_RAD;                                  // measure velocity  
    float err_c2 = current_des2 - current2;                                             // current error
    current_int2 += err_c2;                                                             // integrate error
    current_int2 = fmaxf( fminf(current_int2, current_int_max), -current_int_max);      // anti-windup   
    float ff2 = R*current_des2 + k_t*velocity2;                                         // feedforward terms
    duty_cycle2 = (ff2 + current_Kp*err_c2 + current_Ki*current_int2)/supply_voltage;   // PI current controller
    
    float absDuty2 = abs(duty_cycle2);
    if (absDuty2 > duty_max) {
        duty_cycle2 *= duty_max / absDuty2;
        absDuty2 = duty_max;
    }    
    if (current_des2 == 0){
        motorShield.motorBWrite(0.0, 1);
    } else {
        if (duty_cycle2 < 0) { // backwards
            motorShield.motorBWrite(absDuty2, 1);
        } else { // forwards
            motorShield.motorBWrite(absDuty2, 0);
        }
    }             
    prev_current_des2 = current_des2; 

    bool brake_on = (brake_based_on_q1_angle && (angle1 < brake_release_q1_angle)) || (!brake_based_on_q1_angle && (t.read() < T_brake));
    if (brake_on){ 
        float angle_3_error = angle3_init - angle3;
        angle_3_error = angle3_init - angle3;
        angle_3_cum += angle_3_error;
        float der_error = (angle_3_error - prev_angle_3_error)/0.001;
        prev_angle_3_error = angle_3_error;
        float V_command = (K3_brake * angle_3_error + D3_brake * der_error + I3_brake * angle_3_cum)/12.0f;
                        
        if (V_command < 0) {
            if (V_command <= -1.0){
                V_command = -1.0;
            }
            motorShield.motorCWrite(abs(V_command), 1);
        } else {
            if (V_command >= 1.0){
                V_command = 1.0;
            }
            motorShield.motorCWrite(abs(V_command), 0); //clockwise
        }
    } else {
        current3 = -(((float(motorShield.readCurrentC())/65536.0f)*30.0f)-15.0f);           // measure current
        velocity3 = encoderC.getVelocity() * PULSE_TO_RAD;                                  // measure velocity  
        float err_c3 = current_des3 - current3;                                             // current error
        current_int3 += err_c3;                                                             // integrate error
        current_int3 = fmaxf( fminf(current_int3, current_int_max), -current_int_max);      // anti-windup   
        float ff3 = R*current_des3 + k_t*velocity3;                                         // feedforward terms
        duty_cycle3 = (ff3 + current_Kp*err_c3 + current_Ki*current_int3)/supply_voltage;   // PI current controller
        
        float absDuty3 = abs(duty_cycle3);
        if (absDuty3 > duty_max) {
            duty_cycle3 *= duty_max / absDuty3;
            absDuty3 = duty_max;
        }    
        if (current_des3 == 0){ 
            motorShield.motorCWrite(0.0, 1);
        }
        else {
            if (duty_cycle3 < 0) { // backwards
                motorShield.motorCWrite(absDuty3, 1);
            } else { // forwards
                motorShield.motorCWrite(absDuty3, 0);
            }             
        }
        prev_current_des3 = current_des3; 
    }
}



int main(void) {
    // Link the terminal with our server and start it up
    server.attachTerminal(pc);
    server.init();
    
    // Continually get input from MATLAB and run experiments
    float input_params[NUM_INPUTS];
    
    pc.printf("about to enter while loop \n");
    while(1) {
        // If there are new inputs, this code will run
        if (server.getParams(input_params,NUM_INPUTS)) {          
            pc.printf("got parameters \n");
            // Get inputs from MATLAB          
            start_period                = input_params[0];    // First buffer time, before trajectory
            traj_period                 = input_params[1];    // Trajectory time/length
            end_period                  = input_params[2];    // Second buffer time, after trajectory
            
            duty_max                    = input_params[3];
            
            angle1_init                 = input_params[4];    // Initial angle for q1 (rad)
            angle2_init                 = input_params[5];    // Initial angle for q2 (rad)
            angle3_init                 = input_params[6];   // Initial angle for q3 (rad)

            tau1_weight                 = input_params[7];
            tau2_weight                 = input_params[8];
            tau3_weight                 = input_params[9];
            
            K3_brake                    = input_params[10];
            D3_brake                    = input_params[11];
            I3_brake                    = input_params[12];

            fric_comp_torque            = input_params[13];
            deadzone_radius             = input_params[14];

            q1_max                      = input_params[15];
            q2_min                      = input_params[16];

            q2_retract_based_on_q1_angle  = input_params[17];
            q2_retract_q1_angle         = input_params[18];
            T_q2_in                     = input_params[19];

            brake_based_on_q1_angle     = input_params[20];
            brake_release_q1_angle      = input_params[21];
            T_brake                     = input_params[22];

            T_end                       = input_params[23];


            // Attach current loop interrupt
            currentLoop.attach_us(CurrentLoop,current_control_period_us);
                        
            // Setup experiment
            encoderA.reset();
            encoderB.reset();
            encoderC.reset();
            encoderD.reset();

            motorShield.motorAWrite(0, 0); //turn motor A off
            motorShield.motorBWrite(0, 0); //turn motor B off
            motorShield.motorCWrite(0, 0); //turn motor C off

            float tau1 = 0.0;
            float tau2 = 0.0;
            float tau3 = 0.0;


            // Run experiment
            t.reset();
            t.start();
            while (phase < 3) {
                float time = t.read();
                // Read encoders to get motor states
                angle1 = encoderA.getPulses() *PULSE_TO_RAD + angle1_init;       
                velocity1 = encoderA.getVelocity() * PULSE_TO_RAD;
                 
                angle2 = encoderB.getPulses() * PULSE_TO_RAD + angle2_init;       
                velocity2 = encoderB.getVelocity() * PULSE_TO_RAD;           

                angle3 = encoderC.getPulses() * PULSE_TO_RAD + angle3_init;       
                velocity3 = encoderC.getVelocity() * PULSE_TO_RAD;

                tau1 = 0.0;
                tau2 = 0.0;
                tau3 = 0.0;

                if (q2_retract_based_on_q1_angle) {
                    if ((angle1 > q2_retract_q1_angle) && (angle2 > q2_min + 0.1)) {
                        tau2 = 1.0;
                    }
                } else {
                    if ((time > T_q2_in) && (angle2 > q2_min + 0.1)) {
                        tau2 = 1.0;
                    }
                }

                if (angle1 < q1_max) {
                    tau1 = 1.0;
                } 

                if (time > T_end) {
                    phase = 3;
                }

                
                // Set desired currents with both feedback and feed forward terms           
                current_des1 = tau1_weight * tau1/k_t;
                current_des2 = tau2_weight * tau2/k_t;
                current_des3 = tau3_weight * tau3/k_t;


                if (time > (T_brake)){ // friction compensation -- TODO -- enable only in one direction
                    if (deadzone_radius > 0) { // friction comp enabled
                        if (velocity3 < -deadzone_radius){ //if we are moving backwards, outside the deadzone
                            tau3 -= fric_comp_torque; //friction acts in the positive direction
                        } else if (velocity3 > deadzone_radius) { //if we are moving forwards, outside the deadzone
                            tau3 += fric_comp_torque;//friction acts in the negative direction
                        }
                        current_des3 = tau3/k_t;
                    }
                }


                // Form output to send to MATLAB     
                float output_data[NUM_OUTPUTS];
                // current time
                output_data[0] = t.read();

                // motor 1 state
                output_data[1] = angle1;
                output_data[2] = velocity1;  
                output_data[3] = current1;
                output_data[4] = current_des1;
                output_data[5] = duty_cycle1;

                // motor 2 state
                output_data[6] = angle2;
                output_data[7] = velocity2;
                output_data[8] = current2;
                output_data[9] = current_des2;
                output_data[10]= duty_cycle2;
                // motor 3 state
                output_data[11] = angle3;
                output_data[12] = velocity3;
                output_data[13] = current3;
                output_data[14] = current_des3;
                output_data[15]= duty_cycle3;

                output_data[16] = tau1;
                output_data[17] = tau2;
                output_data[18] = tau3;

                // Send data to MATLAB
                server.sendData(output_data,NUM_OUTPUTS);

                wait_us(torque_control_period_us);   
            }
            
            // Cleanup after experiment
            server.setExperimentComplete();
            currentLoop.detach();
            motorShield.motorAWrite(0, 0); //turn motor A off
            motorShield.motorBWrite(0, 0); //turn motor B off
            motorShield.motorCWrite(0, 0); //turn motor B off
        } // end if
    } // end while
} // end main