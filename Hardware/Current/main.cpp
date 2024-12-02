#include "mbed.h"
#include "rtos.h"
#include "EthernetInterface.h"
#include "ExperimentServer.h"
#include "QEI.h"
#include "BezierCurve.h"
#include "MotorShield.h" 
#include "HardwareSetup.h"

#define NUM_INPUTS 25
#define NUM_OUTPUTS 25 //16

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

// trajectory phase timing parameters
float T1;
float T2;
float T3;

//feedback gains
float K1_pos = 20;
float K2_pos = 20; 
float K3_pos = 20;

float D1_vel = 0;
float D2_vel = 0;
float D3_vel = 0;

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

float boost_torque = 0.5;
float boost_duration = 0.1;

float phase1_playback_speed = 3.0;
float phase2_playack_speed = 1.0;




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

    
    if (t.read() < (T1*phase1_playback_speed-0.1)){  // changed from t.read() < T1
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

//function to get the feed forward torques based on the polynomial fit
float get_val(float t, float params[6]) {
    float ret = 0.0;
    ret = params[0]*float(pow(t,5)) +
        params[1]*float(pow(t,4)) + 
        params[2]*float(pow(t,3)) + 
        params[3]*float(pow(t,2)) + 
        params[4]*float(pow(t,1)) + 
        params[5]; 
    return ret;
}

//function to get the desired position for feedback based on polynomial fit
int n_pos = 6; //degree of polynomial fit
float get_pos_val(float t, float params[n_pos]) {
    float ret = 0.0;
    ret = params[0]*float(pow(t,5)) +
        params[1]*float(pow(t,4)) + 
        params[2]*float(pow(t,3)) + 
        params[3]*float(pow(t,2)) + 
        params[4]*float(pow(t,1)) + 
        params[5];
    ret = ret * -1.0; // flipped
    return ret;
}

//function to get the desired velocity for feedback based on polynomial fit
int n_vel = 6; //degree of polynomial fit
float get_vel_val(float t, float params[n_vel]) {
    float ret = 0.0;
    ret = params[0]*float(pow(t,5)) +
        params[1]*float(pow(t,4)) + 
        params[2]*float(pow(t,3)) + 
        params[3]*float(pow(t,2)) + 
        params[4]*float(pow(t,1)) + 
        params[5];
    ret = ret * -1.0; // flipped
    return ret;
}

int main(void) {
    // Link the terminal with our server and start it up
    server.attachTerminal(pc);
    server.init();
    
    // Continually get input from MATLAB and run experiments
    float input_params[NUM_INPUTS];

     // optimization output fit parameters
    float tau1_1_params[6] = { 4.4395 ,  -1.7283 ,   0.2363 ,  -0.0134 ,   0.0003 ,  -0.0000};
    float tau2_1_params[6] = {-2.6979  ,  1.4597 ,  -0.2795  ,  0.0260   -0.0013 ,   0.0000};
    float tau3_1_params[6] = {0.3874  ,  1.3308 ,  -0.4641,    0.0572,   -0.0032 ,  -0.0000};

    float tau1_2_params[6] = { 3.8199 ,  -5.8710  ,  3.5071 ,  -1.0193  ,  0.1443 ,  -0.0079};
    float tau2_2_params[6] = { -1.3484 ,   2.1431 ,  -1.3236 ,   0.3964 ,  -0.0576 ,   0.0033};
    float tau3_2_params[6] = {0.2344 ,  -0.3579 ,   0.2117,   -0.0606 ,   0.0084  , -0.0005};
    
    float pos1_1_params[6] = {-3.5955,   -0.6761 ,   0.4986 ,  -0.0924 ,  -0.0009 ,   0.0011};
    float pos2_1_params[6] = {3.2381 ,   7.0721 ,  -2.3780 ,   0.1760 ,   0.0012,    0.0003};
    float pos3_1_params[6] = {-0.0697 ,  -1.0782 ,   0.2871 ,  -0.0217 ,   0.0005,   -0.0000};

    float pos1_2_params[6] = {-2.1824 ,   3.2466 ,  -1.8799 ,   0.5396 ,  -0.0783 ,   0.0045};
    float pos2_2_params[6] = {1.8082 ,  -2.5257 ,   1.3114,   -0.3148 ,   0.0359 ,  -0.0016};
    float pos3_2_params[6] = {-1.4201 ,   2.8376 ,  -1.8268 ,   0.4240 ,  -0.0359 ,   0.0006};

    float vel1_1_params[6] = {2.1848 ,  -0.7633 ,  -0.0097  ,  0.0207 ,  -0.0023 ,  -0.0000};
    float vel2_1_params[6] = {-1.5912 ,   0.6240 ,  -0.0500 ,  -0.0034  ,  0.0003 ,   0.0000};
    float vel3_1_params[6] = {8.5866 ,  -3.7114  ,  0.5330 ,  -0.0301 ,   0.0006 ,  -0.0000};

    float vel1_2_params[6] = { -3.5492  ,  4.8072 ,  -2.5503,    0.6673,   -0.0850 ,   0.0040};
    float vel2_2_params[6] = {  -0.8185 ,   1.3868 ,  -0.9064  ,  0.2844  , -0.0428 ,   0.0025};
    float vel3_2_params[6] = { 3.8684 ,  -6.2281  ,  3.9582,   -1.2311 ,   0.1847  , -0.0107};

    
    for (int i = 0; i < 6; ++i) { 
        tau1_1_params[i] *= float(pow(10,5)); 
        tau2_1_params[i] *= float(pow(10,5)); 
        tau3_1_params[i] *= float(pow(10,4)); 
        tau1_2_params[i] *= float(pow(10,4)); 
        tau2_2_params[i] *= float(pow(10,5)); 
        tau3_2_params[i] *= float(pow(10,-18)); 

        pos1_1_params[i] *= float(pow(10, 3));
        pos2_1_params[i] *= float(pow(10, 3));
        pos3_1_params[i] *= float(pow(10, 3));
        pos1_2_params[i] *= float(pow(10, 4));
        pos2_2_params[i] *= float(pow(10, 4));
        pos3_2_params[i] *= float(pow(10, 3));

        vel1_1_params[i] *= float(pow(10, 5));
        vel2_1_params[i] *= float(pow(10, 6));
        vel3_1_params[i] *= float(pow(10, 5));
        vel1_2_params[i] *= float(pow(10, 5));
        vel2_2_params[i] *= float(pow(10, 6));
        vel3_2_params[i] *= float(pow(10, 5));
    } 

    T1 = 0.18; // original time
    T2 = 0.289; // original time
    T3 = 1.0; //additional time to spin
    
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

            K1_pos                      = input_params[7];
            K2_pos                      = input_params[8];
            K3_pos                      = input_params[9];

            D1_vel                      = input_params[10];
            D2_vel                      = input_params[11];
            D3_vel                      = input_params[12];

            tau1_weight                 = input_params[13];
            tau2_weight                 = input_params[14];
            tau3_weight                 = input_params[15];
            
            K3_brake                    = input_params[16];
            D3_brake                    = input_params[17];
            I3_brake                    = input_params[18];

            fric_comp_torque            = input_params[19];
            deadzone_radius             = input_params[20];
            boost_torque                = input_params[21];
            boost_duration              = input_params[22];
            
            phase1_playback_speed       = input_params[23];
            phase2_playack_speed        = input_params[24];


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

            float pos1_des = 0.0;
            float pos2_des = 0.0;
            float pos3_des = 0.0;
                         
            float vel1_des = 0.0;
            float vel2_des = 0.0;
            float vel3_des = 0.0;

            // Run experiment
            t.reset();
            t.start();
            while ((t.read() < (T1*phase1_playback_speed)+(T2*phase2_playack_speed)+T3)) {
            // while(t.read() < (T1*phase1_playback_speed)+(T2*phase2_playack_speed)+T3) { 
                float time = t.read();
                // Read encoders to get motor states
                angle1 = encoderA.getPulses() *PULSE_TO_RAD + angle1_init;       
                velocity1 = encoderA.getVelocity() * PULSE_TO_RAD;
                 
                angle2 = encoderB.getPulses() * PULSE_TO_RAD + angle2_init;       
                velocity2 = encoderB.getVelocity() * PULSE_TO_RAD;           

                angle3 = encoderC.getPulses() * PULSE_TO_RAD + angle3_init;       
                velocity3 = encoderC.getVelocity() * PULSE_TO_RAD;

                if (time < T1*phase1_playback_speed){
                    time = time/phase1_playback_speed;

                    tau1 = get_val(time, tau1_1_params);
                    tau2 = get_val(time, tau2_1_params);
                    tau3 = get_val(time, tau3_1_params);

                    pos1_des = get_pos_val(time, pos1_1_params);
                    pos2_des = get_pos_val(time, pos2_1_params);
                    pos3_des = get_pos_val(time, pos3_1_params);

                    vel1_des = get_vel_val(time, vel1_1_params);
                    vel2_des = get_vel_val(time, vel2_1_params);
                    vel3_des = get_vel_val(time, vel3_1_params);
                

                // } else if (time < T1+boost_duration) { // foot boost
                //     tau1 = get_val(time, tau1_2_params);
                //     tau2 = get_val(time, tau2_2_params);
                //     tau3 = boost_torque; // get_val(time, tau3_2_params);

                //     pos1_des = get_pos_val(time, pos1_2_params);
                //     pos2_des = get_pos_val(time, pos2_2_params);
                //     pos3_des = get_pos_val(time, pos3_2_params);

                //     vel1_des = get_vel_val(time, vel1_2_params);
                //     vel2_des = get_vel_val(time, vel2_2_params);
                //     vel3_des = get_vel_val(time, vel3_2_params);

                } else if (time < (T1*phase1_playback_speed)+T2) {
                    time = (time - (T1*phase1_playback_speed))/phase2_playack_speed + T1;
                    
                    tau1 = get_val(time, tau1_2_params);
                    tau2 = get_val(time, tau2_2_params);
                    tau3 = 0.0;

                    pos1_des = get_pos_val(time, pos1_2_params);
                    pos2_des = get_pos_val(time, pos2_2_params);
                    pos3_des = get_pos_val(time, pos3_2_params);

                    vel1_des = get_vel_val(time, vel1_2_params);
                    vel2_des = get_vel_val(time, vel2_2_params);
                    vel3_des = get_vel_val(time, vel3_2_params);
                } else {
                    tau1 = 0.0;
                    tau2 = 0.0;
                    tau3 = 0.0;

                    // pos1_des = get_pos_val(time, pos1_2_params);
                    // pos2_des = get_pos_val(time, pos2_2_params);
                    // pos3_des = get_pos_val(time, pos3_2_params);

                    // vel1_des = get_vel_val(time, pos1_2_params);
                    // vel2_des = get_vel_val(time, pos2_2_params);
                    // vel3_des = get_vel_val(time, pos3_2_params);
                }
                
                //find errors
                float e_pos1 = pos1_des - angle1;
                float e_pos2 = pos2_des - angle2;
                float e_pos3 = pos3_des - angle3;
                
                float e_vel1 =  vel1_des - velocity1;
                float e_vel2 = vel2_des - velocity2;
                float e_vel3 = vel3_des - velocity3;
 
                //friction compensation for the supporting leg motor
                // if (deadzone_radius > 0){
                //     if (velocity3 < -deadzone_radius) //if we are moving backwards, outside the deadzone
                //     {
                //         tau3 -= fric_comp_torque; //friction acts in the positive direction
                //     }

                //     if (velocity3 > deadzone_radius) //if we are moving forwards, outside the deadzone
                //     {
                //         tau3 += fric_comp_torque;//friction acts in the negative direction
                //     }
                // }
                
                
                // pc.printf(" = %f \n", tau3);

                // pc.printf("tau3 = %f \n", tau3);
                // pc.printf("tau2 = %f \n", tau2);
                // pc.printf("tau1 = %f \n", tau1);

                
                // // Set desired currents with both feedback and feed forward terms           
                current_des1 = K1_pos*(e_pos1) + D1_vel*(e_vel1) + tau1_weight * tau1/k_t;
                current_des2 = K2_pos*(e_pos2) + D2_vel*(e_vel2) + tau2_weight * tau2/k_t;
                current_des3 = K3_pos*(e_pos3) + D3_vel*(e_vel3) + tau3_weight * tau3/k_t;


                if (time > (T1 * phase1_playback_speed-0.1)){
                    // current_des1 = 0;
                    // current_des2 = 0;
                    if (deadzone_radius > 0) {
                        if (velocity3 < -deadzone_radius) //if we are moving backwards, outside the deadzone
                        {
                            tau3 -= fric_comp_torque; //friction acts in the positive direction
                        }

                        if (velocity3 > deadzone_radius) //if we are moving forwards, outside the deadzone
                        {
                            tau3 += fric_comp_torque;//friction acts in the negative direction
                        }
                        current_des3 = tau3/k_t;
                    }
                }

                // if (angle3 > angle3_init + 2*3.141592) {
                //     current_des1 = 0.0;
                //     current_des2 = 0.0;
                //     current_des3 = 0.0;
                //     motorShield.motorAWrite(0, 0); //turn motor A off
                //     motorShield.motorBWrite(0, 0); //turn motor B off
                //     motorShield.motorCWrite(0, 0); //turn motor B off
                // }

                // if (time > T2){
                //     // current_des1 = 0;
                //     // current_des2 = 0;
                //     if (velocity3 < -deadzone_radius) //if we are moving backwards, outside the deadzone
                //     {
                //         tau3 -= fric_comp_torque; //friction acts in the positive direction
                //     }

                //     if (velocity3 > deadzone_radius) //if we are moving forwards, outside the deadzone
                //     {
                //         tau3 += fric_comp_torque;//friction acts in the negative direction
                //     }
                //     current_des3 = tau3/k_t;
                // } 
                // current_des3 = 0;


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

                // optimization values
                output_data[16] = pos1_des;
                output_data[17] = pos2_des;
                output_data[18] = pos3_des;
                output_data[19] = vel1_des;
                output_data[20] = vel2_des;
                output_data[21] = vel3_des;
                output_data[22] = tau1;
                output_data[23] = tau2;
                output_data[24] = tau3;

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