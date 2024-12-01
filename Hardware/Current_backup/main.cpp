#include "mbed.h"
#include "rtos.h"
#include "EthernetInterface.h"
#include "ExperimentServer.h"
#include "QEI.h"
#include "BezierCurve.h"
#include "MotorShield.h" 
#include "HardwareSetup.h"

#define NUM_TORQUE_POINTS 5 // this is a lot to send over through the cable?
#define NUM_INPUTS 8 + 3*NUM_TORQUE_POINTS + 2
#define NUM_OUTPUTS 16

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


float prev_angle_3_error = 0;
float angle_3_cum = 0;
float Kp_body;
float Kd_body;
float Ki_body;

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

    
    if (t.read() < T1-0.1){ 
        float angle_3_error = angle3_init - angle3;
        angle_3_error = angle3_init - angle3;
        angle_3_cum += angle_3_error;
        float der_error = (angle_3_error - prev_angle_3_error)/0.001;
        prev_angle_3_error = angle_3_error;
        float V_command = (Kp_body * angle_3_error + Kd_body * der_error + Ki_body * angle_3_cum)/12.0f;
                        
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
    ret = ret * -1.0;
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
    ret = ret * -1.0;
    return ret;
}

int main(void) {
    // Link the terminal with our server and start it up
    server.attachTerminal(pc);
    server.init();
    
    // Continually get input from MATLAB and run experiments
    float input_params[NUM_INPUTS];
    // pc.printf("%f",input_params[0]);

    // float tau1_1_params[6] = {1.8766,    0.3966,   -0.2075,    0.0205,   -0.0006,   -0.0001};
    // float tau2_1_params[6] = {-6.8813 ,   2.7619,   -0.3989 ,   0.0274 ,  -0.0009 ,   0.0000};
    // float tau3_1_params[6] = {-2.5779 ,   1.1278 ,  -0.1668  ,  0.0102 ,  -0.0003  , -0.0000};

    // float tau1_2_params[6] = { -2.2558,    4.2723 ,  -3.1316  ,  1.1078 ,  -0.1887  ,  0.0134};
    // float tau2_2_params[6] = { 1.2948 ,  -2.5808 ,   1.9780 ,  -0.7274 ,   0.1281 ,  -0.0086};
    // float tau3_2_params[6] = { -0.0793 ,   0.1565 ,  -0.1186   , 0.0431 ,  -0.0075 ,   0.0005};

    // float tau1_1_params[6] = {3.9516,   -0.3434,   -0.1091,    0.0147,   -0.0005 ,  -0.0001};
    // float tau2_1_params[6] = {2.3343 ,   1.1308  , -0.2104  ,  0.0208,   -0.0011,    0.0000};
    // float tau3_1_params[6] = {-4.8763 ,   3.6737 ,  -0.8238  ,  0.0798 ,  -0.0038 ,  -0.0000};

    // float tau1_2_params[6] = {-0.9966 ,   1.9320 ,  -1.4377  ,  0.5129 ,  -0.0876   , 0.0057};
    // float tau2_2_params[6] = {0.9430 ,  -1.8651 ,   1.4181 ,  -0.5170  ,  0.0902,   -0.0060};
    // float tau3_2_params[6] = { -0.1165 ,   0.2247,  -0.1664  ,  0.0591  , -0.0101  ,  0.0007};

     // //optimization output fit parameters
    float tau1_1_params[6] = {-2.3212,   1.1758,   -0.2212,    0.0189,   -0.0007,   -0.0000};
    float tau2_1_params[6] = {1.3169,   -0.6081,    0.1019,   -0.0072,    0.0002,   -0.0000};
    float tau3_1_params[6] = {2.5632,   -1.0983,   0.1645,   -0.0095,    0.0001,   -0.0000};

    float tau1_2_params[6] = { -0.9966,    1.9320,   -1.4377,    0.5129,   -0.0876,    0.0057};
    float tau2_2_params[6] = {  0.9430,   -1.8651,    1.4181,   -0.5170,   0.0902,   -0.0060};
    float tau3_2_params[6] = {  -0.1165,    0.2247,   -0.1664,    0.0591,   -0.0101,    0.0007};
    
    float pos1_1_params[6] = {  1.0475,   -0.5798,    0.1315,   -0.0176,    0.0002,    0.0001};
    float pos2_1_params[6] = {  -2.6424,    1.6529,   -0.4019,    0.0379,  -0.0006,    0.0000};
    float pos3_1_params[6] = { -156.4154,  173.7448,  -36.5023,    2.2081,   -0.0407,  0.0002};

    float pos1_2_params[6] = {2.5230,   -4.2929,    2.4963,   -0.5116,    0.0046,    0.0044};
    float pos2_2_params[6] = {2.7758,   -5.4986,    3.9520,   -1.2997,   0.2061,   -0.0126};
    float pos3_2_params[6] = {-1.4581,  2.8121,   -1.8463,    0.4504,  -0.0409,    0.0009};

    float vel1_1_params[6] = {3.7969,   -1.6244,    0.2049,    0.0007,   -0.0021,   0.0000};
    float vel2_1_params[6] = {-9.7992,   4.1327,   -0.4337,   -0.0245,    0.0041,   -0.0000};
    float vel3_1_params[6] = { -3.1424,    1.3347,   -0.1613,    0.0056,   -0.0000,    0.0000};

    float vel1_2_params[6] = {-1.7894,    3.6797,   -2.9036,    1.0884,   -0.1913,    0.0125};
    float vel2_2_params[6] = {2.6604,   -5.1540,    3.8568,   -1.3969,    0.2453,   -0.0166};
    float vel3_2_params[6] = {-3.4109,    5.9481,   -3.9218,    1.2813,   -0.2302,    0.0174};
    

    T1 = 0.1790*3; //+0.25;
    T2 = T1 + 0.465; // 0.640421948726836; 
    T3 = 3; //additional time to spin

    //from Liane slack message 11/19 4:50 pm
    for (int i = 0; i < 6; ++i) { 
        tau1_1_params[i] *= float(pow(10,5)); 
        tau2_1_params[i] *= float(pow(10,6)); 
        tau3_1_params[i] *= float(pow(10,5)); 
        tau1_2_params[i] *= float(pow(10,3)); 
        tau2_2_params[i] *= float(pow(10,4)); 
        tau3_2_params[i] *= float(pow(10,-24)); 

        pos1_1_params[i] *= float(pow(10, 4));
        pos2_1_params[i] *= float(pow(10, 4));
        // pos3_1_params[i] *= float(pow(10, 4));
        pos1_2_params[i] *= float(pow(10, 3));
        pos2_2_params[i] *= float(pow(10, 3));
        pos3_2_params[i] *= float(pow(10, 3));

        vel1_1_params[i] *= float(pow(10, 5));
        vel2_1_params[i] *= float(pow(10, 5));
        vel3_1_params[i] *= float(pow(10, 4));
        vel1_2_params[i] *= float(pow(10, 5));
        vel2_2_params[i] *= float(pow(10, 5));
        vel3_2_params[i] *= float(pow(10, 4));
    } 
    
    while(1) {
        // If there are new inputs, this code will run
        if (server.getParams(input_params,NUM_INPUTS)) {          
            
            // Get inputs from MATLAB          
            start_period                = input_params[0];    // First buffer time, before trajectory
            traj_period                 = input_params[1];    // Trajectory time/length
            end_period                  = input_params[2];    // Second buffer time, after trajectory

            
            angle1_init                 = input_params[3];    // Initial angle for q1 (rad)
            angle2_init                 = input_params[4];    // Initial angle for q2 (rad)
            angle3_init                 = input_params[5];   // Initial angle for q3 (rad)

            duty_max                   = input_params[6];
            Kp_body   = input_params[7];


            float torque_pts1[NUM_TORQUE_POINTS];
            float torque_pts2[NUM_TORQUE_POINTS];
            float torque_pts3[NUM_TORQUE_POINTS];
            
            for (int i = 0; i< NUM_TORQUE_POINTS;i++) {
              torque_pts1[i] = input_params[8+i];    
            //   torque_pts2[i] = input_params[8+i];  
            //   torque_pts3[i] = input_params[8+i];  
              torque_pts2[i] = input_params[8+NUM_TORQUE_POINTS+i];  
              torque_pts3[i] = input_params[8+2*NUM_TORQUE_POINTS+i];  
            }

            float Kd_body = input_params[8+3*NUM_TORQUE_POINTS];
            float Ki_body = input_params[8+3*NUM_TORQUE_POINTS+1];


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
            while(t.read() < T3) { 
                float time = t.read();
                // Read encoders to get motor states
                angle1 = encoderA.getPulses() *PULSE_TO_RAD + angle1_init;       
                velocity1 = encoderA.getVelocity() * PULSE_TO_RAD;
                 
                angle2 = encoderB.getPulses() * PULSE_TO_RAD + angle2_init;       
                velocity2 = encoderB.getVelocity() * PULSE_TO_RAD;           

                angle3 = encoderC.getPulses() * PULSE_TO_RAD + angle3_init;       
                velocity3 = encoderC.getVelocity() * PULSE_TO_RAD;    

                if (time < T1){
                    time = time/3.0;
                    tau1 = get_val(time, tau1_1_params);
                    tau2 = get_val(time, tau2_1_params);
                    tau3 = get_val(time, tau3_1_params);

                    pos1_des = get_pos_val(time, pos1_1_params);
                    pos2_des = get_pos_val(time, pos2_1_params);
                    pos3_des = get_pos_val(time, pos3_1_params);

                    vel1_des = get_vel_val(time, vel1_1_params);
                    vel2_des = get_vel_val(time, vel2_1_params);
                    vel3_des = get_vel_val(time, vel3_1_params);

                // } else if (time < T1+0.2) {
                //     tau1 = get_val(time, tau1_2_params);
                //     tau2 = get_val(time, tau2_2_params);
                //     tau3 = 5.0; // get_val(time, tau3_2_params);

                //     pos1_des = get_pos_val(time, pos1_2_params);
                //     pos2_des = get_pos_val(time, pos2_2_params);
                //     pos3_des = get_pos_val(time, pos3_2_params);

                //     vel1_des = get_vel_val(time, vel1_2_params);
                //     vel2_des = get_vel_val(time, vel2_2_params);
                //     vel3_des = get_vel_val(time, vel3_2_params);

                } else if (time < T2) {
                    time = time - T1 + T1/3.0;
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
                float fric_comp_torque = 0.020; 
                float deadzone_radius = 0.1; //0.5;
                
                // if (velocity3 < -deadzone_radius) //if we are moving backwards, outside the deadzone
                // {
                //     tau3 -= fric_comp_torque; //friction acts in the positive direction
                // }

                // if (velocity3 > deadzone_radius) //if we are moving forwards, outside the deadzone
                // {
                //     tau3 += fric_comp_torque;//friction acts in the negative direction
                // }
                
                
                // pc.printf(" = %f \n", tau3);

                // pc.printf("tau3 = %f \n", tau3);
                // pc.printf("tau2 = %f \n", tau2);
                // pc.printf("tau1 = %f \n", tau1);

                // //FRICTION COMPENSATION EXPERIMENTS:
                // //run only the supporting leg motor
                // current_des1 = 0;
                // current_des2 = 0;
                // current_des3 = tau3/k_t;

                // tau1 = 0;
                // tau2 = 0;
                // tau3 = 0;
                
                // // Set desired currents with both feedback and feed forward terms           
                current_des1 = K1_pos*(e_pos1) + D1_vel*(e_vel1) + tau1/k_t;
                current_des2 = K2_pos*(e_pos2) + D2_vel*(e_vel2) + tau2/k_t;
                current_des3 = K3_pos*(e_pos3) + D3_vel*(e_vel3) + tau3/k_t;

                // current_des1 = 0;
                // current_des2 = 0;
                // current_des3 = 0;

                if (time > T1){
                    current_des1 = 0;
                    current_des2 = 0;
                }

                if (time > T2){
                    // current_des1 = 0;
                    // current_des2 = 0;

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