% This is the main MATLAB script for Lab 5.
% You will need to modify the Mbed code and this script, but should not need to make any other changes.

% Total experiment time is buffer,trajectory,buffer
traj_time         = 5;
pre_buffer_time   = 2; % this should be 0 for constant points, 2 for Bezier trajectories
post_buffer_time  = 2;


% Maximum duty cycle commanded by controller (should always be <=1.0)
duty_max   = 0.4; 

% Initial leg angles for encoder resets (negative of q1,q2 in lab handout due to direction motors are mounted)
angle1_init = -pi/3; 
angle2_init = -0.3; 
angle3_init = 0;

% outer pos, vel, torque FB loop
K1_pos = 20.0;
K2_pos = 20.0; 
K3_pos = 20.0;

D1_vel = 0.0;
D2_vel = 0.0;
D3_vel = 0.0;

tau1_weight = 1.0;
tau2_weight = 1.0;
tau3_weight = 1.0;

% faster, current control loop. For braking (motor 3)
K3_brake = 30;
D3_brake = 100;
I3_brake = 0.02;

% friction comp
fric_comp_torque = 0.020;
deadzone_radius = 0.1;

% boost from foot
boost_torque = 5.0;
boost_duration = 0.1;

% playback speed of trajectory (speed = 3.0 means the trajectory is played back 3 times slower)
phase1_playback_speed = 3.0;
phase2_playback_speed = 3.0;

%% Run Experiment
[output_data] = RunTrajectoryExperiment(traj_time, pre_buffer_time, post_buffer_time, duty_max, ...
    angle1_init, angle2_init, angle3_init,...
    K1_pos, K2_pos, K3_pos, ...
    D1_vel, D2_vel, D3_vel, ...
    tau1_weight, tau2_weight, tau3_weight, ...
    K3_brake, D3_brake, I3_brake, ...
    fric_comp_torque, deadzone_radius, ...
    boost_torque, boost_duration, ...
    phase1_playback_speed, phase2_playback_speed);

%% Extract data
% t = output_data(:,1);
% x = -output_data(:,12); % actual foot position in X (negative due to direction motors are mounted)
% y = output_data(:,13); % actual foot position in Y
% 
% xdes = -output_data(:,16); % desired foot position in X (negative due to direction motors are mounted)
% ydes = output_data(:,17); % desired foot position in Y