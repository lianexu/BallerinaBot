% This is the main MATLAB script for Lab 5.
%
% You will need to modify the Mbed code and this script, but should not need to make any other changes.
%
%% SET YOUR INPUTS HERE

num_points = 5;
% torque_control_period = 1000;
% K_body = 0.015;
% K_body = 0.05;

pts_torque1 = zeros(num_points);
pts_torque2 = zeros(num_points);
pts_torque3 = zeros(num_points);

% pts_torque1(1:end) = 5;
% pts_torque2(1:end) = 5;
% pts_torque3(1:end) = 5;
% pts_torque3(1:5) = -0.5;

% pts_torque2(1:5) = 10;
% for i=1:num_points-1
% pts_torque3(1:99) = 0.1; % Apply torques
% end
        
% Initial leg angles for encoder resets (negative of q1,q2 in lab handout due to direction motors are mounted)
angle1_init = -1.40848; % -0.581194; %-1.20428; 1.40848;
angle2_init = -0.1; %-0.780162; 
angle3_init = 0;

% Total experiment time is buffer,trajectory,buffer
traj_time         = 10;
pre_buffer_time   = 2; % this should be 0 for constant points, 2 for Bezier trajectories
post_buffer_time  = 2;


% Maximum duty cycle commanded by controller (should always be <=1.0)
duty_max   = 0.4; % 0.4;
% 

Kp_body = 20;
Kd_body = 100;
Ki_body = 0.02;
% Kp_body = 0.0;
% Kd_body = 0.0;
% Ki_body = 0.0;

    % k_p = 18.0;
    % k_d = 0.2;
    % k_i = 0.02;

%% Run Experiment
[output_data] = RunTrajectoryExperiment(angle1_init, angle2_init, angle3_init, pts_torque1, pts_torque2, pts_torque3,...
                                        traj_time, pre_buffer_time, post_buffer_time,...
                                        duty_max, Kp_body, Kd_body, Ki_body);

%% Extract data
% t = output_data(:,1);
% x = -output_data(:,12); % actual foot position in X (negative due to direction motors are mounted)
% y = output_data(:,13); % actual foot position in Y
% 
% xdes = -output_data(:,16); % desired foot position in X (negative due to direction motors are mounted)
% ydes = output_data(:,17); % desired foot position in Y