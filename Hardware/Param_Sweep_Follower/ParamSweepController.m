clear all;
% This is the main MATLAB script for Lab 5.
% You will need to modify the Mbed code and this script, but should not need to make any other changes.

% Total experiment time is buffer,trajectory,buffer
traj_time         = 5;
pre_buffer_time   = 2; % this should be 0 for constant points, 2 for Bezier trajectories
post_buffer_time  = 2;

% Maximum duty cycle commanded by controller (should always be <=1.0)
duty_max   = 0.65; 

% Initial leg angles for encoder resets (negative of q1,q2 in lab handout due to direction motors are mounted)
angle1_init = -pi/3; % = -1.0471975512
angle2_init = -0.3; 
angle3_init = 0;

% For braking (momtor 3) Runs in the faster, current control loop
K3_brake = 30;
D3_brake = 100;
I3_brake = 0.02;

% friction comp
deadzone_radius = -0.3; % set < 0 to turn off friction comp
fric_comp_torque = 0.030; 

% how fast the motors draw in
tau1_weight = 1.0; % how fast tau1 draws in
tau2_weight = -1.0;
tau3_weight = 0.0;

q1_max = 1.73; % when to stop whipping the leg
q2_min = -2.2; % when to stop drawing in the knee

brake_based_on_q1_angle = false;
brake_release_q1_angle = 1.7; %1.725 max
T_brake = 0.6;


q2_retract_based_on_q1_angle = false;
q2_retract_q1_angle = 1.5;
T_q2_in = 0.4; % -1.04 to 1.725

if brake_based_on_q1_angle
    brake_name = num2str(brake_release_q1_angle);
else
    brake_name = num2str(T_brake);
end

if q2_retract_based_on_q1_angle
    retract_name =  num2str(q2_retract_q1_angle);
else
    retract_name = num2str(T_q2_in);
end



T_end = 2.0;


%% Run Experiment
[output_data] = RunTrajectoryExperiment(traj_time, pre_buffer_time, post_buffer_time, duty_max, ...
    angle1_init, angle2_init, angle3_init,...
    tau1_weight, tau2_weight, tau3_weight, ...
    K3_brake, D3_brake, I3_brake, ...
    fric_comp_torque, deadzone_radius, ...
    q1_max, q2_min, ...
    q2_retract_based_on_q1_angle, q2_retract_q1_angle, T_q2_in, ...
    brake_based_on_q1_angle, brake_release_q1_angle, T_brake, ...
    T_end);

%% Extract data
% Idk if I got the negative signs correct
t = output_data(:,1);          % time
pos1 = output_data(:,2);       % position
vel1 = -output_data(:,3);       % velocity
cur1 = -output_data(:,4);       % current
dcur1 = -output_data(:,5);      % desired current
duty1 = -output_data(:,6);      % command

pos2 = output_data(:,7);       % position
vel2 = -output_data(:,8);       % velocity
cur2 = -output_data(:,9);       % current
dcur2 = -output_data(:,10);     % desired current
duty2 = -output_data(:,11);     % command

pos3 = output_data(:,12);       % position
vel3 = output_data(:,13);       % velocity
cur3 = -output_data(:,14);       % current
dcur3 = -output_data(:,15);     % desired current
duty3 = -output_data(:,16);     % command  

tau1 = output_data(:,17);
tau2 = output_data(:,18);
tau3 = output_data(:,19);


pos3(end)

%% Display / save the run
figure(1);
hold on;
plot(t, pos1, 'r.', 'DisplayName', 'pos1');
plot(t, pos2, 'g.', 'DisplayName', 'pos2');
plot(t, pos3, 'b.', 'DisplayName', 'pos3');


name = strcat(string(brake_based_on_q1_angle), ",", brake_name, ",", string(q2_retract_based_on_q1_angle), ",", retract_name, ",", string(pos3(end)));
save(name);