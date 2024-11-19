%% 2.740: Homework 5
% Setup
setupHW5();

%% Trajectory optimization with CasADi
% CasADi is a symbolic framework for algorithmic differentiation and numerical optimization.
% It's what our lab uses extensively to design controllers for locomotion, jumps, backflips, etc.
 
% The syntax is similar to MATLAB's symbolic toolbox you've been using for past homeworks, 
% but it's much more powerful and expressive for us to formulate optimizations.

% For this homework, we're going to optimize the trajectories of the leg you've been using for lab.
% You only need to work in this file, but look at setupHW5.m to familiarize yourself with how 
% we are building the dynamics of the leg, setting parameters, and adding libraries.

% If needed, you have access to the following functions for the leg
% A_fn(z, param)
% b_fn(z, u, param)
% energy_fn(z, param)
% pos_end_effector(z, param)
% vel_end_effector(z, param)
% J_end_effector(z, param)
% keypoints_fn(z, param)

%% [DECISION VARIABLES AND PARAMETERS]:
% q_dim = 2;                  % Number of generalized coordinates
% N = 20;                     % Size of optimization horizon
% dt = 0.025;                  % Discretization time step
% t_span = 0:dt:(N - 1)*dt;   % Time span
q_dim = 3; 
N1 = 10;
N2 = 10;
opti = casadi.Opti();

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% SECTION 2.1.1 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% We can add a decision variable of size (3, 2) like this:
% dv_1 = opti.variable(3, 2);
% Add in your decision variables for q, q_dot, and u here:
q1 = opti.variable(3,N1); % rotation of motors
q1_dot = opti.variable(3,N1); % angular velocity of motors
u1 = opti.variable(3,N1-1); % torque

q2 = opti.variable(3,N2); % rotation of motors
q2_dot = opti.variable(3,N2); % angular velocity of motors
u2 = opti.variable(3,N2-1); % torque

dt1 = opti.variable(1,1);
dt2 = opti.variable(1,1);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% SECTION 2.1.3 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Here are the inertial + kinematic paramters for the leg.
p = opti.parameter(20, 1);

% Add additional parameters for the initial conditions + state constraints + input constraints.
% Use "inf" or "-inf" if a state does not need to be constrained.
q1_0 = opti.parameter(3,1); 
q1_dot_0 = opti.parameter(3,1);
q_max = opti.parameter(2,1); 
q_min = opti.parameter(2,1);
tau_max = opti.parameter(1);
tau_min = opti.parameter(1);

%% [CONSTRAINTS]:
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% SECTION 2.2.1 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% We can constrain our optimization by like this:
% opti.subject_to(dv_1(:, 1) == [1, 2, 3]'); --> This constrains the first column of dv_1 to be [1, 2, 3]'.
% opti.subject_to(dv_1(:, 1) <= [1, 2, 3]'); --> This constrains the first column of dv_1 to be less than or equal to [1, 2, 3]'.
% Instead of [1, 2, 3]', you can constrain it to any expression (parameter or decision variable)
% on the right hand side. 

% negative = clockwise

opti.subject_to(dt1 >= 0);
opti.subject_to(dt2 >= 0);

%%% Phase 1 constraints
opti.subject_to(q1(:, 1) == q1_0);
opti.subject_to(q1_dot(:, 1) == q1_dot_0);

% opti.subject_to(q1(3, 2:end) == q1(3, 1));
% opti.subject_to(q1_dot(3, 2:end) == 0);
opti.subject_to(q1(3, 2:end) <= q1(3, 1) + 0.0001);
opti.subject_to(q1(3, 2:end) >= q1(3, 1) - 0.0001);

opti.subject_to(q1_dot(3, 2:end) == 0);

opti.subject_to(u1(3, :) <= 0);

for k = 1:N1 - 1
    z_k = [q1(:, k); q1_dot(:, k)];
    u_k = u1(:, k);
    qdd_k = A_fn(z_k, p) \ (b_fn(z_k, u_k, p));     % qdd = (M_inv) * (tau - C - G), our favorite equation
    opti.subject_to(q1_dot(:, k + 1) == q1_dot(:, k) + qdd_k * dt1);
    opti.subject_to(q1(:, k + 1) == q1(:, k) + q1_dot(:, k + 1) * dt1);

    % Add in min/max state constraints here:
    opti.subject_to(q1(1:2,k+1) >= q_min);
    opti.subject_to(q1(1:2,k+1) <= q_max);
    % Add in min/max input constraints here:
    opti.subject_to(u1(:,k) >= tau_min); 
    opti.subject_to(u1(:,k) <= tau_max);
end


%%% Phase 2 constraints
opti.subject_to(q2(:, 1) == q1(:,N1));
opti.subject_to(q2_dot(:, 1) == q1_dot(:,N1));
% opti.subject_to(q2(1:2, N2) == q1(1:2,1));
opti.subject_to(q2(3, N2) == q1(3,1) - 2*pi);

opti.subject_to(q2_dot(3, :) <= 0);

opti.subject_to(u2(3, :) == 0); % Body torque all 0 for phase 2

opti.subject_to(q2(1:2,1) >= q_min);
opti.subject_to(q2(1:2,1) <= q_max);
opti.subject_to(u2(:,1) >= tau_min); 
opti.subject_to(u2(:,1) <= tau_max);

for k = 1:N2 - 1
    z_k = [q2(:, k); q2_dot(:, k)];
    u_k = u2(:, k);
    qdd_k = A_fn(z_k, p) \ (b_fn(z_k, u_k, p));     % qdd = (M_inv) * (tau - C - G), our favorite equation
    opti.subject_to(q2_dot(:, k + 1) == q2_dot(:, k) + qdd_k * dt2);
    opti.subject_to(q2(:, k + 1) == q2(:, k) + q2_dot(:, k + 1) * dt2);

    % Add in min/max state constraints here:
    opti.subject_to(q2(1:2,k+1) >= q_min);
    opti.subject_to(q2(1:2,k+1) <= q_max);
    % Add in min/max input constraints here:
    opti.subject_to(u2(:,k) >= tau_min); 
    opti.subject_to(u2(:,k) <= tau_max);
end


%% [OBJECTIVE]:
% Remember, the cost should be SCALAR - check size(cost) to make sure it is
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% SECTION 2.3.1 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% We can add a cost like this:
% cost = dv_1(1, 1) + dv_1(2, 2);
% It's simply a symbolic expression of variables and parameters we've defined so far.

% Implement torque minimization cost here:
% cost_torque1 = u1_k'*u1_k; % [YOUR CODE HERE]
% cost_torque2 = u2_k'*u2_k; % [YOUR CODE HERE]
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% SECTION 2.3.2 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Implement the desired joint state at the end of the trajectory here:
% z_des = [pi/4; pi/4; 0; 0];
% z_end = [q(:, end); q_dot(:, end)];
% cost_joint = (z_end-z_des)'*(z_end-z_des); % [YOUR CODE HERE]

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% SECTION 2.3.3 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Implement the desired end effector state at the end of the trajectory here:
% cart_des = [0.05, -0.175, 5, 5]';
% cart_end = [pos_end_effector(z_end(1), params); vel_end_effector(z_end(2), params)]; % [YOUR CODE HERE];
% cost_cart = (cart_end-cart_des)'*(cart_end-cart_des); % [YOUR CODE HERE]

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% SECTION 2.3.4 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Fill in weights for each cost here, as desired:
% Q_torque = 0;
% Q_cart = 10;
% Q_joint = 1;
% cost = Q_torque*cost_torque + Q_cart*cost_cart + Q_joint*cost_joint;

cost = N1 * dt1 + N2 * dt2;%  + (q2(3, N2)-(q1(3,1) - 2*pi))^2;
opti.minimize(cost);

%% [SOLVER]:
opti.solver('ipopt');

%% [SET PARAMETER VALUES]:
z_0 = [-pi/4; pi/2; 0; 0; 0; 0];
opti.set_value(q1_0, z_0(1:3));
opti.set_value(q1_dot_0, z_0(4:6));
opti.set_value(p, params);
opti.set_value(q_max, q_max_val);
opti.set_value(q_min, q_min_val);
opti.set_value(tau_max, tau_max_val);
opti.set_value(tau_min, tau_min_val);


%% [SOLVE]:
soln = opti.solve();

q1_soln = soln.value(q1);
q1_dot_soln = soln.value(q1_dot);
z1_soln = [q1_soln; q1_dot_soln];
u1_soln = soln.value(u1);
dt1_soln = soln.value(dt1);

q2_soln = soln.value(q2);
q2_dot_soln = soln.value(q2_dot);
z2_soln = [q2_soln; q2_dot_soln];
u2_soln = soln.value(u2);
dt2_soln = soln.value(dt2);


t1_span = 0:dt1_soln:(N1-1)*dt1_soln;
t2_span = (N1-1)*dt1_soln:dt2_soln:(N2-1)*dt2_soln + (N1-1)*dt1_soln;

dt_sim = 0.001;
t1_sim = 0:dt_sim:t1_span(end);
t2_sim = t1_span(end):dt_sim:t2_span(end)+t1_span(end);
N1_sim = length(t1_sim);
N2_sim = length(t2_sim);

t_span = [t1_span t2_span];
z_soln = [z1_soln z2_soln];
q_soln = [q1_soln q2_soln];
% 
figure(3); clf; hold on;
title("Optimization animation");
animateBallTraj2(t_span, z_soln, params, dt1_soln, dt2_soln, N1, N2);


% % Interpolation schemes: {'nearest', 'linear', 'spline', 'pchip', 'cubic'}
u1_out = interpolateOptimizedControl(t1_span, u1_soln, t1_sim, 'spline');
u1_out(:, end - floor(dt1_soln/dt_sim):end) = 0;
z1_sim = zeros(6, N1_sim);
z1_sim(:,1) = z_0;
for i = 1:N1_sim-1
    A = full(A_fn(z1_sim(:, i), params));
    b = full(b_fn(z1_sim(:, i), u1_out(:, i), params));
    Fc = contact_force(z1_sim(:, i)); % idk
    % Fc = [0; 0; 0]; % comment this out?
    qdd = A\(b+Fc);
    z1_sim(4:6, i+1) = z1_sim(4:6, i) + qdd*dt_sim;
    z1_sim(1:3, i+1) = z1_sim(1:3,i) + z1_sim(4:6,i+1)*dt_sim;
end

u2_out = interpolateOptimizedControl(t2_span, u2_soln, t2_sim, 'spline');
u2_out(:, end - floor(dt1_soln/dt_sim):end) = 0;
z2_sim = zeros(6, N2_sim);
z2_sim(:,1) = z2_soln(:,1);
for i = 1:N2_sim-1
    A = full(A_fn(z2_sim(:, i), params));
    b = full(b_fn(z2_sim(:, i), u2_out(:, i), params));
    Fc = contact_force(z2_sim(:, i));
    % Fc = [0; 0; 0]; % comment this out?
    qdd = A\(b+Fc);
    z2_sim(4:6, i+1) = z2_sim(4:6, i) + qdd*dt_sim;
    z2_sim(1:3, i+1) = z2_sim(1:3,i) + z2_sim(4:6,i+1)*dt_sim;
end

t_sim = [t1_sim t2_sim];
z_sim = [z1_sim z2_sim];
% 
figure(4); clf; hold on;
title("Simulation animation");
animateBallerinaTrajectory(t_sim, z_sim, params, dt_sim);

figure(1); clf; hold on;
title("Joint trajectory");
plot(t_span, q_soln(1, :), 'r--');
plot(t_span, q_soln(2, :), 'b--');
% plot(t_sim, z_sim(1, :), 'r-');
% plot(t_sim, z_sim(2, :), 'b-');
% legend('q_1_{opt}', 'q_2_{opt}', 'q_1_{sim}', 'q_2_{sim}');
xlabel('Time (s)'); ylabel('q (rad)');


%% [PLOTS]:
% figure(1); clf; hold on;
% title("Joint trajectory");
% plot(t_span, q_soln(1, :), 'r--');
% plot(t_span, q_soln(2, :), 'b--');
% plot(t_sim, z_sim(1, :), 'r-');
% plot(t_sim, z_sim(2, :), 'b-');
% legend('q_1_{opt}', 'q_2_{opt}', 'q_1_{sim}', 'q_2_{sim}');
% xlabel('Time (s)'); ylabel('q (rad)');
% 
% figure(2); clf; hold on;
% title("Torque trajectory");
% stairs(t_span(1:end-1), u_soln(1, :), 'r--');
% stairs(t_span(1:end-1), u_soln(2, :), 'b--');
% stairs(t_sim, u_out(1, :), 'r-');
% stairs(t_sim, u_out(2, :), 'b-');
% legend('\tau_1_{opt}', '\tau_2_{opt}', '\tau_1_{sim}', '\tau_2_{sim}');
% xlabel('Time (s)'); ylabel('\tau (Nm)');

% figure(3); clf; hold on;
% title("Optimization animation");
% animateBallerinaTrajectory(t_span, z_soln, params, 0.1);

% figure(4); clf; hold on;
% title("Simulation animation");
% animateBallerinaTrajectory(t_sim, z_sim, params, 0.5);

function [u_interp] = interpolateOptimizedControl(t_span, u_soln, t_sim, interp_scheme)
    u_interp = [interp1(t_span(1:end-1), u_soln(1, :), t_sim, interp_scheme);
                interp1(t_span(1:end-1), u_soln(2, :), t_sim, interp_scheme);
                interp1(t_span(1:end-1), u_soln(3, :), t_sim, interp_scheme)];
end

function dz = dynamics(z,u, p)
    % Get mass matrix
    A = A_ballerina(z,p);
    
    % Get forces
    % u = [0 0 0]';
    b = b_ballerina(z,u,p);

    Fc = contact_force(z);

    
    % Solve for qdd
    qdd = A\(b+Fc);
    dz = 0*z;
    
    % Form dz
    dz(1:3) = z(4:6);
    dz(4:6) = qdd; %- 0.95*z(4:6); % is this how you apply damping??
end

function Fc = contact_force(z)  %same thing as the joint_limit_torque
% turn this into the joint limit, rather than a y-coordinate limit

    %% Fixed parameters for contact
    K_c = 10;
    D_c = 0.5;
    
    theta_1 = z(1); % hip 
    theta_2 = z(2); % knee

    theta_1_floor = theta_1 - -pi/2; % violated when < 0
    theta_1_ceiling = pi/3 - theta_1; % violated when < 0

    theta_2_floor = theta_2 - 0.3; % violated when < 0
    theta_2_ceiling = pi*0.8 - theta_2; % violated when < 0

    theta_1_dot = z(4);
    theta_2_dot = z(5);

    Fc = [0; 0; 0];
    if theta_1_floor < 0 && theta_1_dot < 0
        Fc(1) = -K_c * theta_1_floor - D_c * theta_1_dot;
    elseif theta_1_ceiling < 0 && theta_1_dot > 0
        Fc(1) = K_c * theta_1_ceiling - D_c * theta_1_dot;
    else
        Fc(1) = 0;
    end

    if theta_2_floor < 0 && theta_2_dot < 0
        Fc(2) = -K_c * theta_2_floor - D_c * theta_2_dot;
    elseif theta_2_ceiling < 0 && theta_2_dot > 0
        Fc(2) = K_c * theta_2_ceiling - D_c * theta_2_dot;
    else
        Fc(2) = 0;
    end

end