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
q_dim = 2;                  % Number of generalized coordinates
N = 20;                     % Size of optimization horizon
dt = 0.025;                  % Discretization time step
t_span = 0:dt:(N - 1)*dt;   % Time span

opti = casadi.Opti();

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% SECTION 2.1.1 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% We can add a decision variable of size (3, 2) like this:
% dv_1 = opti.variable(3, 2);
% Add in your decision variables for q, q_dot, and u here:
q = opti.variable(2,20);% [YOUR CODE HERE] rotation of motors
q_dot = opti.variable(2,20);% [YOUR CODE HERE] angular velocity of motors
u = opti.variable(2,19); % [YOUR CODE HERE] torque

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% SECTION 2.1.3 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Here are the inertial + kinematic paramters for the leg.
p = opti.parameter(19, 1);

% Add additional parameters for the initial conditions + state constraints + input constraints.
% Use "inf" or "-inf" if a state does not need to be constrained.
q_0 = opti.parameter(2,1); % [YOUR CODE HERE]
q_dot_0 = opti.parameter(2,1); % [YOUR CODE HERE]
q_max = opti.parameter(2,1); % [YOUR CODE HERE]
q_min = opti.parameter(2,1);% [YOUR CODE HERE]
tau_max = opti.parameter(1); % [YOUR CODE HERE]
tau_min = opti.parameter(1); % [YOUR CODE HERE]

%% [CONSTRAINTS]:
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% SECTION 2.2.1 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% We can constrain our optimization by like this:
% opti.subject_to(dv_1(:, 1) == [1, 2, 3]'); --> This constrains the first column of dv_1 to be [1, 2, 3]'.
% opti.subject_to(dv_1(:, 1) <= [1, 2, 3]'); --> This constrains the first column of dv_1 to be less than or equal to [1, 2, 3]'.
% Instead of [1, 2, 3]', you can constrain it to any expression (parameter or decision variable)
% on the right hand side. 

% Now, implement the initial condition constraint here:
% [YOUR CODE HERE]
opti.subject_to(q(:, 1) == q_0);
opti.subject_to(q_dot(:, 1) == q_dot_0);


for k = 1:N - 1
    % The dynamics constraints are below. I didn't want to make this part of the homework, but 
    % I hope you recognize the semi-implicit Euler integration in lines 57-58. 
    z_k = [q(:, k); q_dot(:, k)];
    u_k = u(:, k);
    qdd_k = A_fn(z_k, p) \ (b_fn(z_k, u_k, p));     % qdd = (M_inv) * (tau - C - G), our favorite equation
    opti.subject_to(q_dot(:, k + 1) == q_dot(:, k) + qdd_k * dt);
    opti.subject_to(q(:, k + 1) == q(:, k) + q_dot(:, k + 1) * dt);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% SECTION 2.2.2 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Add in min/max state constraints here:
    % [YOUR CODE HERE]
    opti.subject_to(q(:,k+1) >= q_min);
    opti.subject_to(q(:,k+1) <= q_max);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% SECTION 2.2.3 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Add in min/max input constraints here:
    % [YOUR CODE HERE]
    opti.subject_to(u(:,k) >= tau_min); 
    opti.subject_to(u(:,k) <= tau_max);

end


%% [OBJECTIVE]:
% Remember, the cost should be SCALAR - check size(cost) to make sure it is
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% SECTION 2.3.1 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% We can add a cost like this:
% cost = dv_1(1, 1) + dv_1(2, 2);
% It's simply a symbolic expression of variables and parameters we've defined so far.

% Implement torque minimization cost here:
cost_torque = u_k'*u_k; % [YOUR CODE HERE]

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% SECTION 2.3.2 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Implement the desired joint state at the end of the trajectory here:
z_des = [pi/4; pi/4; 0; 0];
z_end = [q(:, end); q_dot(:, end)];
cost_joint = (z_end-z_des)'*(z_end-z_des); % [YOUR CODE HERE]

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% SECTION 2.3.3 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Implement the desired end effector state at the end of the trajectory here:
cart_des = [0.05, -0.175, 5, 5]';
cart_end = [pos_end_effector(z_end(1), params); vel_end_effector(z_end(2), params)]; % [YOUR CODE HERE];
cost_cart = (cart_end-cart_des)'*(cart_end-cart_des); % [YOUR CODE HERE]

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% SECTION 2.3.4 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Fill in weights for each cost here, as desired:
Q_torque = 0;
Q_cart = 10;
Q_joint = 1;
cost = Q_torque*cost_torque + Q_cart*cost_cart + Q_joint*cost_joint;
opti.minimize(cost);

%% [SOLVER]:
opti.solver('ipopt');

%% [SET PARAMETER VALUES]:
z_0 = [-pi/4; pi/2; 0; 0];
opti.set_value(q_0, z_0(1:2));
opti.set_value(q_dot_0, z_0(3:4));
opti.set_value(p, params);
opti.set_value(q_max, q_max_val);
opti.set_value(q_min, q_min_val);
opti.set_value(tau_max, tau_max_val);
opti.set_value(tau_min, tau_min_val);


%% [SOLVE]:
soln = opti.solve();
q_soln = soln.value(q);
q_dot_soln = soln.value(q_dot);
z_soln = [q_soln; q_dot_soln];
u_soln = soln.value(u);

%% [SIMULATE]:
dt_sim = 0.001;
t_sim = 0:dt_sim:t_span(end);
N_sim = length(t_sim);

% Interpolation schemes: {'nearest', 'linear', 'spline', 'pchip', 'cubic'}
u_out = interpolateOptimizedControl(t_span, u_soln, t_sim, 'cubic');
u_out(:, end - floor(dt/dt_sim):end) = 0;
z_sim = zeros(4, N_sim);
z_sim(:,1) = z_0;
for i = 1:N_sim-1
    A = full(A_fn(z_sim(:, i), params));
    b = full(b_fn(z_sim(:, i), u_out(:, i), params));
    qdd = A\(b);
    z_sim(3:4, i+1) = z_sim(3:4, i) + qdd*dt_sim;
    z_sim(1:2, i+1) = z_sim(1:2,i) + z_sim(3:4,i+1)*dt_sim;
end

%% [PLOTS]:
figure(1); clf; hold on;
title("Joint trajectory");
plot(t_span, q_soln(1, :), 'r--');
plot(t_span, q_soln(2, :), 'b--');
plot(t_sim, z_sim(1, :), 'r-');
plot(t_sim, z_sim(2, :), 'b-');
legend('q_1_{opt}', 'q_2_{opt}', 'q_1_{sim}', 'q_2_{sim}');
xlabel('Time (s)'); ylabel('q (rad)');

figure(2); clf; hold on;
title("Torque trajectory");
stairs(t_span(1:end-1), u_soln(1, :), 'r--');
stairs(t_span(1:end-1), u_soln(2, :), 'b--');
stairs(t_sim, u_out(1, :), 'r-');
stairs(t_sim, u_out(2, :), 'b-');
legend('\tau_1_{opt}', '\tau_2_{opt}', '\tau_1_{sim}', '\tau_2_{sim}');
xlabel('Time (s)'); ylabel('\tau (Nm)');

figure(3); clf; hold on;
title("Optimization animation");
animateTrajectory(t_span, z_soln, params, dt);

figure(4); clf; hold on;
title("Simulation animation");
animateTrajectory(t_sim, z_sim, params, dt_sim);

function [u_interp] = interpolateOptimizedControl(t_span, u_soln, t_sim, interp_scheme)
    u_interp = [interp1(t_span(1:end-1), u_soln(1, :), t_sim, interp_scheme);
                interp1(t_span(1:end-1), u_soln(2, :), t_sim, interp_scheme)];
end