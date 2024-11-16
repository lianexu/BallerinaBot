function [] = buildBallerinaDynamics()
    % q         = [th1  ; th2 ];      % generalized coordinates
    % q_dot     = [dth1 ; dth2];    % first time derivatives
    % q_ddot    = [ddth1;ddth2];  % second time derivatives
    % u         = [tau1 ; tau2];     % controls
    % F         = [Fx ; Fy];

    % Parameters
    % p         = [m1 m2 m3 m4 I1 I2 I3 I4 I5 Ir N l_O_m1 l_B_m2 l_A_m3 l_C_m4 l_OA l_OB l_AC l_DE g]';

    q = casadi.SX.sym('q', 3, 1); % changed to 3 x 1
    q_dot = casadi.SX.sym('q_dot', 3, 1); % changed to 3 x 1
    q_ddot = casadi.SX.sym('q_ddot', 3, 1);  % changed to 3 x 1
    u = casadi.SX.sym('u', 3, 1); % changed to 3 x 1
    F = casadi.SX.sym('F', 3, 1); % changed to 3 x 1
    par = casadi.SX.sym('p', 20, 1); %% changed to 20 because added I5

    par_mass = par(1:4);  % accounted for I5
    par_inertia = par(5:9); 
    par_inertia_rotor = par(10);
    par_gear_ratio = par(11);
    par_com_distances = par(12:15);
    par_lengths = par(16:19);
    par_gravity = par(20);
    

    i_hat = [1; 0; 0]; % edited to match diagram
    j_hat = [0; 1; 0];
    k_hat = cross(i_hat, j_hat);
    e1_hat =  sin(q(1)+q(3))*i_hat + -cos(q(1)+q(3))*j_hat; % edited to match diagram
    e2_hat =  sin(q(1)+q(2)+q(3))*i_hat + -cos(q(1)+q(2)+q(3))*j_hat;

    r_A = par_lengths(1) * e1_hat; % same as before
    r_B = par_lengths(2) * e1_hat;
    r_C = r_A + par_lengths(3) * e2_hat;
    r_D = r_B + par_lengths(3) * e2_hat;
    r_E = r_D + par_lengths(4) * e1_hat;

    r_m1 = par_com_distances(1) * e1_hat; % same as before
    r_m2 = r_B + par_com_distances(2) * e2_hat;
    r_m3 = r_A + par_com_distances(3) * e2_hat;
    r_m4 = r_C + par_com_distances(4) * e1_hat;

    r_A_dot = casadi_ddt(r_A, [q; q_dot], [q_dot; q_ddot]); % drA
    r_B_dot = casadi_ddt(r_B, [q; q_dot], [q_dot; q_ddot]);
    r_C_dot = casadi_ddt(r_C, [q; q_dot], [q_dot; q_ddot]);
    r_D_dot = casadi_ddt(r_D, [q; q_dot], [q_dot; q_ddot]);
    r_E_dot = casadi_ddt(r_E, [q; q_dot], [q_dot; q_ddot]);

    r_m1_dot = casadi_ddt(r_m1, [q; q_dot], [q_dot; q_ddot]); % dr_m1
    r_m2_dot = casadi_ddt(r_m2, [q; q_dot], [q_dot; q_ddot]);
    r_m3_dot = casadi_ddt(r_m3, [q; q_dot], [q_dot; q_ddot]);
    r_m4_dot = casadi_ddt(r_m4, [q; q_dot], [q_dot; q_ddot]);

    w_1 = q_dot(1) + q_dot(3); % omegas, add q_dot(3) to everything
    w_2 = q_dot(1) + q_dot(2) + q_dot(3);
    w_3 = q_dot(1) + q_dot(2) + q_dot(3);
    w_4 = q_dot(1) + q_dot(3);
    w_5 = q_dot(3);

    T_1 = 0.5 * par_mass(1) * dot(r_m1_dot, r_m1_dot) + 0.5 * par_inertia(1) * w_1^2;
    T_2 = 0.5 * par_mass(2) * dot(r_m2_dot, r_m2_dot) + 0.5 * par_inertia(2) * w_2^2;
    T_3 = 0.5 * par_mass(3) * dot(r_m3_dot, r_m3_dot) + 0.5 * par_inertia(3) * w_3^2;
    T_4 = 0.5 * par_mass(4) * dot(r_m4_dot, r_m4_dot) + 0.5 * par_inertia(4) * w_4^2;
    T_5 =  0.5 * par_inertia(5) * w_5^2;


    T_1_rotor = 0.5 * par_inertia_rotor * (q_dot(3) + par_gear_ratio * q_dot(1))^2;
    T_2_rotor = 0.5 * par_inertia_rotor * (q_dot(3) + q_dot(1) + par_gear_ratio * q_dot(2))^2;
    T_3_rotor = 0.5 * par_inertia_rotor * (par_gear_ratio * q_dot(3))^2;
    

    T = simplify(T_1 + T_2 + T_3 + T_4 + T_5 + T_1_rotor + T_2_rotor + T_3_rotor);
    V = 0; % simplify(0+0);

    Q_tau1 = casadi_M2Q(u(1) * k_hat, w_1 * k_hat, q_dot);
    Q_tau2 = casadi_M2Q(u(2) * k_hat, w_2 * k_hat, q_dot);
    Q_tau3 = casadi_M2Q(u(3) * k_hat, w_5 * k_hat, q_dot);

    Q_tau2_reaction = casadi_M2Q(-u(2) * k_hat, w_1 * k_hat, q_dot); % reaction moment of 2 on 1
    Q_tau3_reaction = casadi_M2Q(-u(1) * k_hat, w_5 * k_hat, q_dot); % reaction moment of 1 on 3
  

    Q = simplify(Q_tau1 + Q_tau2 + Q_tau3 + Q_tau2_reaction + Q_tau3_reaction);


    keypoints = [r_A(1:2), r_B(1:2), r_C(1:2), r_D(1:2), r_E(1:2)];

    E = simplify(T + V);
    L = simplify(T - V);
    EoM = casadi_ddt(jacobian(L, q_dot).', [q; q_dot], [q_dot; q_ddot]) - jacobian(L, q).' - Q;
    EoM_leg = casadi.Function('EoM_leg', {q, q_dot, q_ddot, u, par}, {EoM});

    A = (jacobian(EoM, q_ddot));
    b = -EoM_leg(q, q_dot, 0, u, par);
    J = jacobian(r_E, q); J = J(1:2, 1:2);
    z = [q; q_dot];

    A_fn = casadi.Function('A_fn', {z, par}, {A});
    b_fn = casadi.Function('b_fn', {z, u, par}, {b});
    energy_fn = casadi.Function('energy_fn', {z, par}, {E});
    pos_end_effector = casadi.Function('pos_end_effector', {z, par}, {r_E(1:2)});
    vel_end_effector = casadi.Function('vel_end_effector', {z, par}, {r_E_dot(1:2)});
    J_end_effector = casadi.Function('J_end_effector', {z, par}, {J});
    keypoints_fn = casadi.Function('keypoints_fn', {z, par}, {keypoints});
    
    A_fn.save('codegen/A_fn.casadi');
    b_fn.save('codegen/b_fn.casadi');
    energy_fn.save('codegen/energy_fn.casadi');
    pos_end_effector.save('codegen/pos_end_effector.casadi');
    vel_end_effector.save('codegen/vel_end_effector.casadi');
    J_end_effector.save('codegen/J_end_effector.casadi');
    keypoints_fn.save('codegen/keypoints_fn.casadi');
end