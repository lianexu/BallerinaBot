% animate_ballerina(tspan, x, p, dt)

function simulate_ballerina()
    %% Definte fixed paramters
    m1 =.0393 + .2;         m2 =.0368; 
    m3 = .00783;            m4 = .0155;
    I1 = 25.1 * 10^-6;      I2 = 53.5 * 10^-6;
    I3 = 9.25 * 10^-6;      I4 = 22.176 * 10^-6; 
    I5 = 5 * 10^-6; % body
    l_OA=.011;              l_OB=.042; 
    l_AC=.096;              l_DE=.091;
    l_O_m1=0.032;           l_B_m2=0.0344; 
    l_A_m3=0.0622;          l_C_m4=0.0610;
    N = 18.75;
    Ir = 0.0035/N^2;
    g = 9.81;    
    
    %% Parameter vector
    p   = [m1 m2 m3 m4 I1 I2 I3 I4 I5 Ir N l_O_m1 l_B_m2 l_A_m3 l_C_m4 l_OA l_OB l_AC l_DE g]';
      

    %% Perform Dynamic simulation    
    dt = 0.001;
    tf = 6;
    num_steps = floor(tf/dt);
    tspan = linspace(0, tf, num_steps); 
    z0 = [-pi/2+0.2; 0.4; pi/2; 0; 0; 0]; % rotate, squish, rotate

    z_out = zeros(6,num_steps);
    z_out(:,1) = z0;

    u = zeros(3,num_steps);
    % u(1,1:1000) = 0.01; % Apply torques
    % u(2,2000:4000) = 0.1;
    % u(2,4000:5000) = -0.1;
    % u(2,4000:5000) = 0.08;
    u(3,1:500) = -0.05; % negative = clockwise

    for i=1:num_steps-1
        % z_out(:,i) = joint_limits(z_out(:,i));
        dz = dynamics(z_out(:,i), u(:,i), p);
        z_out(:,i+1) = z_out(:,i) + dz*dt;
    end
    % final_state = z_out(:,end);
    
    %% Compute Energy
    % E = energy_ballerina(z_out,p);

   

    % velplot = figure(2);
    % plot(tspan,z_out(6,:)));xlabel('Time (s)'); ylabel('Angular Velocity');

    figure(1)
    plot(tspan,z_out(6,:));


    animate_ballerina(tspan, z_out, p, dt);
end

% function z_edit = joint_limits(z)
%     z_edit = z;
%     if z(1) < -pi/2
%         z_edit(1) = -pi/2;
%         z_edit(4) = 0; 
%         % setting the velocity of that joint to 0. Maybe it should "bounce"
%         % instead of a perfectly inelastic collision???
%     end
%     if z(1) > pi/3
%         z_edit(1) = pi/3;
%         z_edit(4) = 0;
%     end
%     if z(2) < 0.3
%         z_edit(2) = 0.3;
%         z_edit(5) = 0;
%     end
% 
%     if z(2) > 2.51
%         z_edit(2) = 2.51;
%         z_edit(5) = 0;
%     end
% 
% end

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