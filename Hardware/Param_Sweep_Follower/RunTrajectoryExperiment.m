% goes with 274-final-V1

function output_data = RunTrajectoryExperiment( ...
    traj_time, pre_buffer_time, post_buffer_time, duty_max, ...
    angle1_init, angle2_init, angle3_init,...
    tau1_weight, tau2_weight, tau3_weight, ...
    K3_brake, D3_brake, I3_brake, ...
    fric_comp_torque, deadzone_radius, ...
    q1_max, q2_min, ...
    q2_retract_based_on_q1_angle, q2_retract_q1_angle, T_q2_in, ...
    brake_based_on_q1_angle, brake_release_q1_angle, T_brake, ...
    T_end)

    % Figure for plotting motor data
    figure(1);  clf;       
    a1 = subplot(421);
    h1 = plot([0],[0]);
    h1.XData = []; h1.YData = [];
    ylabel('Angle 1 (rad)');

    a5 = subplot(422);
    h21 = plot([0],[0]);
    h21.XData = []; h21.YData = [];
    ylabel('Angle 2 (rad)');

    a9 = subplot(423);
    h31 = plot([0],[0]);
    h31.XData = []; h31.YData = [];
    ylabel('Angle 3 (rad)');
    
    % Figure for plotting state of the leg
    figure(2)
    clf
    hold on
    axis equal
    axis([-.35 .35 -.35 .1]);
   
    h_OB = plot([0],[0],'LineWidth',2);
    h_AC = plot([0],[0],'LineWidth',2);
    h_BD = plot([0],[0],'LineWidth',2);
    h_CE = plot([0],[0],'LineWidth',2);
    hip_motor = plot([0],[0], 'LineWidth', 3);
    % limits1 =  plot([0],[0], 'LineWidth', 2, 'LineStyle',':', 'Color', 'r'); % draw the hardware limits in red
    % limits2 =  plot([0],[0], 'LineWidth', 2, 'LineStyle', ':', 'Color', 'r'); % draw the hardware limits in red

    
    % Define leg length parameters
    m1 =.0393 + .2;         m2 =.0368; 
    m3 = .00783;            m4 = .0155;
    I1 = 25.1 * 10^-6;      I2 = 53.5 * 10^-6;
    I3 = 9.25 * 10^-6;      I4 = 22.176 * 10^-6;
    I5 = 90 * 10^-6;
    l_OA=.011;              l_OB=.042; 
    l_AC=.096;              l_DE=.091;
    l_O_m1=0.032;           l_B_m2=0.0344; 
    l_A_m3=0.0622;          l_C_m4=0.0610;
    Nmot = 18.75;
    Ir = 0.0035/Nmot^2;

    p   = [l_OA l_OB l_AC l_DE];
    p   = [p m1 m2 m3 m4 I1 I2 I3 I4 I5 Ir Nmot l_O_m1 l_B_m2 l_A_m3 l_C_m4]';
    % p   = [m1 m2 m3 m4 I1 I2 I3 I4 I5 Ir N l_O_m1 l_B_m2 l_A_m3 l_C_m4 l_OA l_OB l_AC l_DE g]';  
    
    % This function will get called any time there is new data from
    % the Nucleo board. Data comes in blocks, rather than one at a time.
    function my_callback(new_data)
        % Parse new data
        t = new_data(:,1);          % time
        pos1 = new_data(:,2);       % position
        vel1 = new_data(:,3);       % velocity
        cur1 = new_data(:,4);       % current
        dcur1 = new_data(:,5);      % desired current
        duty1 = new_data(:,6);      % command
        
        pos2 = new_data(:,7);       % position
        vel2 = new_data(:,8);       % velocity
        cur2 = new_data(:,9);       % current
        dcur2 = new_data(:,10);     % desired current
        duty2 = new_data(:,11);     % command

        pos3 = new_data(:,12);       % position
        vel3 = new_data(:,13);       % velocity
        cur3 = new_data(:,14);       % current
        dcur3 = new_data(:,15);     % desired current
        duty3 = new_data(:,16);     % command  

        tau1 = new_data(:,17);
        tau2 = new_data(:,18);
        tau3 = new_data(:,19);

        
        N = length(pos1);
        
        % Update motor data plots
        h1.XData(end+1:end+N) = t;   
        h1.YData(end+1:end+N) = pos1; % switch sign on all plotted values due to direction motors are mounted

        
        h21.XData(end+1:end+N) = t;   
        h21.YData(end+1:end+N) = pos2;


        h31.XData(end+1:end+N) = t;   
        h31.YData(end+1:end+N) = pos3;
        
        % Calculate leg state and update plots
        z = [-pos1(end) -pos2(end) -pos3(end) -vel1(end) -vel2(end) -vel3(end)]';
        keypoints = keypoints_ballerina(z,p);
        
        % TODO: could also plot Jacobian, control force vector here?
        
        rA = keypoints(:,1); 
        rB = keypoints(:,2);
        rC = keypoints(:,3);
        rD = keypoints(:,4);
        rE = keypoints(:,5);
        th3 = z(3);

        set(h_OB,'XData',[0 rB(1)],'YData',[0 rB(2)]);
        set(h_AC,'XData',[rA(1) rC(1)],'YData',[rA(2) rC(2)]);
        set(h_BD,'XData',[rB(1) rD(1)],'YData',[rB(2) rD(2)]);
        set(h_CE,'XData',[rC(1) rE(1)],'YData',[rC(2) rE(2)]);
        set(hip_motor,'XData',[0 0.05*sin(th3)]);
        set(hip_motor,'YData',[0 -0.05*cos(th3)]);
        
    end
    
    frdm_ip  = '192.168.1.100';     % FRDM board ip
    frdm_port= 11223;               % FRDM board port  
    params.callback = @my_callback; % callback function
    %params.timeout  = 2;            % end of experiment timeout
    
    % Parameters for tuning
    start_period                = pre_buffer_time;    % In seconds 
    end_period                  = post_buffer_time;   % In seconds
    
    % Specify inputs
    input = [start_period traj_time end_period duty_max];
    input = [input angle1_init angle2_init angle3_init];
    input = [input tau1_weight tau2_weight tau3_weight];
    input = [input K3_brake D3_brake I3_brake];
    input = [input fric_comp_torque deadzone_radius];
    input = [input q1_max q2_min];
    input = [input q2_retract_based_on_q1_angle q2_retract_q1_angle T_q2_in];
    input = [input brake_based_on_q1_angle brake_release_q1_angle T_brake];
    input = [input T_end];
    
    params.timeout  = (start_period+traj_time+end_period);  
    
    output_size = 19;    % number of outputs expected
    output_data = RunExperiment(frdm_ip,frdm_port,input,output_size,params);
    % linkaxes([a1 a3],'x')
    
end