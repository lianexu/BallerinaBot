% goes with 274-final-V1

function output_data = RunTrajectoryExperiment( angle1_init, angle2_init, angle3_init, pts_torque1, pts_torque2, pts_torque3, traj_time, pre_buffer_time, post_buffer_time, duty_max, torque_control_period)
    
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
    
    % a2 = subplot(424);
    % h2 = plot([0],[0]);
    % h2.XData = []; h2.YData = [];
    % ylabel('Velocity 1 (rad/s)');
    % 
    % a6 = subplot(425);
    % h22 = plot([0],[0]);
    % h22.XData = []; h22.YData = [];
    % ylabel('Velocity 2 (rad/s)');
    % 
    % a10 = subplot(426);
    % h32 = plot([0],[0]);
    % h32.XData = []; h32.YData = [];
    % ylabel('Velocity 3 (rad/s)');
    
    % a3 = subplot(424);
    % h3 = plot([0],[0]);
    % h3.XData = []; h3.YData = [];
    % ylabel('Current 1 (A)');
    % hold on;
    % subplot(424);
    % h3 = plot([0],[0],'r');
    % h3.XData = []; h3.YData = [];
    % hold off;
    % 
    % a7 = subplot(425);
    % h23 = plot([0],[0]);
    % h23.XData = []; h23.YData = [];
    % ylabel('Current 2 (A)');
    % hold on;
    % subplot(425);
    % h24 = plot([0],[0],'r');
    % h24.XData = []; h24.YData = [];
    % hold off;
    % 
    % a11 = subplot(426);
    % h33 = plot([0],[0]);
    % h33.XData = []; h33.YData = [];
    % ylabel('Current 3 (A)');
    % hold on;
    % subplot(426);
    % h33 = plot([0],[0],'r');
    % h33.XData = []; h33.YData = [];
    % hold off;
    
    % a4 = subplot(430);
    % h5 = plot([0],[0]);
    % h5.XData = []; h5.YData = [];
    % ylabel('Duty Cycle 1');
    % 
    % a8 = subplot(431);
    % h25 = plot([0],[0]);
    % h25.XData = []; h25.YData = [];
    % ylabel('Duty Cycle 2');
    % 
    % a12 = subplot(432);
    % h35 = plot([0],[0]);
    % h35.XData = []; h35.YData = [];
    % ylabel('Duty Cycle 3');

    
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
        
        N = length(pos1);
        
        % Update motor data plots
        h1.XData(end+1:end+N) = t;   
        h1.YData(end+1:end+N) = -pos1; % switch sign on all plotted values due to direction motors are mounted
        % h2.XData(end+1:end+N) = t;   
        % h2.YData(end+1:end+N) = -vel1;
        % h3.XData(end+1:end+N) = t;   
        % h3.YData(end+1:end+N) = -cur1;
        % h4.XData(end+1:end+N) = t;   
        % h4.YData(end+1:end+N) = -dcur1;
        % h5.XData(end+1:end+N) = t;   
        % h5.YData(end+1:end+N) = -duty1;
        
        h21.XData(end+1:end+N) = t;   
        h21.YData(end+1:end+N) = -pos2;
        % h22.XData(end+1:end+N) = t;   
        % h22.YData(end+1:end+N) = -vel2;
        % h23.XData(end+1:end+N) = t;   
        % h23.YData(end+1:end+N) = -cur2;
        % h24.XData(end+1:end+N) = t;   
        % h24.YData(end+1:end+N) = -dcur2;
        % h25.XData(end+1:end+N) = t;   
        % h25.YData(end+1:end+N) = -duty2;

        h31.XData(end+1:end+N) = t;   
        h31.YData(end+1:end+N) = -pos3;
        % h32.XData(end+1:end+N) = t;   
        % h32.YData(end+1:end+N) = -vel3;
        % h33.XData(end+1:end+N) = t;   
        % h33.YData(end+1:end+N) = -cur3;
        % h34.XData(end+1:end+N) = t;   
        % h34.YData(end+1:end+N) = -dcur3;
        % h35.XData(end+1:end+N) = t;   
        % h35.YData(end+1:end+N) = -duty3;
        
        % Calculate leg state and update plots
        z = [pos1(end) pos2(end) pos3(end) vel1(end) vel2(end) vel3(end)]';
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
        set(hip_motor,'XData',[-0.05*cos(th3) 0.05*cos(th3)],'YData',[-0.05*sin(th3) 0.05*sin(th3)]);
        
    end
    
    frdm_ip  = '192.168.1.100';     % FRDM board ip
    frdm_port= 11223;               % FRDM board port  
    params.callback = @my_callback; % callback function
    %params.timeout  = 2;            % end of experiment timeout
    
    % Parameters for tuning
    start_period                = pre_buffer_time;    % In seconds 
    end_period                  = post_buffer_time;   % In seconds
    
    % Specify inputs
    input = [start_period traj_time end_period];
    input = [input angle1_init angle2_init angle3_init];
    input = [input duty_max torque_control_period];
    input = [input pts_torque1(:)' pts_torque2(:)' pts_torque3(:)']; % final size of input should be 28x1
    
    params.timeout  = (start_period+traj_time+end_period);  
    
    output_size = 16;    % number of outputs expected
    output_data = RunExperiment(frdm_ip,frdm_port,input,output_size,params);
    % linkaxes([a1 a3],'x')
    
end