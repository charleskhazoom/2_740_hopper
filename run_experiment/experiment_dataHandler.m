function output_data = experiment_dataHandler(time_param, q0, gains, control_method,duty_max, torque_profile, q_profile, qd_profile, hip_flight, knee_flight, arm_flight)
    
    % Figure for plotting motor data
    figure(1);  clf;       
    a1 = subplot(4,3,1);
    h1 = plot([0],[0]);
    h1.XData = []; h1.YData = [];
    ylabel('Angle 1 (rad)');
    hold on;
    subplot(4,3,1);
    h6 = plot([0],[0],'r');
    h6.XData = []; h6.YData = [];
    hold off;
    
    a2 = subplot(4,3,4);
    h2 = plot([0],[0]);
    h2.XData = []; h2.YData = [];
    ylabel('Velocity 1 (rad/s)');
    hold on;
    subplot(4,3,4);
    h7 = plot([0],[0],'r');
    h7.XData = []; h7.YData = [];
    hold off;
    
    a3 = subplot(4,3,7);
    h3 = plot([0],[0]);
    h3.XData = []; h3.YData = [];
    ylabel('Current 1 (A)');
    hold on;
    subplot(4,3,7);
    h4 = plot([0],[0],'r');
    h4.XData = []; h4.YData = [];
    hold off;
    
    a4 = subplot(4,3,10);
    h5 = plot([0],[0]);
    h5.XData = []; h5.YData = [];
    ylabel('Duty Cycle 1');
    

    a5 = subplot(4,3,2);
    h21 = plot([0],[0]);
    h21.XData = []; h21.YData = [];
    ylabel('Angle 2 (rad)');
    hold on;
    subplot(4,3,2);
    h26 = plot([0],[0],'r');
    h26.XData = []; h26.YData = [];
    hold off;
    
    a6 = subplot(4,3,5);
    h22 = plot([0],[0]);
    h22.XData = []; h22.YData = [];
    ylabel('Velocity 2 (rad/s)');
    hold on;
    subplot(4,3,5);
    h27 = plot([0],[0],'r');
    h27.XData = []; h27.YData = [];
    hold off;
    
    a7 = subplot(4,3,8);
    h23 = plot([0],[0]);
    h23.XData = []; h23.YData = [];
    ylabel('Current 2 (A)');
    hold on;
    subplot(4,3,8);
    h24 = plot([0],[0],'r');
    h24.XData = []; h24.YData = [];
    hold off;
    
    a8 = subplot(4,3,11);
    h25 = plot([0],[0]);
    h25.XData = []; h25.YData = [];
    ylabel('Duty Cycle 2');
         
    a9 = subplot(4,3,3);
    h31 = plot([0],[0]);
    h31.XData = []; h31.YData = [];
    ylabel('Angle 3 (rad)');
    hold on;
    subplot(4,3,3);
    h36 = plot([0],[0],'r');
    h36.XData = []; h36.YData = [];
    hold off;
    
    a10 = subplot(4,3,6);
    h32 = plot([0],[0]);
    h32.XData = []; h32.YData = [];
    ylabel('Velocity 3 (rad/s)');
    hold on;
    subplot(4,3,6);
    h37 = plot([0],[0],'r');
    h37.XData = []; h37.YData = [];
    hold off;
    
    a11 = subplot(4,3,9);
    h33 = plot([0],[0]);
    h33.XData = []; h33.YData = [];
    ylabel('Current 3 (A)');
    hold on;
    subplot(4,3,9);
    h34 = plot([0],[0],'r');
    h34.XData = []; h34.YData = [];
    hold off;
    
    a12 = subplot(4,3,12);
    h35 = plot([0],[0]);
    h35.XData = []; h35.YData = [];
    ylabel('Duty Cycle 3');
    
    
    % This function will get called any time there is new data from
    % the Nucleo board. Data comes in blocks, rather than one at a time.
    function my_callback(new_data)
        % Parse new data
        t = new_data(:,1);          % time
        pos1 = new_data(:,2);       % position
        pos1_des = new_data(:,25);  % desired position
        vel1 = new_data(:,3);       % velocity
        vel1_des = new_data(:,28);  % desired velocity
        cur1 = new_data(:,4);       % current
        dcur1 = new_data(:,5);      % desired current
        duty1 = new_data(:,6);      % command
        
        pos2 = new_data(:,7);       % position
        pos2_des = new_data(:,26);  % desired position
        vel2 = new_data(:,8);       % velocity
        vel2_des = new_data(:,29);  % desired velocity
        cur2 = new_data(:,9);       % current
        dcur2 = new_data(:,10);     % desired current
        duty2 = new_data(:,11);     % command
        
        pos3 = new_data(:,12);      % position
        pos3_des = new_data(:,27);  % desired position
        vel3 = new_data(:,13);      % velocity
        vel3_des = new_data(:,30);  % desired velocity
        cur3 = new_data(:,14);      % current
        dcur3 = new_data(:,15);     % desired current
        duty3 = new_data(:,16);     % command
        
        x_foot = -new_data(:,17);         % actual foot position (negative due to direction motors are mounted)
        y_foot = new_data(:,18);          % actual foot position
        x_arm  = new_data(:,21);          % actual arm position
        y_arm  = new_data(:,22);          % actual arm position
        
        N = length(pos1);
        
        % Update motor data plots
        h1.XData(end+1:end+N) = t;   
        h1.YData(end+1:end+N) = -pos1; % switch sign on all plotted values due to direction motors are mounted
        h2.XData(end+1:end+N) = t;   
        h2.YData(end+1:end+N) = -vel1;
        h3.XData(end+1:end+N) = t;   
        h3.YData(end+1:end+N) = -cur1;
        h4.XData(end+1:end+N) = t;   
        h4.YData(end+1:end+N) = -dcur1;
        h5.XData(end+1:end+N) = t;   
        h5.YData(end+1:end+N) = -duty1;
        h6.XData(end+1:end+N) = t;
        h6.YData(end+1:end+N) = -pos1_des;
        h7.XData(end+1:end+N) = t;
        h7.YData(end+1:end+N) = -vel1_des;
        
        h21.XData(end+1:end+N) = t;   
        h21.YData(end+1:end+N) = -pos2;
        h22.XData(end+1:end+N) = t;   
        h22.YData(end+1:end+N) = -vel2;
        h23.XData(end+1:end+N) = t;   
        h23.YData(end+1:end+N) = -cur2;
        h24.XData(end+1:end+N) = t;   
        h24.YData(end+1:end+N) = -dcur2;
        h25.XData(end+1:end+N) = t;   
        h25.YData(end+1:end+N) = -duty2;
        h26.XData(end+1:end+N) = t;
        h26.YData(end+1:end+N) = -pos2_des;
        h27.XData(end+1:end+N) = t;
        h27.YData(end+1:end+N) = vel2_des;
        
        h31.XData(end+1:end+N) = t;   
        h31.YData(end+1:end+N) = -pos3;
        h32.XData(end+1:end+N) = t;   
        h32.YData(end+1:end+N) = -vel3;
        h33.XData(end+1:end+N) = t;   
        h33.YData(end+1:end+N) = -cur3;
        h34.XData(end+1:end+N) = t;   
        h34.YData(end+1:end+N) = -dcur3;
        h35.XData(end+1:end+N) = t;   
        h35.YData(end+1:end+N) = -duty3;
        h36.XData(end+1:end+N) = t;
        h36.YData(end+1:end+N) = -pos3_des;
        h37.XData(end+1:end+N) = t;
        h37.YData(end+1:end+N) = vel3_des;
        
    end
    
    frdm_ip  = '192.168.1.100';     % FRDM board ip
    frdm_port= 11223;               % FRDM board port  
    params.callback = @my_callback; % callback function
    %params.timeout  = 2;            % end of experiment timeout 
    
    % Parameters for tuning
    traj_time                   = time_param(1);    % In seconds
    start_period                = time_param(2);    % In seconds 
    end_period                  = time_param(3);    % In seconds

    % Gains
    gains = [gains.K_q1 gains.K_q2 gains.K_q3 gains.D_qd1 gains.D_qd2 gains.D_qd3]';
    
    % Specify inputs
    input = [start_period end_period traj_time];
    input = [input q0'];
    input = [input gains'];
    input = [input hip_flight knee_flight arm_flight];
    input = [input control_method duty_max];
    input = [input torque_profile(:)' q_profile(:)' qd_profile(:)'];
    
    params.timeout  = (start_period+traj_time+end_period);  
    
    output_size = 30;    % number of outputs expected
    output_data = experiment_interface(frdm_ip,frdm_port,input,output_size,params);
    linkaxes([a1 a2 a3 a4],'x')
    
end

    
        % for now, let's not worry about animating the jumping leg
        
%     figure(2)
%     clf
%     hold on
%     axis equal
%     axis([-.25 .25 -.25 .1]);
%    
%     h_OB = plot([0],[0],'LineWidth',2);
%     h_AC = plot([0],[0],'LineWidth',2);
%     h_BD = plot([0],[0],'LineWidth',2);
%     h_CE = plot([0],[0],'LineWidth',2);
%     
%     h_foot= plot([0],[0],'k');
%     h_des = plot([0],[0],'k--');
%     h_des.XData=[];
%     h_des.YData=[];
%     h_foot.XData=[];
%     h_foot.YData=[];
%     
%     % Define leg length parameters
%     l_OA = 0.011; 
%     l_OB = 0.042; 
%     l_AC = 0.096; 
%     l_DE = 0.091;
% 
%     p   = [l_OA l_OB l_AC l_DE]';

%         z = [pos1(end) pos2(end) vel1(end) vel2(end)]';
%         keypoints = keypoints_leg(z,p);
%         
%         rA = keypoints(:,1); 
%         rB = keypoints(:,2);
%         rC = keypoints(:,3);
%         rD = keypoints(:,4);
%         rE = keypoints(:,5);
% 
%         set(h_OB,'XData',[0 rB(1)],'YData',[0 rB(2)]);
%         set(h_AC,'XData',[rA(1) rC(1)],'YData',[rA(2) rC(2)]);
%         set(h_BD,'XData',[rB(1) rD(1)],'YData',[rB(2) rD(2)]);
%         set(h_CE,'XData',[rC(1) rE(1)],'YData',[rC(2) rE(2)]);
%         
%         h_foot.XData(end+1:end+N) = x_foot;
%         h_foot.YData(end+1:end+N) = y_foot;
%         h_des.XData(end+1:end+N) = xdes;
%         h_des.YData(end+1:end+N) = ydes;
