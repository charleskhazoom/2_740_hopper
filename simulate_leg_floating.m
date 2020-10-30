function simulate_leg_floating()
path_sym_dynamics = 'dynamics_gen/';
addpath(path_sym_dynamics)
%% description of the variables
% q   = [x;y;th1  ; th2 ; th3];      % generalized coordinates
% dq  = [dx;dy;dth1 ; dth2; dth3];    % first time derivatives
% ddq = [ddx;ddy;ddth1;ddth2;ddth3];  % second time derivatives
% u   = [tau1 ; tau2 ; tau3];     % controls : torques on hip, knee, arm
% F   = [Fx ; Fy]; % constraint force
% state z = [q;dq]
    %% Definte fixed paramters
    m1 =.0393 + .2;         m2 =.0368; 
    m3 = .00783;            m4 = .0155;
    I1 = 25.1 * 10^-6;      I2 = 53.5 * 10^-6;
    I3 = 9.25 * 10^-6;      I4 = 22.176 * 10^-6;
    l_OA=.011;              l_OB=.042; 
    l_AC=.096;              l_DE=.091;
    l_O_m1=0.032;           l_B_m2=0.0344; 
    l_A_m3=0.0622;          l_C_m4=0.0610;
    N = 18.75;
    Ir = 0.0035/N^2;
    g = 9.81;    
    
    % parameters to be adjusted according to design
    m_body = 0.186+0.211;%
    l_body = 0.04;
    l_arm = 0.1;
    l_cm_arm = 0.8*l_arm;
    m_arm = 0.1; % 100 grams ?
    I_arm = m_arm*l_cm_arm^2;
%     restitution_coeff = 0.;
%     friction_coeff = 0.3;
    ground_height = 0;
    %% Parameter vector
    p   = [m1 m2 m3 m4 m_body m_arm I1 I2 I3 I4 I_arm Ir N l_O_m1 l_B_m2...
    l_A_m3 l_C_m4 l_cm_arm l_OA l_OB l_AC l_DE l_body l_arm g]';        % parameters
       

    
    %% Perform Dynamic simulation
    tf = 0.8;
%     num_step = floor(tf/dt);
    tspan = [0 tf];
%     tspan = linspace(0, tf, num_step); 
    
%     z0 = [0;0.2; 10*pi/180; 10*pi/180; 0; 0;0;0;0;0]; %[-pi/4; pi/2; 0; 0];
    desired_hip_pos0 = [0.03;0.08];
    guess_leg_angle  = [10*pi/180; 10*pi/180];
    init_leg_angle = fsolve(@(x)solve_init_pose(x,desired_hip_pos0,p),guess_leg_angle);
    init_arm_angle=0;
    
    z0 = [desired_hip_pos0;init_leg_angle;init_arm_angle;0;0;0;0;0];
    

    
    z_out = zeros(10,1);
    z_out(:,1) = z0;
    tout(1) = tspan(1);
    % simulate stance
    opts1 = odeset('events',@(t,z) detect_LO(t,z,p),'AbsTol',1e-6,'RelTol',1e-6);
    [tsim,zsim,tLO,zLO] = ode45(@(t,z) get_dynamics_stance(t,z,p),[tspan(1) tspan(2)],z0',opts1);

    tout = [tout tsim(2:end)'];
    z_out = [z_out  zsim(2:end,:)'];
    
    opts2 = odeset('events',@(t,z) detect_TD(t,z,p),'AbsTol',1e-6,'RelTol',1e-6);
    [tsim,zsim,tTD,zTD] = ode45(@(t,z) get_dynamics_flight(t,z,p),[tsim(end):0.002:tspan(2)],zsim(end,:)',opts2);

    tout = [tout tsim(2:end)'];
    z_out = [z_out  zsim(2:end,:)'];

    %% Compute Energy
    E = energy_floating(z_out,p);
    figure(1); clf
    plot(tout,E);xlabel('Time (s)'); ylabel('Energy (J)');
    %% Compute ground force
    Fgnd = zeros(2,length(tout));
    for i = 1:length(z_out)
    A = A_stance(z_out(:,i),p);
    
    if tout(i) <= tLO
        tau = control_law_flight(tout(i),z_out(:,i),p);
        b = b_stance(z_out(:,i),tau,p);
        x_augmented = A\(b);
        Fgnd(:,i) = x_augmented(6:7); % vertical constraint force
    else
        Fgnd(:,i)=[0;0];
    end
    

    end
    figure;plot(tout,Fgnd)
    ylabel('Ground Force')
    xlabel('Time (s)');
    legend('x','y')
    %% Compute foot position over time
    rE = zeros(2,length(tout));
    vE = zeros(2,length(tout));
    for i = 1:length(tout)
        rE(:,i) = position_foot(z_out(:,i),p);
        vE(:,i) = velocity_foot(z_out(:,i),p);
    end
    
    figure(2); clf;
    plot(tout,rE(1,:),'r','LineWidth',2)
    hold on
    plot(tout,rE(2,:),'b','LineWidth',2)
    title('Foot position')
    
    xlabel('Time (s)'); ylabel('Position (m)'); legend({'x','y'});

    figure(3); clf;
    plot(tout,vE(1,:),'r','LineWidth',2)
    hold on
    plot(tout,vE(2,:),'b','LineWidth',2)
    title('Foot Velocity')

    xlabel('Time (s)'); ylabel('Velocity (m)'); legend({'vel_x','vel_y'});
    
    figure(4)
    ax1(1) = subplot(211);
    plot(tout,z_out(1:2,:))
    legend('x','y');
    xlabel('Time (s)');
    ylabel('pos x,y (m)');
    
    ax1(2) = subplot(212);
    plot(tout,z_out(3:5,:)*180/pi)
    legend('q1','q2','q3');
    xlabel('Time (s)');
    ylabel('Angle (deg)');
    
    figure(5)
    ax1(3) = subplot(211);
    plot(tout,z_out(6:7,:))
    legend('xdot','ydot');
    xlabel('Time (s)');
    ylabel('base velocity (m/sec)');
    
    ax1(4) = subplot(212);
    plot(tout,z_out(8:10,:)*180/pi)
    legend('q1_dot','q2_dot','q3_dot');
    xlabel('Time (s)');
    ylabel('Ang.Vel (deg/s)');
    
    

    %% Animate Solution
    figure(6); clf;
    hold on
   
  
    % Target traj
    plot([-.2 .7],[ground_height ground_height],'k'); 
    
    animateSol(tout, z_out,p);
end

function tau = control_law_stance(t, z, p)
 
    tau = [-0.5;-5;0];

% 
end

function tau = control_law_flight(t, z, p)
 
    tau = [0;0;0];

% 
end


function [dz,F] = get_dynamics_stance(t,z,p)
    % here we solve for xdd,ydd and constraint for F
    % Get mass matrix
    A = A_stance(z,p);
    
    % Compute Controls
    tau = control_law_stance(t,z,p);
    
    % Get b = Q - V(q,qd) - G(q)
    b = b_stance(z,tau,p);
    
    % Solve for qdd.
    x_augmented = A\(b);
    qdd = x_augmented(1:5);
%     F = x_augmented(6:7); % constraint force
    
    dz = 0*z; % initialize dz
    
    % Form dz
    dz(1:5) = z(6:10);
    dz(6:10) = qdd;
    
end


function [dz,F] = get_dynamics_flight(t,z,p)
    
    % Get mass matrix
    A = A_floating(z,p);
    
    % Compute Controls
    
    
    tau = control_law_flight(t,z,p);
    
    % Get b = Q - V(q,qd) - G(q)
    F = [0;0]; % constraint force is zero in flight
    b = b_floating(z,tau,p,F);
    
    % Solve for qdd.
    qdd = A\(b);
    
    dz = 0*z; % initialize dz
    
    % Form dz
    dz(1:5) = z(6:10);
    dz(6:10) = qdd;
    
end

function [value,isterminal,direction] = detect_LO(t,z,p)
    % recompute here: how to pass these variables from dynamics without
    % recomputing everything?
    A = A_stance(z,p);
    tau = control_law_flight(t,z,p);
    b = b_stance(z,tau,p);

        
    x_augmented = A\(b);
    F_normal = x_augmented(7); % vertical constraint force
        
    value = F_normal;% detect when F cross zero
    direction  = -1; % when F is going down
    isterminal = 1; % = 1 : stop integration to change dynamics
end

function [value,isterminal,direction] = detect_TD(t,z,p)
    % recompute here: how to pass these variables from dynamics without
    % recomputing everything?
%     A = A_stance(z,p);
%     tau = control_law_flight(t,z,p);
%     b = b_stance(z,tau,p);

      pos_foot = position_foot(z,p);
%     x_augmented = A\(b);
%     F_normal = x_augmented(7); % vertical constraint force
    
    value = pos_foot(2);% detect when F cross zero
    direction  = -1; % when position is going down
    isterminal = 0; % = 1 : stop integration to change dynamics
end

function animateSol(tspan, x,p)
    % Prepare plot handles
    hold on
    h_OB = plot([0],[0],'LineWidth',2);
    h_AC = plot([0],[0],'LineWidth',2);
    h_BD = plot([0],[0],'LineWidth',2);
    h_CE = plot([0],[0],'LineWidth',2);
    h_O_arm_base = plot([0],[0],'LineWidth',2);
    h_arm_base_tip = plot([0],[0],'LineWidth',2);
    
    xlabel('x'); ylabel('y');
    h_title = title('t=0.0s');
    
    axis equal
    axis([-.2 0.7 -.01 .5]);

    %Step through and update animation
    for i = 1:length(tspan)
        % skip frame.
        if mod(i,1)
            continue;
        end
        t = tspan(i);
        z = x(:,i); 
        keypoints = keypoints_floating(z,p); %keypoints = [rA(1:2) rB(1:2) rC(1:2) rD(1:2) rE(1:2) r_origin(1:2) r_arm_base(1:2) r_arm_tip(1:2)];


        rA = keypoints(:,1); % Vector to base of cart
        rB = keypoints(:,2);
        rC = keypoints(:,3); % Vector to tip of pendulum
        rD = keypoints(:,4);
        rE = keypoints(:,5);
        r_origin = keypoints(:,6);
        r_arm_base =  keypoints(:,7);
        r_arm_tip  =  keypoints(:,8);

        set(h_title,'String',  sprintf('t=%.2f',t) ); % update title
        
        
        set(h_O_arm_base,'XData',[r_origin(1) r_arm_base(1)]);
        set(h_O_arm_base,'YData',[r_origin(2) r_arm_base(2)]);
        
        set(h_arm_base_tip,'XData',[r_arm_base(1) r_arm_tip(1)]);
        set(h_arm_base_tip,'YData',[r_arm_base(2) r_arm_tip(2)]);
        
        set(h_OB,'XData',[r_origin(1) rB(1)]);
        set(h_OB,'YData',[r_origin(2) rB(2)]);
        
        set(h_AC,'XData',[rA(1) rC(1)]);
        set(h_AC,'YData',[rA(2) rC(2)]);
        
        set(h_BD,'XData',[rB(1) rD(1)]);
        set(h_BD,'YData',[rB(2) rD(2)]);
        
        set(h_CE,'XData',[rC(1) rE(1)]);
        set(h_CE,'YData',[rC(2) rE(2)]);

        pause(.01)
    end
end