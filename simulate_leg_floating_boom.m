function simulate_leg_floating()
path_sym_dynamics = 'dynamics_gen_boom/';
addpath(path_sym_dynamics)
close all
%% description of the variables
% q   = [x;y;th1  ; th2 ; th3];      % generalized coordinates
% dq  = [dx;dy;dth1 ; dth2; dth3];    % first time derivatives
% ddq = [ddx;ddy;ddth1;ddth2;ddth3];  % second time derivatives
% u   = [tau1 ; tau2 ; tau3];     % controls : torques on hip, knee, arm
% F   = [Fx ; Fy]; % constraint force
% state z = [q;dq]
    %% Definte fixed paramters
    m1 =.0393 + .2; % 0.2 is motor mass
    m2 =.0368; 
    m3 = .00783;
    m4 = .0155;
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
    l_body = 68.1/1000;
    l_arm = 5*0.0254;
    l_cm_arm = 0.8*l_arm;
    l_cm_body=l_body/2;% assume body com is at half of the body length (makes sense since main body is composed of two motors (hip+arm) + brackets. com will be ~between both motors

    m_arm = 0.2; % 100 grams ?
    I_arm = m_arm*l_cm_arm^2;
%     restitution_coeff = 0.;
%     friction_coeff = 0.3;
    ground_height = 0;
 %% motor parameters
 motor_kt = 0.18;
 motor_R = 2;
 %% boom parameters
 m_offset_x = 0.4;
 m_offset_y = 0.16;
 l_boom = 8*0.0254;
 h_boom = 0.2281;%0.3; % to be adjusted for ground.
 hob = 91.3/1000;
 k = 0.2877/1; % Nm/rad
    %% Parameter vector
 
 p   = [m1 m2 m3 m4 m_body m_arm I1 I2 I3 I4 I_arm Ir N l_O_m1 l_B_m2...
    l_A_m3 l_C_m4 l_cm_arm l_cm_body l_OA l_OB l_AC l_DE l_body l_arm g...
    m_offset_x m_offset_y l_boom h_boom hob k motor_kt motor_R]';        % parameters
 
% p   = [m1 m2 m3 m4 m_body m_arm I1 I2 I3 I4 I_arm Ir N l_O_m1 l_B_m2...
%     l_A_m3 l_C_m4 l_cm_arm l_cm_body l_OA l_OB l_AC l_DE l_body l_arm g motor_kt motor_R]';        % parameters

    
    %% Perform Dynamic simulation
    tf = 0.8;
%     num_step = floor(tf/dt);
    tspan = [0 tf];
%     tspan = linspace(0, tf, num_step); 
    
%     z0 = [0;0.2; 10*pi/180; 10*pi/180; 0; 0;0;0;0;0]; %[-pi/4; pi/2; 0; 0];
    desired_hip_pos0 = [0.014;0.12];
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
    
    z0=zsim(end,:)';
%     opts2 = odeset('events',@(t,z) detect_TD(t,z,p),'AbsTol',1e-6,'RelTol',1e-6);
        opts2 = odeset('AbsTol',1e-6,'RelTol',1e-6);

%     [tsim,zsim,tTD,zTD] = ode45(@(t,z) get_dynamics_flight(t,z,p,z0),[tsim(end):0.002:tspan(2)],z0,opts2);
    [tsim,zsim] = ode45(@(t,z) get_dynamics_flight(t,z,p,z0),[tsim(end) tspan(2)],z0,opts2);

    tout = [tout tsim(2:end)'];
    z_out = [z_out  zsim(2:end,:)'];

%%
for i =1:length(z_out)
    if tout(i) <= tLO % if before liftoff;
        u_out(:,i) = control_law_stance(tout(i),z_out(:,i),p);
        
    else 
        u_out(:,i) = control_law_flight(tout(i),z_out(:,i),p,z0);
    end
        F_boom(:,i) = Force_boom(z_out(:,i),p);
        th_boom(:,i) = angle_boom(z_out(:,i),p);
end

figure;
plot(F_boom);


idx_sh = find(imag(th_boom)~=0,1)

figure;
plot(tout,th_boom)
fignb=1;


    %% Animate Solution
    figure(fignb); 
    fignb=fignb+1; 
    clf;
    hold on
   
  
    % Target traj
    plot([-.2 .7],[ground_height ground_height],'k'); 
    
    animateSol(tout, z_out,p);
    
    %% make plots

    fignb = plot_solution(tout,z_out,u_out,tLO,p,fignb);
    
    %% save trajectoryname
    name = 'some_traj_for_se_hwan'
    dt_traj = 0.001;
    save_traj(tout,z_out,u_out,name,dt_traj)
end

function tau = control_law_stance(t, z, p)
 
    tau = [2;-1;0];
    tau = saturate_torque(z,tau,p);
%     tau = [0;0;0];
% 
end

function tau = control_law_flight(t, z, p,z0)
 
    %tau = [0.5;0;0.5];
%     tau = [-0.5;-0.5;1];
    
    kp = 1;
    kd = 0.5;
    q_ref = z0(3:5);
    q = z(3:5);
    
    qd = z(8:10);
    
    qd_ref = [0;0;0];

    tau = kp*(q_ref-q) + kd*(qd_ref - qd);
    tau = saturate_torque(z,tau,p);

    
%     tau = [0;0;0];

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


function [dz,F] = get_dynamics_flight(t,z,p,z0)
    
    % Get mass matrix
    A = A_floating(z,p);
    
    % Compute Controls
    
    
    tau = control_law_flight(t,z,p,z0);
    
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
    tau = control_law_stance(t,z,p);
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

function u = saturate_torque(z,u,p)
    emax = 12;
    kt = p(33); 
    R  = p(34);
    N  = p(13);
    
    qd = z(8:10);
    
    
    tau_max = (emax - kt*qd*N)*kt/R*N;
    tau_min = (-emax - kt*qd*N)*kt/R*N;
    
    u = max(min(u,tau_max),tau_min);
    
end
