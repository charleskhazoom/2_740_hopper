close all
clear

% Objective functions
% 1 - final x com position based on ballistic traj

obj_func = 1;
use_boom = 1;

%% Add Libraries
% add casadi library
addpath(genpath('casadi'));
addpath(genpath('dynamics_gen'));

%% Set Auxillary Data
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
m_body = 0.186 +0.211;
l_body = 0.04;
l_arm = 0.1;
l_cm_arm = 0.8*l_arm;
l_cm_body = l_body/2;% assume body com is at half of the body length (makes sense since main body is composed of two motors (hip+arm) + brackets. com will be ~between both motors

m_arm = 0.2; % 100 grams ?
I_arm = m_arm*l_cm_arm^2;

ground_height = 0;
mu = 1.2; % friction coef

max_voltage = 15; % volts
motor_kt = 0.18;
motor_R = 2;
tau_max = (max_voltage)*motor_kt/motor_R*N;

m_offset_x = use_boom * 0.4;
m_offset_y = use_boom * 0.16;
l_boom = 8*0.0254;
h_boom = 0.1921; % to be adjusted for ground.
hob = 91.3/1000;
k = use_boom * 0.2877; % Nm/rad
boom_angle_min = -deg2rad(70);
boom_angle_max = deg2rad(70);

%% Parameter vector
p   = [m1 m2 m3 m4 m_body m_arm I1 I2 I3 I4 I_arm Ir N l_O_m1 l_B_m2...
    l_A_m3 l_C_m4 l_cm_arm l_cm_body l_OA l_OB l_AC l_DE l_body l_arm g...
    m_offset_x m_offset_y l_boom h_boom hob k motor_kt motor_R]';        % parameters

%% Initial conditions
desired_hip_pos0 = [0.0;0.1];%[0.03;0.06];
guess_leg_angle  = [10*pi/180; 10*pi/180];
init_leg_angle = fsolve(@(x)solve_init_pose(x,desired_hip_pos0,p),guess_leg_angle);
init_arm_angle = pi;
z0 = [desired_hip_pos0;init_leg_angle;init_arm_angle;0;0;0;0;0];

% Test for valid initial condition
if(init_leg_angle(1) > deg2rad(75) || init_leg_angle(1) < -deg2rad(75) ||...
        init_leg_angle(2) > deg2rad(142) || init_leg_angle(2) < -deg2rad(32))
    error('Infeasible starting position')
end

%% State/Control Bounds
q_min = [-0.2 -0.5 -deg2rad(75) deg2rad(32) -2*pi -1.5 -1.5 -15 -15 -15]';
q_max = [0.35 0.5  deg2rad(75)  deg2rad(142)  2*pi  3  3  15  15  15]';

u_min = -[tau_max tau_max tau_max]';
u_max = [tau_max tau_max tau_max]';

t_stance_vec = linspace(18,20,1);
landing_pos = zeros(1,length(t_stance_vec));
stance_time = zeros(1,length(t_stance_vec));
for j = 1:length(t_stance_vec)
    %% Integration Settings
    res = 5;
    dt = 0.01 / res;
    N_stance = floor(res * t_stance_vec(j)); % for now, let's fix stance time
    N_steps = N_stance + 15;
    
    %% Optimization Variables
    % create optimization object
    opti = casadi.Opti();
    % create optimization variables
    X = opti.variable(22,N_steps);
    
    % names for optimization variables
    qdd = X(1:5,:);    % joint acceleration (includes floating base coordinates)
    qd  = X(6:10,:);   % joint velocity
    q   = X(11:15,:);  % joint position
    f = X(16:17,:);  % foot force
    tau_motor = X(18:20,:); % actuator torques
    boom_angle = X(21,:);
    boom_force = X(22,:);
    
    %% Initial Constraints
    opti.subject_to(q(:,1) == z0(1:5))
    opti.subject_to(qd(:,1) == z0(6:10))
    
    %% Path Costs/Constraints
    
    for k = 1:N_steps
        
        disp([num2str(k),' of ',num2str(N_steps)]);
        
        % This iteration
        qk = q(:,k);
        qdk = qd(:,k);
        qddk = qdd(:,k);
        tauk = tau_motor(:,k);
        fk = f(:,k);
        boom_angle_k = boom_angle(:,k);
        boom_force_k = boom_force(:,k);
        
        % Dynamics
        if k <= N_stance
            Ak = A_stance([qk;qdk],p);
            bk = b_stance([qk;qdk],tauk,p);
            xk_augmented = Ak\(bk);
            opti.subject_to(qddk == xk_augmented(1:5));
            opti.subject_to(fk == xk_augmented(6:7));
        else
            Ak = A_floating([qk;qdk],p);
            bk = b_floating([qk;qdk],tauk,p,[0;0]);
            xk = Ak\(bk);
            opti.subject_to(qddk == xk);
            opti.subject_to(fk == [0;0]);
        end
        
        if k < N_steps
            opti.subject_to(q(:,k+1) == qk + dt*qdk) % position integration
            opti.subject_to(qd(:,k+1) == qdk + dt*qddk) % velocity integration
        end
        
        % Reaction Force
        opti.subject_to(fk(1) <= mu*fk(2)); % friction
        opti.subject_to(fk(1) >= -mu*fk(2));
        opti.subject_to(fk(2) >= 0); % unilateral
        opti.subject_to(fk(2) <= 80);
        
        % Voltage Inequality
        opti.subject_to( (tauk(1)/N)*motor_R/motor_kt + motor_kt*qdk(3)*N <= max_voltage);
        opti.subject_to( (tauk(1)/N)*motor_R/motor_kt + motor_kt*qdk(3)*N >= -max_voltage);
        opti.subject_to( (tauk(2)/N)*motor_R/motor_kt + motor_kt*qdk(4)*N <= max_voltage);
        opti.subject_to( (tauk(2)/N)*motor_R/motor_kt + motor_kt*qdk(4)*N >= -max_voltage);
        opti.subject_to( (tauk(3)/N)*motor_R/motor_kt + motor_kt*qdk(5)*N <= max_voltage);
        opti.subject_to( (tauk(3)/N)*motor_R/motor_kt + motor_kt*qdk(5)*N >= -max_voltage);
        
        % Boom angle constraint
        opti.subject_to(boom_angle_k == angle_boom([qk;qdk],p));
        opti.subject_to(boom_force_k == Force_boom([qk;qdk],p));
        opti.subject_to(boom_angle_k <= boom_angle_max);
        opti.subject_to(boom_angle_k >= boom_angle_min);
        
        % State Bounds
        opti.subject_to(qk <= q_max(1:5));
        opti.subject_to(qk >= q_min(1:5));
        opti.subject_to(qdk <= q_max(6:10));
        opti.subject_to(qdk >= q_min(6:10));
        
        % Control Bounds
        opti.subject_to(tauk <= u_max);
        opti.subject_to(tauk >= u_min);
        
        % Foot above ground
        foot_height_k = position_foot([qk;qdk],p);
        opti.subject_to(foot_height_k(2) >= 0);
        
        % Regularization
        if k == 1
            cost = (0.1*qdk(5)*qdk(5) + 0.1*tauk(3)*tauk(3))*dt;
        elseif k > N_stance
            cost = cost + (0.01*qdk(3:5)'*qdk(3:5) + 0.01*tauk(1:3)'*tauk(1:3))*dt;
        else
            cost = cost + (0.01*qdk(5)*qdk(5) + 0.01*tauk(3)*tauk(3))*dt;
        end
        
    end
    
    %% Terminal Costs/Constraints
    qN = q(:,N_stance);
    qdN = qd(:,N_stance);
    
    com_fin = com_pos([qN;qdN],p);
    dcom_fin = com_vel([qN;qdN],p);
    
    % Takeoff conditions
    opti.subject_to(com_fin(1) >= 0.0)
    opti.subject_to(dcom_fin(1) >= 0.0)
    opti.subject_to(dcom_fin(2) >= 0.0)
    
    switch obj_func
        case 1
            g_with_boom = 0.5*g;
            vz = dcom_fin(2);
            z = com_fin(2);
            vx = dcom_fin(1);
            x = com_fin(1);
            t_flight = (1/g_with_boom) * (vz + sqrt(vz^2 + 2*z*g_with_boom));
            x_land = x + vx*t_flight;
            cost = cost - x_land; % horizontal landing position
    end
    
    opti.minimize(cost);
    
    %% Initial guess
    opti.set_initial(q,repmat(z0(1:5),1,N_steps));
    opti.set_initial(qd,repmat(z0(6:10),1,N_steps));
    
    %% Solve!
    opti.solver('ipopt');
    sol = opti.solve();
    
    %% Decompose solution
    Xs{j} = sol.value(X); % full TO output
    qs{j} = sol.value(q); % position vector
    qds{j} = sol.value(qd); % velocity vector
    qdds{j} = sol.value(qdd); % acceleration vector
    fs{j} = sol.value(f); % front foot GRF
    taus{j} = sol.value(tau_motor); % Joint torques
    ts{j} = 0:dt:dt*(N_steps-1);
    t_stances{j} = 0:dt:dt*(N_stance-1);
    boom_angs{j} = sol.value(boom_angle); % boom angle
    boom_fs{j} = sol.value(boom_force); % boom force
    
    landing_pos(j) = -sol.value(cost);
    takeoff_pos_x(j) = sol.value(com_fin(1));
    takeoff_pos_y(j) = sol.value(com_fin(2));
    takeoff_vel_x(j) = sol.value(dcom_fin(1));
    takeoff_vel_y(j) = sol.value(dcom_fin(2));
    takeoff_vel_ratio(j) = sol.value(dcom_fin(1)/dcom_fin(2));
    stance_time(j) = t_stances{j}(end);
    
end

%% Compare differnt stance times
[~,indx] = max(landing_pos);

figure()
subplot(3,2,1)
plot(stance_time,landing_pos,'ro')
xlabel('Stance Time (s)')
ylabel('Horizontal Jump Distance');
subplot(3,2,2)
plot(stance_time,takeoff_vel_ratio,'ro')
xlabel('Stance Time (s)')
ylabel('Takeoff Velocity Ratio');
subplot(3,2,3)
plot(stance_time,takeoff_pos_x,'ro')
xlabel('Stance Time (s)')
ylabel('X Position at Takeoff (m)');
subplot(3,2,4)
plot(stance_time,takeoff_pos_y,'ro')
xlabel('Stance Time (s)')
ylabel('Y Position at Takeoff (m)');
subplot(3,2,5)
plot(stance_time,takeoff_vel_x,'ro')
xlabel('Stance Time (s)')
ylabel('X Velocity at Takeoff (m/s)');
subplot(3,2,6)
plot(stance_time,takeoff_vel_y,'ro')
xlabel('Stance Time (s)')
ylabel('Y Velocity at Takeoff (m)');

%% Simulate & plot
[tsim, zsim, tstance, zstance] = simulate_optimal_solution_casadi(t_stances{indx},z0,taus{indx},p);

plot_with_bounds(ts{indx},qs{indx},qds{indx},...
    fs{indx},taus{indx},boom_angs{indx},boom_fs{indx},...
    q_min,q_max,u_min,u_max,max_voltage,mu,...
    N,motor_R,motor_kt,boom_angle_min,boom_angle_max);

figure()
plot(zsim(:,1)',zsim(:,2)','ro-')
hold on
plot(qs{indx}(1,:),qs{indx}(2,:),'bo-')
xlabel('Body Position - x (m)')
ylabel('Body Position - y (m)')

%% Animate
figure()
animateSol(tsim,zsim',p);

%% Save traj
save_traj(ts{indx},[qs{indx};qds{indx}],taus{indx},'matt_test_traj.mat',1/100)



