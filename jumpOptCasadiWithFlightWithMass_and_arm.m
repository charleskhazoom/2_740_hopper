close all
clear

% Objective functions
% 1 - final x com position based on ballistic traj

obj_func = 1;
use_boom = 0;

%% Add Libraries
% add casadi library
addpath(genpath('casadi'));
addpath(genpath('dynamics_gen_boom'));

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
l_body = 68.1/1000;
l_arm = 5*0.0254;% dummy assignation for some initializations
l_cm_arm = 1*l_arm;% dummy assignation for some initializations
l_cm_body = l_body/2;% assume body com is at half of the body length (makes sense since main body is composed of two motors (hip+arm) + brackets. com will be ~between both motors

m_arm = 0.2; % 100 grams ?
 
I_arm = m_arm*l_cm_arm^2; % dummy assignation for some initializations

ground_height = 0;
mu = 0.8; % friction coef

max_voltage = 20; % volts
motor_kt = 0.18;
motor_R = 2;
tau_max = (max_voltage)*motor_kt/motor_R*1.2;

m_offset_x = use_boom * 0.4;
m_offset_y = use_boom * 0.16;
l_boom = 8*0.0254;
h_boom = 0.1921; % to be adjusted for ground.
hob = 91.3/1000;
boom_stiffness = use_boom * 0.2877/1.2; % Nm/rad
boom_angle_min = -deg2rad(70);
boom_angle_max = deg2rad(70);

%% Parameter vector
p   = [m1 m2 m3 m4 m_body m_arm I1 I2 I3 I4 I_arm Ir N l_O_m1 l_B_m2...
    l_A_m3 l_C_m4 l_cm_arm l_cm_body l_OA l_OB l_AC l_DE l_body l_arm g...
    m_offset_x m_offset_y l_boom h_boom hob boom_stiffness motor_kt motor_R]';        % parameters

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
q_min = [-0.2 -0.5 -deg2rad(75) deg2rad(32) -2*pi -1.5 -1.5 -150 -150 -150]';
q_max = [0.35 0.5  deg2rad(75)  deg2rad(142)  2*pi  3  3     150  150  150]';

u_min = -[tau_max tau_max tau_max]';
u_max = [tau_max tau_max tau_max]';


mass_vec = linspace(0.05,0.4,10);


l_arm_length_vec = [4 8 12]*0.0254; %linspace(2,20,10)*0.0254;

landing_pos = zeros(1,length(mass_vec));
stance_time = zeros(1,length(mass_vec));

for aa = 1:length(l_arm_length_vec)
    aa


    for j = 1:length(mass_vec)
        l_arm = l_arm_length_vec(aa);
        m_arm = mass_vec(j);
        l_cm_arm = 1*l_arm;
        I_arm = m_arm*l_cm_arm^2;
        
        p   = [m1 m2 m3 m4 m_body m_arm I1 I2 I3 I4 I_arm Ir N l_O_m1 l_B_m2...
            l_A_m3 l_C_m4 l_cm_arm l_cm_body l_OA l_OB l_AC l_DE l_body l_arm g...
            m_offset_x m_offset_y l_boom h_boom hob boom_stiffness motor_kt motor_R]';        % parameters

        j
        %% Integration Settings
        res = 5;
        dt = 0.01/res;
        N_stance = floor(res * 17); % for now, let's fix stance time
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
            opti.subject_to((tauk(1))*motor_R/motor_kt + motor_kt*qdk(3) <= max_voltage);
            opti.subject_to((tauk(1))*motor_R/motor_kt + motor_kt*qdk(3) >= -max_voltage);
            opti.subject_to((tauk(2))*motor_R/motor_kt + motor_kt*qdk(4) <= max_voltage);
            opti.subject_to((tauk(2))*motor_R/motor_kt + motor_kt*qdk(4) >= -max_voltage);
            opti.subject_to((tauk(3))*motor_R/motor_kt + motor_kt*qdk(5) <= max_voltage);
            opti.subject_to((tauk(3))*motor_R/motor_kt + motor_kt*qdk(5) >= -max_voltage);
            
            % Boom angle constraint
            
            if use_boom == 1
                opti.subject_to(boom_angle_k == angle_boom([qk;qdk],p));
                opti.subject_to(boom_force_k == Force_boom([qk;qdk],p));
                opti.subject_to(boom_angle_k <= boom_angle_max);
                opti.subject_to(boom_angle_k >= boom_angle_min);
            end
            
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
                if use_boom ==1
                    g_with_boom = 0.5*g;
                else
                    g_with_boom = g;
                end
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
        Xs{j,aa} = sol.value(X); % full TO output
        qs{j,aa} = sol.value(q); % position vector
        qds{j,aa} = sol.value(qd); % velocity vector
        qdds{j,aa} = sol.value(qdd); % acceleration vector
        fs{j,aa} = sol.value(f); % front foot GRF
        taus{j,aa} = sol.value(tau_motor); % Joint torques
        ts{j,aa} = 0:dt:dt*(N_steps-1);
        t_stances{j,aa} = 0:dt:dt*(N_stance-1);
        boom_angs{j,aa} = sol.value(boom_angle); % boom angle
        boom_fs{j,aa} = sol.value(boom_force); % boom force
        
        landing_pos(j,aa) = -sol.value(cost);
        takeoff_pos_x(j,aa) = sol.value(com_fin(1));
        takeoff_pos_y(j,aa) = sol.value(com_fin(2));
        takeoff_vel_x(j,aa) = sol.value(dcom_fin(1));
        takeoff_vel_y(j,aa) = sol.value(dcom_fin(2));
        takeoff_vel_ratio(j,aa) = sol.value(dcom_fin(1)/dcom_fin(2));
        stance_time(j,aa) = t_stances{j,aa}(end);
        
    end
end
%% Compare different arm masses for a given arm length

indx_l_arm = 1; % do this for a given arm length

[~,indx_mass] = max(landing_pos(:,indx_l_arm));


figure()
subplot(3,2,1)
plot(mass_vec,landing_pos(:,indx_l_arm),'ro')
xlabel('Arm Mass (kg)')
ylabel('Horizontal Jump Distance');
subplot(3,2,2)
plot(mass_vec,takeoff_vel_ratio(:,indx_l_arm),'ro')
xlabel('Arm Mass (kg)')
ylabel('Takeoff Velocity Ratio');
subplot(3,2,3)
plot(mass_vec,takeoff_pos_x(:,indx_l_arm),'ro')
xlabel('Arm Mass (kg)')
ylabel('X Position at Takeoff (m)');
subplot(3,2,4)
plot(mass_vec,takeoff_pos_y(:,indx_l_arm),'ro')
xlabel('Arm Mass (kg)')
ylabel('Y Position at Takeoff (m)');
subplot(3,2,5)
plot(mass_vec,takeoff_vel_x(:,indx_l_arm),'ro')
xlabel('Stance Time (s)')
xlabel('Arm Mass (kg)')
subplot(3,2,6)
plot(mass_vec,takeoff_vel_y(:,indx_l_arm),'ro')
xlabel('Arm Mass (kg)')
ylabel('Y Velocity at Takeoff (m)');

%% Compare different arm length for a given stance time
indx_mass = 1; % do this for a given arm mass

last_arm_idx = length(l_arm_length_vec);
l_arm_length_vec =l_arm_length_vec(1:last_arm_idx);

figure()
title('')
subplot(3,2,1)
plot(l_arm_length_vec,landing_pos(indx_mass,:),'ro')
xlabel('Arm Length (m)')
ylabel('Horizontal Jump Distance');
subplot(3,2,2)
plot(l_arm_length_vec,takeoff_vel_ratio(indx_mass,:),'ro')
xlabel('Arm Length (m)')
ylabel('Takeoff Velocity Ratio');
subplot(3,2,3)
plot(l_arm_length_vec,takeoff_pos_x(indx_mass,:),'ro')
xlabel('Arm Length (m)')
ylabel('X Position at Takeoff (m)');
subplot(3,2,4)
plot(l_arm_length_vec,takeoff_pos_y(indx_mass,:),'ro')
xlabel('Arm Length (m)')
ylabel('Y Position at Takeoff (m)');
subplot(3,2,5)
plot(l_arm_length_vec,takeoff_vel_x(indx_mass,:),'ro')
xlabel('Arm Length (m)')
ylabel('X Velocity at Takeoff (m/s)');
subplot(3,2,6)
plot(l_arm_length_vec,takeoff_vel_y(indx_mass,:),'ro')
xlabel('Arm Length (m)')
ylabel('Y Velocity at Takeoff (m)');



%% Simulate & plot
indx_l_arm=3;
% take correct arm length for simulation
l_arm = l_arm_length_vec(indx_l_arm);
m_arm = mass_vec(indx_mass);

l_cm_arm = 1*l_arm;
I_arm = m_arm*l_cm_arm^2;

p   = [m1 m2 m3 m4 m_body m_arm I1 I2 I3 I4 I_arm Ir N l_O_m1 l_B_m2...
    l_A_m3 l_C_m4 l_cm_arm l_cm_body l_OA l_OB l_AC l_DE l_body l_arm g...
    m_offset_x m_offset_y l_boom h_boom hob boom_stiffness motor_kt motor_R]';        % parameters


[tsim, zsim, tstance, zstance] = simulate_optimal_solution_casadi(t_stances{indx_mass,indx_l_arm},z0,taus{indx_mass,indx_l_arm},p);

plot_with_bounds(ts{indx_mass,indx_l_arm},qs{indx_mass,indx_l_arm},qds{indx_mass,indx_l_arm},...
    fs{indx_mass,indx_l_arm},taus{indx_mass,indx_l_arm},boom_angs{indx_mass,indx_l_arm},boom_fs{indx_mass,indx_l_arm},...
    q_min,q_max,u_min,u_max,max_voltage,mu,...
    N,motor_R,motor_kt,boom_angle_min,boom_angle_max);

figure();
plot(zsim(:,1)',zsim(:,2)','ro-')
hold on
plot(qs{indx_mass}(1,:),qs{indx_mass}(2,:),'bo-')
xlabel('Body Position - x (m)')
ylabel('Body Position - y (m)')

%% Animate

figure();
set(gcf,'position',[68           1        1853         970]);

animateSol(tsim,zsim',p);

%% Save traj
save_traj(ts{indx_mass,indx_l_arm},[qs{indx_mass,indx_l_arm};qds{indx_mass,indx_l_arm}],taus{indx_mass,indx_l_arm},'test_traj_right_kt.mat',1/100)


%% simulate in loop for arm length

for aa = 1:length(l_arm_length_vec)
    
    % take correct arm length for simulation
    l_arm = l_arm_length_vec(aa);
    m_arm = mass_vec(indx_mass);

    l_cm_arm = 1*l_arm;
    I_arm = m_arm*l_cm_arm^2;
    
    p   = [m1 m2 m3 m4 m_body m_arm I1 I2 I3 I4 I_arm Ir N l_O_m1 l_B_m2...
        l_A_m3 l_C_m4 l_cm_arm l_cm_body l_OA l_OB l_AC l_DE l_body l_arm g...
        m_offset_x m_offset_y l_boom h_boom hob boom_stiffness motor_kt motor_R]';        % parameters

    
    [tsim, zsim, tstance, zstance] = simulate_optimal_solution_casadi(t_stances{indx_mass,aa},z0,taus{indx_mass,aa},p);
    rE{aa} = position_foot(zsim',p);
    
   
    
end

figure;

for aa = 1:length(l_arm_length_vec)
     plot(l_arm_length_vec(aa),rE{aa}(1,end),'or');hold on;
end
xlabel('Arm Length (m)');
ylabel('Jumping Distance (m)'); grid on
%%

%% simulate in loop for arm mass

for mm = 1:length(mass_vec)
    
    % take correct arm length for simulation
    l_arm = l_arm_length_vec(indx_arm);
    m_arm = mass_vec(mm);

    l_cm_arm = 1*l_arm;
    I_arm = m_arm*l_cm_arm^2;
    
    p   = [m1 m2 m3 m4 m_body m_arm I1 I2 I3 I4 I_arm Ir N l_O_m1 l_B_m2...
        l_A_m3 l_C_m4 l_cm_arm l_cm_body l_OA l_OB l_AC l_DE l_body l_arm g...
        m_offset_x m_offset_y l_boom h_boom hob boom_stiffness motor_kt motor_R]';        % parameters

    
    [tsim, zsim, tstance, zstance] = simulate_optimal_solution_casadi(t_stances{mm,indx_arm},z0,taus{mm,indx_arm},p);
    rE{mm} = position_foot(zsim',p);
    
   
end

figure;

for mm = 1:length(mass_vec)
     plot(mass_vec(mm),rE{mm}(1,end),'or');hold on;
end
xlabel('Arm Mass (kg)');
ylabel('Jumping Distance (m)'); grid on