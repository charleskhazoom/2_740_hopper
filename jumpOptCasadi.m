close all
clear

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
mu = 0.8; % friction coef

max_voltage = 12; % volts
motor_kt = 0.18;
motor_R = 2;
tau_max = (max_voltage)*motor_kt/motor_R*N;

m_offset_x = 0.4;
m_offset_y = 0.16;
l_boom = 8*0.0254;
h_boom = 0.3; % to be adjusted for ground.
hob = 91.3/1000;
k = 0.2877/1.35; % Nm/rad

%% Parameter vector
p   = [m1 m2 m3 m4 m_body m_arm I1 I2 I3 I4 I_arm Ir N l_O_m1 l_B_m2...
    l_A_m3 l_C_m4 l_cm_arm l_cm_body l_OA l_OB l_AC l_DE l_body l_arm g...
    m_offset_x m_offset_y l_boom h_boom hob k motor_kt motor_R]';        % parameters

%% Initial conditions
desired_hip_pos0 = [0.014;0.15];%[0.03;0.06];
guess_leg_angle  = [10*pi/180; 10*pi/180];
init_leg_angle = fsolve(@(x)solve_init_pose(x,desired_hip_pos0,p),guess_leg_angle);
init_arm_angle = 0;
z0 = [desired_hip_pos0;init_leg_angle;init_arm_angle;0;0;0;0;0];

%% State/Control Bounds
q_min = [-0.2 -0.5 -deg2rad(75) deg2rad(32) -1.5*pi -2 -2 -16 -16 -16]';
q_max = [0.35 0.5  deg2rad(75)  deg2rad(142)  1.5*pi  8  8  16  16  16]';

u_min = -[tau_max tau_max tau_max]';
u_max = [tau_max tau_max tau_max]';

%% Integration Settings
res = 5;
dt = 0.01 / res;
N_steps = res * 28; % for now, let's fix stance time

%% Optimization Variables
% create optimization object
opti = casadi.Opti();
% create optimization variables
X = opti.variable(20,N_steps);

% names for optimization variables
qdd = X(1:5,:);    % joint acceleration (includes floating base coordinates)
qd  = X(6:10,:);   % joint velocity
q   = X(11:15,:);  % joint position
f = X(16:17,:);  % foot force
tau_motor = X(18:20,:); % actuator torques

%% Initial Constraints
opti.subject_to(q(:,1) == z0(1:5))
opti.subject_to(qd(:,1) == z0(6:10))

%% Path Costs/Constraints

for k = 1:N_steps-1
    
    disp([num2str(k),' of ',num2str(N_steps-1)]);
    
    % This iteration
    qk = q(:,k);
    qdk = qd(:,k);
    qddk = qdd(:,k);
    tauk = tau_motor(:,k);
    fk = f(:,k);
    
    % Dynamics
    Ak = A_stance([qk;qdk],p);
    bk = b_stance([qk;qdk],tauk,p);
    
    xk_augmented = Ak\(bk);
    opti.subject_to(qddk == xk_augmented(1:5));
    opti.subject_to(fk == xk_augmented(6:7));
    
    opti.subject_to(q(:,k+1) == qk + dt*qdk) % position integration
    opti.subject_to(qd(:,k+1) == qdk + dt*qddk) % velocity integration
    
    opti.subject_to(fk(1) <= mu*fk(2)); % friction
    opti.subject_to(fk(1) >= -mu*fk(2));
    
    opti.subject_to(fk(2) >= 0); % unilateral
    
    % Voltage Inequality
    opti.subject_to( (tauk(1)/N)*motor_R/motor_kt + motor_kt*qdk(3)*N <= max_voltage);
    opti.subject_to( (tauk(1)/N)*motor_R/motor_kt + motor_kt*qdk(3)*N >= -max_voltage);
    
    opti.subject_to( (tauk(2)/N)*motor_R/motor_kt + motor_kt*qdk(4)*N <= max_voltage);
    opti.subject_to( (tauk(2)/N)*motor_R/motor_kt + motor_kt*qdk(4)*N >= -max_voltage);
    
    opti.subject_to( (tauk(3)/N)*motor_R/motor_kt + motor_kt*qdk(5)*N <= max_voltage);
    opti.subject_to( (tauk(3)/N)*motor_R/motor_kt + motor_kt*qdk(5)*N >= -max_voltage);
    
    % State Bounds
    opti.subject_to(qk <= q_max(1:5));
    opti.subject_to(qk >= q_min(1:5));
    opti.subject_to(qdk <= q_max(6:10));
    opti.subject_to(qdk >= q_min(6:10));
    
    % Control Bounds
    opti.subject_to(tauk <= u_max);
    opti.subject_to(tauk >= u_min);
    
end

%% Terminal Costs/Constraints
qN = q(:,N_steps);
qdN = qd(:,N_steps);

com_fin = com_pos([qN;qdN],p);
dcom_fin = com_vel([qN;qdN],p);

vz = dcom_fin(2);
z = com_fin(2);
vx = dcom_fin(1);
x = com_fin(1);
t_flight = (1/g) * (vz + sqrt(vz^2 + 2*z*g));
x_land = x + vx*t_flight;

%cost = -com_fin(1) - com_fin(2); % position at end of stance
cost = -x_land; % horizontal landing position
opti.minimize(cost);

%% Initial guess
opti.set_initial(q,repmat(z0(1:5),1,N_steps));
opti.set_initial(qd,repmat(z0(6:10),1,N_steps));

%% Solve!
opti.solver('ipopt');
sol = opti.solve();

%% Decompose solution
Xs = sol.value(X); % full TO output
qs = sol.value(q); % position vector
qds = sol.value(qd); % position vector
fs = sol.value(f); % front foot GRF
taus = sol.value(tau_motor); % Joint torques
ts = 0:dt:dt*(N_steps-1);

%% Simulate, Plot, Compare
[tsim, zsim, tstance, zstance] = simulate_optimal_solution_casadi(ts,z0,taus,p);

% Turn these plots into a function
% Compare body position/velocity
figure()
subplot(2,1,1)
plot(tsim,zsim(:,1)','r')
hold on
plot(tsim,zsim(:,2)','b')
plot(ts,qs(1,:),'r--')
plot(ts,qs(2,:),'b--')
xlabel('Time (s)')
ylabel('Body Position (m)')
legend('x','y')

subplot(2,1,2)
plot(tsim,zsim(:,6)','r')
hold on
plot(tsim,zsim(:,7)','b')
plot(ts,qds(1,:),'r--')
plot(ts,qds(2,:),'b--')
xlabel('Time (s)')
ylabel('Body Velocity (m/s)')
legend('x','y')

% Compare joint position/velocity
figure()
subplot(2,1,1)
plot(tsim,zsim(:,3)','m')
hold on
plot(tsim,zsim(:,4)','g')
plot(tsim,zsim(:,5)','k')
plot(ts,qs(3,:),'m--')
plot(ts,qs(4,:),'g--')
plot(ts,qs(5,:),'k--')
xlabel('Time (s)')
ylabel('Joint Position (rad)')
legend('\theta_1','\theta_2','\theta_3')

subplot(2,1,2)
plot(tsim,zsim(:,8)','m')
hold on
plot(tsim,zsim(:,9)','g')
plot(tsim,zsim(:,10)','k')
plot(ts,qds(3,:),'m--')
plot(ts,qds(4,:),'g--')
plot(ts,qds(5,:),'k--')
xlabel('Time (s)')
ylabel('Joint Velocity (rad/s)')
legend('\theta_1','\theta_2','\theta_3')

figure()
animateSol(tsim,zsim',p);





