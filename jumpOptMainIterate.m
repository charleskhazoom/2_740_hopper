clear;clc;close all;
addpath('dynamics_gen/')

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
l_cm_body=l_body/2;% assume body com is at half of the body length (makes sense since main body is composed of two motors (hip+arm) + brackets. com will be ~between both motors

m_arm = 0.1; % 100 grams ?
I_arm = m_arm*l_cm_arm^2;
ground_height = 0;

%% Parameter vector
mu = 0.9; % friction coef
max_voltage = 20; % volts
motor_kt = 0.18;
motor_R = 2;
p   = [m1 m2 m3 m4 m_body m_arm I1 I2 I3 I4 I_arm Ir N l_O_m1 l_B_m2...
    l_A_m3 l_C_m4 l_cm_arm l_cm_body l_OA l_OB l_AC l_DE l_body l_arm g motor_kt motor_R]';        % parameters
auxdata.p = p;

%% Set Bounds

desired_hip_pos0 = [0.014;0.15];%[0.03;0.06];
guess_leg_angle  = [10*pi/180; 10*pi/180];
init_leg_angle = fsolve(@(x)solve_init_pose(x,desired_hip_pos0,p),guess_leg_angle);
init_arm_angle = 0;
z0 = [desired_hip_pos0;init_leg_angle;init_arm_angle;0;0;0;0;0];

% Time
bounds.phase(1).initialtime.lower = 0;
bounds.phase(1).initialtime.upper = 0;

bounds.phase(1).finaltime.lower = 0.2;
bounds.phase(1).finaltime.upper = 0.3;

bounds.phase(2).duration.lower = 0.2;
bounds.phase(2).duration.upper = 0.35;

bounds.phase(2).initialtime.lower = bounds.phase(1).finaltime.lower;
bounds.phase(2).initialtime.upper = bounds.phase(1).finaltime.upper;

bounds.phase(2).finaltime.lower = bounds.phase(2).initialtime.lower+bounds.phase(2).duration.lower;
bounds.phase(2).finaltime.upper = bounds.phase(2).initialtime.upper+bounds.phase(2).duration.upper;

% State
bounds.phase(1).initialstate.lower = z0';
bounds.phase(1).initialstate.upper = z0';

bounds.phase(1).state.lower = [-0.08 -0.1 -pi -pi -pi -2 -2 -16 -16 -16];
bounds.phase(1).state.upper = [0.35   0.5  pi  pi pi  8  8  16  16  16];

bounds.phase(1).finalstate.lower = bounds.phase(1).state.lower;
bounds.phase(1).finalstate.upper = bounds.phase(1).state.upper;

bounds.phase(2).initialstate.lower = bounds.phase(1).state.lower;
bounds.phase(2).initialstate.upper = bounds.phase(1).state.upper;

bounds.phase(2).state.lower = bounds.phase(1).state.lower;
bounds.phase(2).state.upper = bounds.phase(1).state.upper;

bounds.phase(2).finalstate.lower = bounds.phase(1).state.lower;
bounds.phase(2).finalstate.upper = bounds.phase(1).state.upper;

% Control
bounds.phase(1).control.lower = [-25 -25 -25];
bounds.phase(1).control.upper = [25 25 25];

bounds.phase(2).control.lower = -[2 2 2];
bounds.phase(2).control.upper = [2 2 2];

% friction bounds for force
bounds.phase(1).path.lower(1) = -mu;
bounds.phase(1).path.upper(1) = mu;

% unilateral bounds for force
bounds.phase(1).path.lower(2) = 0;
bounds.phase(1).path.upper(2) = 50;

% bounds for voltage (3 motors)
for motor = 1:3
    bounds.phase(1).path.lower(end+1) = -max_voltage;
    bounds.phase(1).path.upper(end+1) = max_voltage;
    
    bounds.phase(2).path.lower(motor) = -max_voltage;
    bounds.phase(2).path.upper(motor) = max_voltage;
end

% Continuity
bounds.eventgroup(1).lower = zeros(1,11);
bounds.eventgroup(1).upper = zeros(1,11);

% Integral
% bounds.phase(1).integral.lower = [0 0 0 0 0 0];
%bounds.phase(1).integral.upper = [20 10 10 10 10 100];

bounds.phase(1).integral.lower = [0 0];
bounds.phase(1).integral.upper = [200 200];
bounds.phase(2).integral.lower = [0 0];
bounds.phase(2).integral.upper = [200 200];

%% Initial Guess
[t_guess, z_guess, u_guess] = get_initial_guess(bounds.phase(1).finaltime.upper+bounds.phase(2).duration.upper,z0,p);
guess.phase(1).time = t_guess{1};
guess.phase(1).state = z_guess{1};
guess.phase(1).control = u_guess{1}';
%guess.phase(1).integral = [2 2 2 2 2 30];
guess.phase(1).integral = [2 2];

guess.phase(2).time = t_guess{2};
guess.phase(2).state = z_guess{2};
guess.phase(2).control = u_guess{2}';
guess.phase(2).integral = [2 2];

% guess.phase(1).time = [0;bounds.phase(1).finaltime.upper];
% guess.phase(1).state = [z0';z0'];
% guess.phase(1).control = zeros(2,3);
% guess.phase(1).integral = [2 2 2 2 2];
% 
% guess.phase(2).time = [bounds.phase(1).finaltime.upper;bounds.phase(2).finaltime.upper];
% guess.phase(2).state = [z0';z0'];
% guess.phase(2).control = zeros(2,3);

%% Mesh Setup
mesh.method = 'hp-PattersonRao';
mesh.tolerance = 1e-2;
mesh.maxiterations = 3;
mesh.colpointsmin = 4;
mesh.colpointsmax = 12;

%% Problem Setup
setup.name = 'jumpOpt';
setup.functions.continuous = @jumpOptContinuous;
setup.functions.endpoint = @jumpOptEndpoint;
setup.auxdata = auxdata;

setup.derivatives.supplier = 'sparseCD';
setup.derivatives.derivativelevel = 'first';
setup.derivatives.dependencies = 'sparse';

setup.bounds = bounds;
setup.guess = guess;
setup.mesh = mesh;
setup.nlp.solver = 'ipopt';

cnt = 1;
iter_max = [100 200 300 400 500 600 1000 2000];
for i = iter_max
    setup.nlp.ipoptoptions.maxiterations = i;
    
    setup.method = 'RPM-Differentiation';
    setup.scales.method = 'automatic-bounds'; % 'automatic-guessUpdate';
    
    %% Solve
    output = gpops2(setup);
    solution{cnt} = output.result.solution;
    interp_sol{cnt} = output.result.interpsolution;
    
    %% Compare via Plots
    figure(1)
    plot([solution{cnt}.phase(1).time;solution{cnt}.phase(2).time],...
        [solution{cnt}.phase(1).state(:,1);solution{cnt}.phase(2).state(:,1)]')
    hold on
    title('Body Position - x (m)')
    
    figure(2)
    plot([solution{cnt}.phase(1).time;solution{cnt}.phase(2).time],...
        [solution{cnt}.phase(1).state(:,2);solution{cnt}.phase(2).state(:,2)]')
    hold on
    title('Body Position - y (m)')
    
    figure(3)
    plot([solution{cnt}.phase(1).time;solution{cnt}.phase(2).time],...
        [solution{cnt}.phase(1).state(:,3);solution{cnt}.phase(2).state(:,3)]')
    hold on
    title('Joint Position - \theta_1 (rad)')
    
    figure(4)
    plot([solution{cnt}.phase(1).time;solution{cnt}.phase(2).time],...
        [solution{cnt}.phase(1).state(:,4);solution{cnt}.phase(2).state(:,4)]')
    hold on
    title('Joint Position - \theta_2 (rad)')
    
    figure(5)
    plot([solution{cnt}.phase(1).time;solution{cnt}.phase(2).time],...
        [solution{cnt}.phase(1).state(:,5);solution{cnt}.phase(2).state(:,5)]')
    hold on
    title('Joint Position - \theta_3 (rad)')
    
    figure(6)
    plot([solution{cnt}.phase(1).time;solution{cnt}.phase(2).time],...
        [solution{cnt}.phase(1).state(:,6);solution{cnt}.phase(2).state(:,6)]')
    hold on
    title('Body Velocity - v_x (m/s)')
    
    figure(7)
    plot([solution{cnt}.phase(1).time;solution{cnt}.phase(2).time],...
        [solution{cnt}.phase(1).state(:,7);solution{cnt}.phase(2).state(:,7)]')
    hold on
    title('Body Velocity - v_y (m/s)')
    
    figure(8)
    plot([solution{cnt}.phase(1).time;solution{cnt}.phase(2).time],...
        [solution{cnt}.phase(1).state(:,10);solution{cnt}.phase(2).state(:,10)]')
    hold on
    title('Joint Velocity - \theta_3 (rad/s)')
    
    figure(9)
    plot([solution{cnt}.phase(1).time;solution{cnt}.phase(2).time],...
        [solution{cnt}.phase(1).control(:,1);solution{cnt}.phase(2).control(:,1)]')
    hold on
    title('Control Input - \tau_1 (Nm)')
    
    figure(10)
    plot([solution{cnt}.phase(1).time;solution{cnt}.phase(2).time],...
        [solution{cnt}.phase(1).control(:,2);solution{cnt}.phase(2).control(:,2)]')
    hold on
    title('Control Input - \tau_2 (Nm)')
    
    figure(11)
    plot([solution{cnt}.phase(1).time;solution{cnt}.phase(2).time],...
        [solution{cnt}.phase(1).control(:,3);solution{cnt}.phase(2).control(:,3)]')
    hold on
    title('Control Input - \tau_3 (Nm)')
    
    cnt = cnt + 1;
end
for i = 1:8
    figure(i)
    legend(['Iter: ',num2str(iter_max(1))],...
        ['Iter: ',num2str(iter_max(2))],...
        ['Iter: ',num2str(iter_max(3))],...
        ['Iter: ',num2str(iter_max(4))],...
        ['Iter: ',num2str(iter_max(5))],...
        ['Iter: ',num2str(iter_max(6))],...
        ['Iter: ',num2str(iter_max(7))],...
        ['Iter: ',num2str(iter_max(8))])
end



% %% Plot
% plot_solution2(interp_sol,bounds,p);
% 
% %% Compare and Animate simulated solution
% simulate_optimal_solution(interp_sol,p)
