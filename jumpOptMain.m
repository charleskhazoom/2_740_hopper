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
    m_body = 0.186+0.211;%
    l_body = 0.04;
    l_arm = 0.1;
    l_cm_arm = 0.8*l_arm;
    l_cm_body=l_body/2;% assume body com is at half of the body length (makes sense since main body is composed of two motors (hip+arm) + brackets. com will be ~between both motors

    m_arm = 0.1; % 100 grams ?
    I_arm = m_arm*l_cm_arm^2;
%     restitution_coeff = 0.;
%     friction_coeff = 0.3;
    ground_height = 0;
    %% Parameter vector


mu = 0.8; % friction coef
max_voltage = 12; % volts
motor_kt = 0.18;
motor_R = 2;
    
p   = [m1 m2 m3 m4 m_body m_arm I1 I2 I3 I4 I_arm Ir N l_O_m1 l_B_m2...
    l_A_m3 l_C_m4 l_cm_arm l_cm_body l_OA l_OB l_AC l_DE l_body l_arm g motor_kt motor_R]';        % parameters


auxdata.p = p;



%% Set Bounds
desired_hip_pos0 = [0.03;0.08];
guess_leg_angle  = [10*pi/180; 10*pi/180];
init_leg_angle = fsolve(@(x)solve_init_pose(x,desired_hip_pos0,p),guess_leg_angle);
init_arm_angle=0;
z0 = [desired_hip_pos0;init_leg_angle;init_arm_angle;0;0;0;0;0];

bounds.phase(1).initialtime.lower = 0;
bounds.phase(1).initialtime.upper = 0;

bounds.phase(1).finaltime.lower = 0.1;
bounds.phase(1).finaltime.upper = 0.1;

bounds.phase(1).initialstate.lower = z0';
bounds.phase(1).initialstate.upper = z0';

bounds.phase(1).state.lower = [-1 -1 -2*pi -2*pi -2*pi -10 -10 -40 -40 -40];
bounds.phase(1).state.upper = [3 3 2*pi 2*pi 2*pi 10 10 40 40 40];

bounds.phase(1).finalstate.lower = [-1 -1 -2*pi -2*pi -2*pi -10 -10 -40 -40 -40];
bounds.phase(1).finalstate.upper = [3 3 2*pi 2*pi 2*pi 10 10 40 40 40];

bounds.phase(1).control.lower = [-99 -99 -99];
bounds.phase(1).control.upper = [99 99 99];

bounds.phase(1).integral.lower = 0;
bounds.phase(1).integral.upper = 1e5;

% friction bounds for force
bounds.phase(1).path.lower(1) = -mu;
bounds.phase(1).path.upper(1) = mu;

% unilateral bounds for force
bounds.phase(1).path.lower(2) = 0;
bounds.phase(1).path.upper(2) = inf;


% bounds for voltage (3 motors)
for motor = 1:3
    bounds.phase(1).path.lower(end+1) = -max_voltage;
    bounds.phase(1).path.upper(end+1) = max_voltage;
end


%% Initial Guess
guess.phase(1).time = [0;0.5];
guess.phase(1).state = [z0';z0'];
guess.phase(1).control = zeros(2,3);
guess.phase(1).integral = 100;

%% Mesh Setupmesh.method = 'hp-PattersonRao';
mesh.tolerance = 1e-3;
mesh.maxiterations = 20;
mesh.colpointsmin = 4;
mesh.colpointsmax = 10;

%% Problem Setup
setup.name = 'jumpOpt';
setup.functions.continuous = @jumpOptContinuous;
setup.functions.endpoint = @jumpOptEndpoint;
setup.auxdata = auxdata;
setup.bounds = bounds;
setup.guess = guess;
setup.mesh = mesh;
setup.nlp.solver = 'ipopt';
setup.derivatives.supplier = 'sparseCD';
setup.derivatives.derivativelevel = 'second';
setup.method = 'RPM-Differentiation';
setup.scales.method = 'automatic-bounds';

%% Solve
output = gpops2(setup);
solution = output.result.solution;

%% Parse Solution

n_phase = length(solution.phase);
tout =[];
zout = [];
uout = [];
for i = 1:length(n_phase)
    tout = [tout solution.phase(i).time'];
    zout = [zout solution.phase(i).state'];
    uout = [uout solution.phase(i).control'];
end

tLO = solution.phase(1).time(end); % time lift off is the last time of first phase 
%-------------------make all kind of plot---------------------------------%
fignb=1;


figure(fignb);
fignb=fignb+1;
clf;
hold on


% Target traj
plot([-.2 .7],[ground_height ground_height],'k');
animateSol(tout, zout,p);

plot_solution(tout,zout,uout,tLO,p,fignb); %outputs next fignb
