clear all; clc;

%% LOAD RESULTS FROM TRAJECTORY OPTIMIZATION

data = load('some_traj_for_se_hwan.mat'); % load in trajectory data

% opt_parameters = []; % total traj. time, dt, etc., as needed
torque_profile = data.traj.torques;
torque_profile = torque_profile(:,1:25);
%torque_profile = repmat([0,0,0]',1,25);


q_profile = data.traj.q;
q_profile = q_profile(:,1:25);
%q_profile = repmat([pi/3, -pi/2, 3*pi/2]',1,25);

qd_profile = data.traj.qd;
qd_profile = qd_profile(:,1:25);
%qd_profile = repmat([0,0,0]',1,25);

% arm direction is good :))
% hip direction is also negative :))
% knee direction is opposite defined direction (SIM: CW positive, IRL: CCW positive)

%% INITIALIZATION

% Initial leg angles for encoder resets (negative of q1,q2 in lab handout due to direction motors are mounted)
angle1_init = 0;            % hip angle % -75 deg
angle2_init = -pi/2;        % knee angle % 32 deg
angle3_init = 0;            % arm angle  % 142 deg (at extremes, min max)

q0 = [angle1_init angle2_init angle3_init]';

% Total experiment time is buffer,trajectory,buffer
traj_time         = 1;    % this should be from opt_parameters
pre_buffer_time   = 5;      % give hopper 3s to reach initial config 
post_buffer_time  = 3;

time_param = [traj_time pre_buffer_time post_buffer_time]';

% Gains for joint impedance controller
gains.K_q1 = 2;             % proportional gain for hip angle
gains.K_q2 = 2;             % proportional gain for knee angle
gains.K_q3 = 2;             % proportional gain for arm angle

gains.D_qd1 = .1;            % derivative gain for hip angle
gains.D_qd2 = .1;            % derivative gain for knee angle
gains.D_qd3 = .1;            % derivative gain for arm angle

% Maximum duty cycle commanded by controller (should always be <=1.0)
duty_max   = 0.5;            % initialize to be low, change to 1.0 once tested successfully

% Controller choice: 0 = ffwd torque, 1 = PD joint control, 2 = both
control_method = 0;

%% Run Experiment
[output_data] = experiment_dataHandler( time_param, q0, gains, control_method,...
                                       duty_max, torque_profile, q_profile, qd_profile);

                                   
%% Extract data (TBD, discuss what we want to look at when experiment is ready)
t = output_data(:,1);

%% Plotting (TBD, discuss what we want to look at when experiment is ready)

