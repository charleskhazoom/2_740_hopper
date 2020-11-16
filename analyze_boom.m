clear;close all;clc;
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
    l_arm = 5*0.0254;
    l_cm_arm = 0.8*l_arm;
    l_cm_body=l_body/2;% assume body com is at half of the body length (makes sense since main body is composed of two motors (hip+arm) + brackets. com will be ~between both motors

    m_arm = 0.1; % 100 grams ?
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
 h_boom = 0.2281; % to be adjusted for ground.
 hob = 91.3/1000;
 k = 0.2877; % Nm/rad
    %% Parameter vector
 
 p   = [m1 m2 m3 m4 m_body m_arm I1 I2 I3 I4 I_arm Ir N l_O_m1 l_B_m2...
    l_A_m3 l_C_m4 l_cm_arm l_cm_body l_OA l_OB l_AC l_DE l_body l_arm g...
    m_offset_x m_offset_y l_boom h_boom hob k motor_kt motor_R]';        % parameters


y= [0:0.01:0.5];
z = [y*0;y;y*0;y*0;y*0;y*0;y*0;y*0;y*0;y*0];
th_boom = angle_boom(z,p)*180/pi;
F_boom  = Force_boom(z,p);


figure;plot(y,th_boom);hold on;

plot(y,F_boom);
xlabel(' y = Robot Height (m)');
legend('boom angle (deg)','boom force (N)');