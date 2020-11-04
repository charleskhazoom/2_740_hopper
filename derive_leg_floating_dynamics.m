clear
path_sym_dynamics = 'dynamics_gen/';
addpath(path_sym_dynamics)

write_dynamics_files = true;
% Define variables for time, generalized coordinates + derivatives, controls, and parameters 
syms t x y th1 th2 th3 dx dy dth1 dth2 dth3 ddx ddy ddth1 ddth2 ddth3 real
syms m1 m2 m3 m4 m_body m_arm I1 I2 I3 I4 I_arm l_O_m1 l_B_m2 l_A_m3 l_C_m4 l_cm_arm l_cm_body g real
syms l_OA l_OB l_AC l_DE l_arm l_body real 
syms tau1 tau2 tau3 Fx Fy real
syms Ir N real

% Group them
q   = [x;y;th1  ; th2 ; th3];      % generalized coordinates
dq  = [dx;dy;dth1 ; dth2; dth3];    % first time derivatives
ddq = [ddx;ddy;ddth1;ddth2;ddth3];  % second time derivatives
u   = [tau1 ; tau2 ; tau3];     % controls
F   = [Fx ; Fy];

p   = [m1 m2 m3 m4 m_body m_arm I1 I2 I3 I4 I_arm Ir N l_O_m1 l_B_m2...
    l_A_m3 l_C_m4 l_cm_arm l_cm_body l_OA l_OB l_AC l_DE l_body l_arm g]';        % parameters

% Generate Vectors and Derivativess
ihat = [0; -1; 0];
jhat = [1; 0; 0];

xhat = [1; 0; 0];
yhat = [0; 1; 0];

khat = cross(ihat,jhat);
e1hat =  cos(th1)*ihat + sin(th1)*jhat;
e2hat =  cos(th1+th2)*ihat + sin(th1+th2)*jhat;
e3hat = -cos(th3)*ihat-sin(th3)*jhat;

ddt = @(r) jacobian(r,[q;dq])*[dq;ddq]; % a handy anonymous function for taking time derivatives

r_origin = x*jhat - y*ihat; % origin is hip 
rA = l_OA * e1hat + r_origin;
rB = l_OB * e1hat + r_origin;
rC = rA  + l_AC * e2hat;
rD = rB  + l_AC * e2hat;
rE = rD  + l_DE * e1hat;
r_arm_base = r_origin - l_body*ihat;
r_arm_tip = r_arm_base +l_arm*e3hat;

r_m1 = l_O_m1 * e1hat + r_origin;
r_m2 = rB + l_B_m2 * e2hat;
r_m3 = rA + l_A_m3 * e2hat;
r_m4 = rC + l_C_m4 * e1hat;
r_arm_cm = r_arm_base +l_cm_arm*e3hat; 
r_body_cm = r_origin - l_cm_body*ihat; 

drA = ddt(rA);
drB = ddt(rB);
drC = ddt(rC);
drD = ddt(rD);
drE = ddt(rE);
dr_arm_base = ddt(r_arm_base);
dr_arm_tip = ddt(r_arm_tip);


dr_m1 = ddt(r_m1);
dr_m2 = ddt(r_m2);
dr_m3 = ddt(r_m3);
dr_m4 = ddt(r_m4);
dr_arm_cm = ddt(r_arm_cm);
dr_body_cm = ddt(r_body_cm);

% Calculate Kinetic Energy, Potential Energy, and Generalized Forces
F2Q = @(F,r) simplify(jacobian(r,q)'*(F));    % force contributions to generalized forces
M2Q = @(M,w) simplify(jacobian(w,dq)'*(M));   % moment contributions to generalized forces


omega1 = dth1;
omega2 = dth1 + dth2;
omega3 = dth1 + dth2;
omega4 = dth1;

T1 = (1/2)*m1 * dot(dr_m1,dr_m1) + (1/2) * I1 * omega1^2;
T2 = (1/2)*m2 * dot(dr_m2,dr_m2) + (1/2) * I2 * omega2^2;
T3 = (1/2)*m3 * dot(dr_m3,dr_m3) + (1/2) * I3 * omega3^2;
T4 = (1/2)*m4 * dot(dr_m4,dr_m4) + (1/2) * I4 * omega4^2;
T_body = (1/2)*m_body * dot(dr_body_cm,dr_body_cm); 
T_arm = (1/2)*m_arm * dot(dr_arm_cm,dr_arm_cm) + (1/2) * I_arm * dth3^2;

T1r = (1/2)*Ir*(N*dth1)^2;
T2r = (1/2)*Ir*(dth1 + N*dth2)^2;
T3r = (1/2)*Ir*(N*dth3)^2;


Vg1 = m1*g*dot(r_m1, -ihat);
Vg2 = m2*g*dot(r_m2, -ihat);
Vg3 = m3*g*dot(r_m3, -ihat);
Vg4 = m4*g*dot(r_m4, -ihat);
Vg_arm = m_arm*g*dot(r_arm_cm, -ihat);
Vg_body = m_body*g*dot(r_body_cm, -ihat);

T = simplify(T1 + T2 + T3 + T4 + T_body + T_arm + T1r + T2r + T3r);
Vg = Vg1 + Vg2 + Vg3 + Vg4 + Vg_arm + Vg_body;

Q_tau1 = M2Q(tau1*khat,omega1*khat);
Q_tau2 = M2Q(tau2*khat,omega2*khat); 
Q_tau2R= M2Q(-tau2*khat,omega1*khat);


Q_tau3 = M2Q(tau3*khat,dth3*khat);

Q_F = F2Q(F,rE(1:2));


Q_tau = Q_tau1+Q_tau2 + Q_tau2R + Q_tau3;

Q = Q_tau + Q_F;


% Assemble the array of cartesian coordinates of the key points
keypoints = [rA(1:2) rB(1:2) rC(1:2) rD(1:2) rE(1:2) r_origin(1:2) r_arm_base(1:2) r_arm_tip(1:2)];

%% 
% Derive Energy Function and Equations of Motion
E = T+Vg;
L = T-Vg;
%% equations flight

eom = ddt(jacobian(L,dq).') - jacobian(L,q).' - Q;


name = 'floating';
% Rearrange Equations of Motion
A_flight = jacobian(eom,ddq);
b_flight = A_flight*ddq - eom;

% Equations of motion are
% eom = A *ddq + (coriolis term) + (gravitational term) - Q = 0
Mass_floating_base = A_flight;
Grav_floating_base = simplify(jacobian(Vg, q)');
Corr_floating_base = simplify( eom + Q - Grav_floating_base - A_flight*ddq);

% Compute foot jacobian
J = jacobian(rE,q);

% Compute ddt( J )
dJ= reshape( ddt(J(:)) , size(J) );

% Write Energy Function and Equations of Motion
z  = [q ; dq];

rE = rE(1:2);
drE= drE(1:2);
J  = J(1:2,1:5);
dJ = dJ(1:2,1:5);

% com position
com_pos = (r_m1*m1+r_m2*m2+r_m3*m3+r_m4*m4+r_arm_cm*m_arm+r_body_cm*m_body)/...
    (m1+m2+m3+m4+m_arm+m_body);

com_vel = (dr_m1*m1 + dr_m2*m2 + dr_m3*m3 + dr_m4*m4 + dr_arm_cm*m_arm+dr_body_cm*m_body)/...
    (m1+m2+m3+m4+m_arm+m_body);

%for flight phase, use floating_base functions created here with F = [0;0];
if write_dynamics_files==true
    matlabFunction(A_flight,'file',[path_sym_dynamics,'A_', name],'vars',{z p});
    matlabFunction(b_flight,'file',[path_sym_dynamics,'b_', name],'vars',{z u p F});
    matlabFunction(E,'file',[path_sym_dynamics,'energy_', name],'vars',{z p});
    matlabFunction(rE,'file',[path_sym_dynamics,'position_foot'],'vars',{z p});
    matlabFunction(drE,'file',[path_sym_dynamics,'velocity_foot'],'vars',{z p});
    matlabFunction(J ,'file',[path_sym_dynamics,'jacobian_foot'],'vars',{z p});
    matlabFunction(dJ ,'file',[path_sym_dynamics,'jacobian_dot_foot'],'vars',{z p});
    matlabFunction(Vg ,'file',[path_sym_dynamics,'energy_pot_gravity'],'vars',{z p});
    matlabFunction(com_pos ,'file',[path_sym_dynamics,'com_pos'],'vars',{z p});
    matlabFunction(com_vel ,'file',[path_sym_dynamics,'com_vel'],'vars',{z p});
    matlabFunction(Grav_floating_base ,'file', [path_sym_dynamics,'Grav_leg'] ,'vars',{z p});
    matlabFunction(Corr_floating_base ,'file', [path_sym_dynamics,'Corr_leg']     ,'vars',{z p});
    matlabFunction(keypoints,'file',[path_sym_dynamics,'keypoints_', name],'vars',{z p});
end


%% constraints in stance

name = 'stance';
% Position constraint of end effector
end_eff_constr = ddt(drE); % this must be equal to zero when leg on the ground

eom_augmented = [eom;end_eff_constr];
x_augmented = [ddq;F];


A_stance = jacobian(eom_augmented,x_augmented);
b_stance = A_stance*x_augmented - eom_augmented;

% for stance phase : 
% solve for x_augmented using x_augmented = A_stance\b_augmented
% x_augmented constains [ddq;F] = [ddx;ddy;ddth1;ddth2;ddth3;Fx;Fy];
% then, during stance, ddx,ddy,Fx,Fy can be computed. They can subsequently be used
% used in the more general ''floating base'' functions


if write_dynamics_files==true 
    matlabFunction(A_stance,'file',[path_sym_dynamics, 'A_', name],'vars',{z p});
    matlabFunction(b_stance,'file',[path_sym_dynamics ,'b_', name],'vars',{z u p});
end

