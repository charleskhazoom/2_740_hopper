function output = jumpOptEndpointStanceOnly(input)


% Should probably get CoM position and height rather than body position and
% height...
x = input.phase(1).finalstate(end,1);
z = input.phase(1).finalstate(end,2);
vx = input.phase(1).finalstate(end,6);
vz = input.phase(1).finalstate(end,7);
g = input.auxdata.p(26);

t_flight = (1/g) * (vz + sqrt(vz^2 + 2*z*g));
x_land = x + vx*t_flight;

% % Cost Function - max body height with minor torque minimization
% output.objective = -1*input.phase(1).finalstate(end,1) +...
%     -2*input.phase(1).finalstate(end,2) +...
%     .0005*input.phase(1).integral(1) +...
%     .0005*input.phase(1).integral(2);
% 
% 
% % Cost Function - max body velocity with minor torque minimization
% output.objective = -1*input.phase(1).finalstate(end,6) +...
%     -2*input.phase(1).finalstate(end,7) +...
%     .0005*input.phase(1).integral(1) +...
%     .0005*input.phase(1).integral(2);

% Cost Function - max x land position with minor torque minimization
output.objective = -2*x_land +...
    .0005*input.phase(1).integral(1) +...
    .0005*input.phase(1).integral(2);