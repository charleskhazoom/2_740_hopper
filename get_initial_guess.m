function [tout_, zout_, uout_] = get_initial_guess(tf,z0,p)
path_sym_dynamics = 'dynamics_gen/';
addpath(path_sym_dynamics)

%% Perform Dynamic simulation
tspan = [0 tf];

% simulate stance
opts1 = odeset('events',@(t,z) detect_LO(t,z,p),'AbsTol',1e-6,'RelTol',1e-6);
[tout_{1},zout_{1}] = ode45(@(t,z) get_dynamics_stance(t,z,p),[tspan(1) tspan(2)],z0',opts1);
tstart = tout_{1}(end);
zstart = zout_{1}(end,:)';
for i =1:length(tout_{1})
        uout_{1}(:,i) = control_law_stance(tout_{1}(i),zout_{1}(i,:)',p);
end

opts2 = odeset('events',@(t,z) detect_TD(t,z,p),'AbsTol',1e-6,'RelTol',1e-6);
[tout_{2},zout_{2}] = ode45(@(t,z) get_dynamics_flight(t,z,p),[tstart:0.002:tspan(2)],zstart,opts2);
for i =1:length(tout_{2})
        uout_{2}(:,i) = control_law_flight(tout_{2}(i),zout_{2}(i,:)',p);
end


end

function tau = control_law_stance(t, z, p)

tau = [-0.5;-5;0];

%
end

function tau = control_law_flight(t, z, p)

%tau = [0.5;0;0.5];
%     tau = [-0.5;-0.5;1];
tau = [0;0;0];
%
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


function [dz,F] = get_dynamics_flight(t,z,p)

% Get mass matrix
A = A_floating(z,p);

% Compute Controls


tau = control_law_flight(t,z,p);

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
tau = control_law_flight(t,z,p);
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
