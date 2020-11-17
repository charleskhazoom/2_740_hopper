function [t_out, z_out, t_stance, z_stance] = simulate_optimal_solution_casadi(time,z0,tau_motor,p)

%% Simulate Stance
dt = time(2) - time(1);
t_out = [];
z_out = [];
for i = 1:length(time)-1
    [t_,z_] = ode45(@(t,z) get_dynamics_stance(t,z,tau_motor(:,i),p),...
        [time(i) time(i+1)],z0');
    t_out = [t_out;t_];
    z_out = [z_out;z_];
    z0 = z_out(end,:)';
end

t_stance = t_out;
z_stance = z_out;

%% Simulate Flight
t_flight = time(end);
while (t_flight < time(end)+0.3)
    [t_,z_] = ode45(@(t,z) get_dynamics_flight(t,z,[0;0;0],p),...
        [t_flight t_flight+dt],z0');
    t_out = [t_out;t_];
    z_out = [z_out;z_];
    z0 = z_out(end,:)';
    t_flight = t_flight+dt;
end

end

function dz = get_dynamics_stance(t,z,tau,p)
% here we solve for xdd,ydd and constraint for F

% Get mass matrix
A = A_stance(z,p);

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


function dz = get_dynamics_flight(t,z,tau,p)

% Get mass matrix
A = A_floating(z,p);

% Get b = Q - V(q,qd) - G(q)
b = b_floating(z,tau,p,[0;0]);

% Solve for qdd.
qdd = A\(b);

dz = 0*z; % initialize dz

% Form dz
dz(1:5) = z(6:10);
dz(6:10) = qdd;

end