function simulate_optimal_solution(solution,p)
path_sym_dynamics = 'dynamics_gen/';
addpath(path_sym_dynamics)

%% Simulate Stance
z0 = solution.phase(1).state(1,:)';
t_out = [];
z_out = [];
for i = 1:length(solution.phase(1).time)-1
    [t_,z_] = ode45(@(t,z) get_dynamics_stance(t,z,solution.phase(1).control(i,:)',p),...
        [solution.phase(1).time(i) solution.phase(1).time(i+1)],z0');
    t_out = [t_out;t_];
    z_out = [z_out;z_];
    z0 = z_out(end,:)';
end

%% Simulate Flight
for i = 1:length(solution.phase(2).time)-1
    [t_,z_] = ode45(@(t,z) get_dynamics_flight(t,z,solution.phase(2).control(i,:)',p),...
        [solution.phase(2).time(i) solution.phase(2).time(i+1)],z0');
    t_out = [t_out;t_];
    z_out = [z_out;z_];
    z0 = z_out(end,:)';
end

%% Plot Optimal Solution Versus Simulated
lw = 2.25;
% Figure 1 - base position/velocity
figure()
subplot(2,1,1)
plot(t_out,z_out(:,1)','r','linewidth',lw)
hold on
plot(t_out,z_out(:,2)','b','linewidth',lw)
plot([solution.phase(1).time;solution.phase(2).time],...
    [solution.phase(1).state(:,1);solution.phase(2).state(:,1)]','r--','linewidth',lw)
plot([solution.phase(1).time;solution.phase(2).time],...
    [solution.phase(1).state(:,2);solution.phase(2).state(:,2)]','b--','linewidth',lw)
xlabel('Time (s)')
ylabel('Base Position (m)')
legend('x - simulated','y - simulated','x - optimal','y - optimal')

subplot(2,1,2)
plot(t_out,z_out(:,6)','r','linewidth',lw)
hold on
plot(t_out,z_out(:,7)','b','linewidth',lw)
plot([solution.phase(1).time;solution.phase(2).time],...
    [solution.phase(1).state(:,6);solution.phase(2).state(:,6)]','r--','linewidth',lw)
plot([solution.phase(1).time;solution.phase(2).time],...
    [solution.phase(1).state(:,7);solution.phase(2).state(:,7)]','b--','linewidth',lw)
xlabel('Time (s)')
ylabel('Base Velocity (m/s)')
legend('x - simulated','y - simulated','x - optimal','y - optimal')

% Figure 3 - joint position/velocity
figure()
subplot(2,1,1)
plot(t_out,z_out(:,3)','m','linewidth',lw)
hold on
plot(t_out,z_out(:,4)','g','linewidth',lw)
plot(t_out,z_out(:,5)','k','linewidth',lw)
plot([solution.phase(1).time;solution.phase(2).time],...
    [solution.phase(1).state(:,3);solution.phase(2).state(:,3)]','m--','linewidth',lw)
plot([solution.phase(1).time;solution.phase(2).time],...
    [solution.phase(1).state(:,4);solution.phase(2).state(:,4)]','g--','linewidth',lw)
plot([solution.phase(1).time;solution.phase(2).time],...
    [solution.phase(1).state(:,5);solution.phase(2).state(:,5)]','k--','linewidth',lw)
xlabel('Time (s)')
ylabel('Joint Position (rad)')
legend('\theta_1 - simulated','\theta_2 - simulated','\theta_3 - simulated',...
    '\theta_1 - optimal','\theta_2 - optimal','\theta_3 - optimal')

subplot(2,1,2)
plot(t_out,z_out(:,8)','m','linewidth',lw)
hold on
plot(t_out,z_out(:,9)','g','linewidth',lw)
plot(t_out,z_out(:,10)','k','linewidth',lw)
plot([solution.phase(1).time;solution.phase(2).time],...
    [solution.phase(1).state(:,8);solution.phase(2).state(:,8)]','m--','linewidth',lw)
plot([solution.phase(1).time;solution.phase(2).time],...
    [solution.phase(1).state(:,9);solution.phase(2).state(:,9)]','g--','linewidth',lw)
plot([solution.phase(1).time;solution.phase(2).time],...
    [solution.phase(1).state(:,10);solution.phase(2).state(:,10)]','k--','linewidth',lw)
xlabel('Time (s)')
ylabel('Joint Velocity (rad/s)')
legend('\theta_1 - simulated','\theta_2 - simulated','\theta_3 - simulated',...
    '\theta_1 - optimal','\theta_2 - optimal','\theta_3 - optimal')

%% Animate Simulated Solution
figure()
animateSol([solution.phase(1).time;solution.phase(2).time],...
    [solution.phase(1).state(:,:);solution.phase(2).state(:,:)]',p);

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
