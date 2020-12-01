clear
close all
clc

load('time_arm_optimization_10x10.mat') % actually 13 x 13

%% Jacobians
% Assume unit torque at each joint

% Pick arms to test
arm_small = 2;
arm_opt = 8;
arm_big = 12;

tau_max = 2.16;
for i = 1:length(ts{4,arm_small})
    J = jacobian_foot([qs{4,arm_small}(:,i);qds{4,arm_small}(:,i)],p);
    J = J(1:2,3:4);
    fmax_small(:,i) = (J'*[tau_max;tau_max])';
    J = jacobian_foot([qs{4,arm_opt}(:,i);qds{4,arm_opt}(:,i)],p);
    J = J(1:2,3:4);
    fmax_opt(:,i) = (J'*[tau_max;tau_max])';
    J = jacobian_foot([qs{4,arm_big}(:,i);qds{4,arm_big}(:,i)],p);
    J = J(1:2,3:4);
    fmax_big(:,i) = (J'*[tau_max;tau_max])';
end

fmax_mag_small = sqrt(fmax_small(1,:).^2+fmax_small(2,:).^2);
fmax_mag_opt = sqrt(fmax_opt(1,:).^2+fmax_opt(2,:).^2);
fmax_mag_big = sqrt(fmax_big(1,:).^2+fmax_big(2,:).^2);

%% Plot
figure()
plot(ts{4,arm_small},fmax_mag_small,'r','Linewidth',2.25)
hold on
plot(ts{4,arm_small},fmax_mag_opt,'g','Linewidth',2.25)
plot(ts{4,arm_small},fmax_mag_big,'b','Linewidth',2.25)
grid on
xlabel('Time (s)')
ylabel('||(J^T)^(-1)\tau|| (N)')
title('Max Achievable Ground Reaction Force ')
set(gca,'Fontsize',18)
legend('Short Arm (2.16in)','Optimal Arm Length (9.15in)','Long Arm (13.83in)')
axis([-inf inf 0.2 0.4])
