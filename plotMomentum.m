clear
close all
clc

load('time_arm_optimization_10x10.mat') % actually 13 x 13
%% Momentum
% Total mass
total_mass = m1+m2+m3+m4+m_body+m_arm;
body_mass = total_mass - m_arm;

% Pick arms to test
arm_small = 2;
arm_opt = 8;
arm_big = 12;

% Base Momentum
base_mom_small(1:2,:) = body_mass.*qds{4,arm_small}(1:2,:);
base_mom_opt(1:2,:) = body_mass.*qds{4,arm_opt}(1:2,:);
base_mom_big(1:2,:) = body_mass.*qds{4,arm_big}(1:2,:);

% Arm momentum
for i = 1:length(ts{4,arm_small})
    arm_mom_small(1:3,i) = m_arm.*arm_tip_vel([qs{4,arm_small}(:,i);qds{4,arm_small}(:,i)],p);
end
for i = 1:length(ts{4,arm_opt})
    arm_mom_opt(1:3,i) = m_arm.*arm_tip_vel([qs{4,arm_opt}(:,i);qds{4,arm_opt}(:,i)],p);
end
for i = 1:length(ts{4,arm_big})
    arm_mom_big(1:3,i) = m_arm.*arm_tip_vel([qs{4,arm_big}(:,i);qds{4,arm_big}(:,i)],p);
end

% Center of mass momentum
for i = 1:length(ts{4,arm_small})
    com_mom_small(1:3,i) = total_mass.*com_vel([qs{4,arm_small}(:,i);qds{4,arm_small}(:,i)],p);
end
for i = 1:length(ts{4,arm_opt})
    com_mom_opt(1:3,i) = total_mass.*com_vel([qs{4,arm_opt}(:,i);qds{4,arm_opt}(:,i)],p);
end
for i = 1:length(ts{4,arm_big})
    com_mom_big(1:3,i) = total_mass.*com_vel([qs{4,arm_big}(:,i);qds{4,arm_big}(:,i)],p);
end
com_mom_mag_small = sqrt(com_mom_small(1,:).^2+com_mom_small(2,:).^2);
com_mom_mag_opt = sqrt(com_mom_opt(1,:).^2+com_mom_opt(2,:).^2);
com_mom_mag_big = sqrt(com_mom_big(1,:).^2+com_mom_big(2,:).^2);


%% Plotting

% Base, Arm, CoM, x and y plotted together
lt = {'--',':'};
lw = 2.25;
figure()
subplot(3,1,1) % base
for i = 1:2
    plot(ts{4,arm_small},base_mom_small(i,:),['r',lt{i}],'Linewidth',2.25)
    hold on
    plot(ts{4,arm_opt},base_mom_opt(i,:),['g',lt{i}],'Linewidth',2.25)
    plot(ts{4,arm_big},base_mom_big(i,:),['b',lt{i}],'Linewidth',2.25)
end
subplot(3,1,2) % arm
for i = 1:2
    plot(ts{4,arm_small},arm_mom_small(i,:),['r',lt{i}],'Linewidth',2.25)
    hold on
    plot(ts{4,arm_opt},arm_mom_opt(i,:),['g',lt{i}],'Linewidth',2.25)
    plot(ts{4,arm_big},arm_mom_big(i,:),['b',lt{i}],'Linewidth',2.25)
end
subplot(3,1,3) % com
for i = 1:2
    plot(ts{4,arm_small},com_mom_small(i,:),['r',lt{i}],'Linewidth',2.25)
    hold on
    plot(ts{4,arm_opt},com_mom_opt(i,:),['g',lt{i}],'Linewidth',2.25)
    plot(ts{4,arm_big},com_mom_big(i,:),['b',lt{i}],'Linewidth',2.25)
end

% Base, Arm, CoM, x and y plotted together
lw = 2.25;
dir = {'X','Y'};
figure()
for i = 1:2
    subplot(3,2,i) % base
    plot(ts{4,arm_small},base_mom_small(i,:),['r'],'Linewidth',2.25)
    hold on
    plot(ts{4,arm_opt},base_mom_opt(i,:),['g'],'Linewidth',2.25)
    plot(ts{4,arm_big},base_mom_big(i,:),['b'],'Linewidth',2.25)
    title(['Base Momentum - ',dir{i},' Component'])
    ylabel(['m_{base}v_{base,',dir{i},'} (kg\cdot m/s)'])
    xlabel('Time (s)')
    grid on
    if i == 2
        legend('Short Arm','Optimal Arm Length','Long Arm')
    end
end
for i = 1:2
    subplot(3,2,2+i) % arm
    plot(ts{4,arm_small},arm_mom_small(i,:),['r'],'Linewidth',2.25)
    hold on
    plot(ts{4,arm_opt},arm_mom_opt(i,:),['g'],'Linewidth',2.25)
    plot(ts{4,arm_big},arm_mom_big(i,:),['b'],'Linewidth',2.25)
    title(['Arm Momentum - ',dir{i},' Component'])
    ylabel(['m_{arm}v_{arm,',dir{i},'} (kg\cdot m/s)'])
    xlabel('Time (s)')
    grid on
    if i == 2
        legend('Short Arm','Optimal Arm Length','Long Arm')
    end
end
for i = 1:2
    subplot(3,2,4+i) % com
    plot(ts{4,arm_small},com_mom_small(i,:),['r'],'Linewidth',2.25)
    hold on
    plot(ts{4,arm_opt},com_mom_opt(i,:),['g'],'Linewidth',2.25)
    plot(ts{4,arm_big},com_mom_big(i,:),['b'],'Linewidth',2.25)
    title(['CoM Momentum - ',dir{i},' Component'])
    ylabel(['m_{com}v_{com,',dir{i},'} (kg\cdot m/s)'])
    xlabel('Time (s)')
    grid on
    if i == 2
        legend('Short Arm','Optimal Arm Length','Long Arm')
    end
end

% Plot only the magnitude of the CoM momentum
figure()
plot(ts{4,arm_small},com_mom_mag_small(1,:),['r'],'Linewidth',2.25)
hold on
plot(ts{4,arm_opt},com_mom_mag_opt(1,:),['g'],'Linewidth',2.25)
plot(ts{4,arm_big},com_mom_mag_big(1,:),['b'],'Linewidth',2.25)
title('CoM Momentum')
ylabel('||m_{com}v_{com}|| (kg\cdot m/s)')
xlabel('Time (s)')
grid on
legend('Short Arm (2.16in)','Optimal Arm Length (9.15in)','Long Arm (13.83in)')
axis([-inf inf 0 1.4])
set(gca,'Fontsize',18)


