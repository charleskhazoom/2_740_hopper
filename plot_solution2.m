function plot_solution2(solution,bounds,p)

%% Com, body, joints
num_phase = length(solution.phase);

for j = 1:num_phase

for i = 1:length(solution.phase(j).time)
    solution.phase(j).rCOM(:,i) = com_pos(solution.phase(j).state(i,:)',p);
    solution.phase(j).drCOM(:,i) = com_vel(solution.phase(j).state(i,:)',p);
end

end

lw = [2.5 1.25];
lwB = 2.0;
ls = {'-','--'};

figure()
subplot(3,2,1) % com pos
for j = 1:num_phase
    plot(solution.phase(j).time,solution.phase(j).rCOM(1,:),['c',ls{j}],'Linewidth',lw(j))
    hold on
    plot(solution.phase(j).time,solution.phase(j).rCOM(2,:),['y',ls{j}],'Linewidth',lw(j))
end
xlabel('Time (s)')
ylabel('CoM Position (m)')
title('Position')
legend('x','y')

subplot(3,2,2) % com velocity
for j = 1:num_phase
    plot(solution.phase(j).time,solution.phase(j).drCOM(1,:),['c',ls{j}],'Linewidth',lw(j))
    hold on
    plot(solution.phase(j).time,solution.phase(j).drCOM(2,:),['y',ls{j}],'Linewidth',lw(j))
end
xlabel('Time (s)')
ylabel('CoM Velocity (m/s)')
title('Velocity')
legend('v_x','v_y')

subplot(3,2,3) % base pos
for j = 1:num_phase
    plot(solution.phase(j).time,solution.phase(j).state(:,1)',['r',ls{j}],'Linewidth',lw(j))
    hold on
    plot(solution.phase(j).time,solution.phase(j).state(:,2)',['b',ls{j}],'Linewidth',lw(j))
    plot(solution.phase(j).time,bounds.phase(j).state.lower(1).*ones(1,length(solution.phase(j).time)),'r:','Linewidth',lwB)
    plot(solution.phase(j).time,bounds.phase(j).state.upper(1).*ones(1,length(solution.phase(j).time)),'r:','Linewidth',lwB)
    plot(solution.phase(j).time,bounds.phase(j).state.lower(2).*ones(1,length(solution.phase(j).time)),'b:','Linewidth',lwB)
    plot(solution.phase(j).time,bounds.phase(j).state.upper(2).*ones(1,length(solution.phase(j).time)),'b:','Linewidth',lwB)
end
xlabel('Time (s)')
ylabel('Base Position (m)')
legend('x','y')

subplot(3,2,4) % base velocity
for j = 1:num_phase
plot(solution.phase(j).time,solution.phase(j).state(:,6)',['r',ls{j}],'Linewidth',lw(j))
hold on
plot(solution.phase(j).time,solution.phase(j).state(:,7)',['b',ls{j}],'Linewidth',lw(j))
plot(solution.phase(j).time,bounds.phase(j).state.lower(6).*ones(1,length(solution.phase(j).time)),'r:','Linewidth',lwB)
plot(solution.phase(j).time,bounds.phase(j).state.upper(6).*ones(1,length(solution.phase(j).time)),'r:','Linewidth',lwB)
plot(solution.phase(j).time,bounds.phase(j).state.lower(7).*ones(1,length(solution.phase(j).time)),'b:','Linewidth',lwB)
plot(solution.phase(j).time,bounds.phase(j).state.upper(7).*ones(1,length(solution.phase(j).time)),'b:','Linewidth',lwB)
end
xlabel('Time (s)')
ylabel('Base Velocity (m/s)')
legend('v_x','v_y')

subplot(3,2,5) % joint pos
for j = 1:num_phase
    plot(solution.phase(j).time,solution.phase(j).state(:,3)',['m',ls{j}],'Linewidth',lw(j))
    hold on
    plot(solution.phase(j).time,solution.phase(j).state(:,4)',['g',ls{j}],'Linewidth',lw(j))
    plot(solution.phase(j).time,solution.phase(j).state(:,5)',['k',ls{j}],'Linewidth',lw(j))
    plot(solution.phase(j).time,bounds.phase(j).state.lower(3).*ones(1,length(solution.phase(j).time)),'m:','Linewidth',lwB)
    plot(solution.phase(j).time,bounds.phase(j).state.upper(3).*ones(1,length(solution.phase(j).time)),'m:','Linewidth',lwB)
    plot(solution.phase(j).time,bounds.phase(j).state.lower(4).*ones(1,length(solution.phase(j).time)),'g:','Linewidth',lwB)
    plot(solution.phase(j).time,bounds.phase(j).state.upper(4).*ones(1,length(solution.phase(j).time)),'g:','Linewidth',lwB)
    plot(solution.phase(j).time,bounds.phase(j).state.lower(5).*ones(1,length(solution.phase(j).time)),'k:','Linewidth',lwB)
    plot(solution.phase(j).time,bounds.phase(j).state.upper(5).*ones(1,length(solution.phase(j).time)),'k:','Linewidth',lwB)
end
xlabel('Time (s)')
ylabel('Joint Angle (rad)')
legend('\theta_1','\theta_2','\theta_3')

subplot(3,2,6) % joint velocity
for j = 1:num_phase
    plot(solution.phase(j).time,solution.phase(j).state(:,8)',['m',ls{j}],'Linewidth',lw(j))
    hold on
    plot(solution.phase(j).time,solution.phase(j).state(:,9)',['g',ls{j}],'Linewidth',lw(j))
    plot(solution.phase(j).time,solution.phase(j).state(:,10)',['k',ls{j}],'Linewidth',lw(j))
    plot(solution.phase(j).time,bounds.phase(j).state.lower(8).*ones(1,length(solution.phase(j).time)),'m:','Linewidth',lwB)
    plot(solution.phase(j).time,bounds.phase(j).state.upper(8).*ones(1,length(solution.phase(j).time)),'m:','Linewidth',lwB)
    plot(solution.phase(j).time,bounds.phase(j).state.lower(9).*ones(1,length(solution.phase(j).time)),'g:','Linewidth',lwB)
    plot(solution.phase(j).time,bounds.phase(j).state.upper(9).*ones(1,length(solution.phase(j).time)),'g:','Linewidth',lwB)
    plot(solution.phase(j).time,bounds.phase(j).state.lower(10).*ones(1,length(solution.phase(j).time)),'k:','Linewidth',lwB)
    plot(solution.phase(j).time,bounds.phase(j).state.upper(10).*ones(1,length(solution.phase(j).time)),'k:','Linewidth',lwB)
end
xlabel('Time (s)')
ylabel('Joint Velocity (rad/s)')
legend('d\theta_1/dt','d\theta_2/dt','d\theta_3/dt')

%% Control Input
figure()
subplot(2,1,1)
for i = 1:num_phase
    plot(solution.phase(j).time,solution.phase(j).control(:,1)',['m',ls{j}],'Linewidth',lw(j))
    hold on
    plot(solution.phase(j).time,solution.phase(j).control(:,2)',['g',ls{j}],'Linewidth',lw(j))
    plot(solution.phase(j).time,solution.phase(j).control(:,3)',['k',ls{j}],'Linewidth',lw(j))
    plot(solution.phase(j).time,bounds.phase(j).control.lower(1).*ones(1,length(solution.phase(j).time)),'m:','Linewidth',lwB)
    plot(solution.phase(j).time,bounds.phase(j).control.upper(1).*ones(1,length(solution.phase(j).time)),'m:','Linewidth',lwB)
    plot(solution.phase(j).time,bounds.phase(j).control.lower(2).*ones(1,length(solution.phase(j).time)),'g:','Linewidth',lwB)
    plot(solution.phase(j).time,bounds.phase(j).control.upper(2).*ones(1,length(solution.phase(j).time)),'g:','Linewidth',lwB)
    plot(solution.phase(j).time,bounds.phase(j).control.lower(3).*ones(1,length(solution.phase(j).time)),'k:','Linewidth',lwB)
    plot(solution.phase(j).time,bounds.phase(j).control.upper(3).*ones(1,length(solution.phase(j).time)),'k:','Linewidth',lwB)
end
xlabel('Time (s)')
ylabel('Control Input (Nm)')
legend('\tau_1','\tau_2','\tau_3')

subplot(2,1,2)
kt = p(27);
R  = p(28);
N  = p(13);
for j = 1:num_phase
    solution.phase(j).V(1,:) = (solution.phase(j).control(:,1)'/N)*R/kt + kt*solution.phase(j).state(:,8)'*N;
    solution.phase(j).V(2,:) = (solution.phase(j).control(:,2)'/N)*R/kt + kt*solution.phase(j).state(:,9)'*N;
    solution.phase(j).V(3,:) = (solution.phase(j).control(:,3)'/N)*R/kt + kt*solution.phase(j).state(:,10)'*N;
    
    plot(solution.phase(j).time,solution.phase(1).V(1,:),['m',ls{j}],'Linewidth',lw(j))
    hold on;
    plot(solution.phase(j).time,solution.phase(j).V(2,:),['g',ls{j}],'Linewidth',lw(j))
    plot(solution.phase(j).time,solution.phase(j).V(3,:),['k',ls{j}],'Linewidth',lw(j))
    plot(solution.phase(j).time,bounds.phase(j).path.lower(3).*ones(1,length(solution.phase(j).time)),'m:','Linewidth',lwB)
    plot(solution.phase(j).time,bounds.phase(j).path.upper(3).*ones(1,length(solution.phase(j).time)),'m:','Linewidth',lwB)
    plot(solution.phase(j).time,bounds.phase(j).path.lower(4).*ones(1,length(solution.phase(j).time)),'g:','Linewidth',lwB)
    plot(solution.phase(j).time,bounds.phase(j).path.upper(4).*ones(1,length(solution.phase(j).time)),'g:','Linewidth',lwB)
    plot(solution.phase(j).time,bounds.phase(j).path.lower(5).*ones(1,length(solution.phase(j).time)),'k:','Linewidth',lwB)
    plot(solution.phase(j).time,bounds.phase(j).path.upper(5).*ones(1,length(solution.phase(j).time)),'k:','Linewidth',lwB)
end
xlabel('Time (s)')
ylabel('Voltage (V)')
legend('V_1','V_2','V_3')

%% Ground Reaction Force
for j = 1:num_phase
    solution.phase(j).Fgnd = zeros(2,length(solution.phase(j).time));
    
    if j == 1
        for i = 1:length(solution.phase(j).time) % stance
            A = A_stance(solution.phase(j).state(i,:)',p);
            b = b_stance(solution.phase(j).state(i,:)',solution.phase(j).control(i,:)',p);
            x_augmented = A\(b);
            solution.phase(j).Fgnd(:,i) = x_augmented(6:7); % vertical constraint force
        end
    end
    
    figure()
    plot(solution.phase(j).time,solution.phase(j).Fgnd(1,:),['r',ls{j}],'Linewidth',lw(j))
    hold on
    plot(solution.phase(j).time,solution.phase(j).Fgnd(2,:),['b',ls{j}],'Linewidth',lw(j))
    plot(solution.phase(j).time,bounds.phase(j).path.lower(1).*solution.phase(j).Fgnd(2,:),'r:','Linewidth',lwB)
    plot(solution.phase(j).time,bounds.phase(j).path.upper(1).*solution.phase(j).Fgnd(2,:),'r:','Linewidth',lwB)
    plot(solution.phase(j).time,bounds.phase(j).path.lower(2).*ones(1,length(solution.phase(j).time)),'b:','Linewidth',lwB)
    plot(solution.phase(j).time,bounds.phase(j).path.upper(2).*ones(1,length(solution.phase(j).time)),'b:','Linewidth',lwB)
end
xlabel('Time (s)')
ylabel('Ground Reaction Force (N)')
legend('F_x','F_y')






