function plot_solution2(solution,bounds,p)

%% Com, body, joints
for i = 1:length(solution.phase(1).time)
    solution.phase(1).rCOM(:,i) = com_pos(solution.phase(1).state(i,:)',p);
    solution.phase(1).drCOM(:,i) = com_vel(solution.phase(1).state(i,:)',p);
end
for i = 1:length(solution.phase(2).time)
    solution.phase(2).rCOM(:,i) = com_pos(solution.phase(2).state(i,:)',p);
    solution.phase(2).drCOM(:,i) = com_vel(solution.phase(2).state(i,:)',p);
end

lwS = 2.5;
lwF = 1.25;
lwB = 2.0;

figure()
subplot(3,2,1) % com pos
plot(solution.phase(1).time,solution.phase(1).rCOM(1,:),'c','Linewidth',lwS)
hold on
plot(solution.phase(1).time,solution.phase(1).rCOM(2,:),'y','Linewidth',lwS)
plot(solution.phase(2).time,solution.phase(2).rCOM(1,:),'c--','Linewidth',lwF)
plot(solution.phase(2).time,solution.phase(2).rCOM(2,:),'y--','Linewidth',lwF)
xlabel('Time (s)')
ylabel('CoM Position (m)')
title('Position')
legend('x','y')

subplot(3,2,2) % com velocity
plot(solution.phase(1).time,solution.phase(1).drCOM(1,:),'c','Linewidth',lwS)
hold on
plot(solution.phase(1).time,solution.phase(1).drCOM(2,:),'y','Linewidth',lwS)
plot(solution.phase(2).time,solution.phase(2).drCOM(1,:),'c--','Linewidth',lwF)
plot(solution.phase(2).time,solution.phase(2).drCOM(2,:),'y--','Linewidth',lwF)
xlabel('Time (s)')
ylabel('CoM Velocity (m/s)')
title('Velocity')
legend('v_x','v_y')

subplot(3,2,3) % base pos
plot(solution.phase(1).time,solution.phase(1).state(:,1)','r','Linewidth',lwS)
hold on
plot(solution.phase(1).time,solution.phase(1).state(:,2)','b','Linewidth',lwS)
plot(solution.phase(1).time,bounds.phase(1).state.lower(1).*ones(1,length(solution.phase(1).time)),'r:','Linewidth',lwB)
plot(solution.phase(1).time,bounds.phase(1).state.upper(1).*ones(1,length(solution.phase(1).time)),'r:','Linewidth',lwB)
plot(solution.phase(1).time,bounds.phase(1).state.lower(2).*ones(1,length(solution.phase(1).time)),'b:','Linewidth',lwB)
plot(solution.phase(1).time,bounds.phase(1).state.upper(2).*ones(1,length(solution.phase(1).time)),'b:','Linewidth',lwB)
plot(solution.phase(2).time,solution.phase(2).state(:,1)','r--','Linewidth',lwF)
plot(solution.phase(2).time,solution.phase(2).state(:,2)','b--','Linewidth',lwF)
plot(solution.phase(2).time,bounds.phase(2).state.lower(1).*ones(1,length(solution.phase(2).time)),'r:','Linewidth',lwB)
plot(solution.phase(2).time,bounds.phase(2).state.upper(1).*ones(1,length(solution.phase(2).time)),'r:','Linewidth',lwB)
plot(solution.phase(2).time,bounds.phase(2).state.lower(2).*ones(1,length(solution.phase(2).time)),'b:','Linewidth',lwB)
plot(solution.phase(2).time,bounds.phase(2).state.upper(2).*ones(1,length(solution.phase(2).time)),'b:','Linewidth',lwB)
xlabel('Time (s)')
ylabel('Base Position (m)')
legend('x','y')

subplot(3,2,4) % base velocity
plot(solution.phase(1).time,solution.phase(1).state(:,6)','r','Linewidth',lwS)
hold on
plot(solution.phase(1).time,solution.phase(1).state(:,7)','b','Linewidth',lwS)
plot(solution.phase(1).time,bounds.phase(1).state.lower(6).*ones(1,length(solution.phase(1).time)),'r:','Linewidth',lwB)
plot(solution.phase(1).time,bounds.phase(1).state.upper(6).*ones(1,length(solution.phase(1).time)),'r:','Linewidth',lwB)
plot(solution.phase(1).time,bounds.phase(1).state.lower(7).*ones(1,length(solution.phase(1).time)),'b:','Linewidth',lwB)
plot(solution.phase(1).time,bounds.phase(1).state.upper(7).*ones(1,length(solution.phase(1).time)),'b:','Linewidth',lwB)
plot(solution.phase(2).time,solution.phase(2).state(:,6)','r--','Linewidth',lwF)
plot(solution.phase(2).time,solution.phase(2).state(:,7)','b--','Linewidth',lwF)
plot(solution.phase(2).time,bounds.phase(2).state.lower(6).*ones(1,length(solution.phase(2).time)),'r:','Linewidth',lwB)
plot(solution.phase(2).time,bounds.phase(2).state.upper(6).*ones(1,length(solution.phase(2).time)),'r:','Linewidth',lwB)
plot(solution.phase(2).time,bounds.phase(2).state.lower(7).*ones(1,length(solution.phase(2).time)),'b:','Linewidth',lwB)
plot(solution.phase(2).time,bounds.phase(2).state.upper(7).*ones(1,length(solution.phase(2).time)),'b:','Linewidth',lwB)
xlabel('Time (s)')
ylabel('Base Velocity (m/s)')
legend('v_x','v_y')

subplot(3,2,5) % joint pos
plot(solution.phase(1).time,solution.phase(1).state(:,3)','m','Linewidth',lwS)
hold on
plot(solution.phase(1).time,solution.phase(1).state(:,4)','g','Linewidth',lwS)
plot(solution.phase(1).time,solution.phase(1).state(:,5)','k','Linewidth',lwS)
plot(solution.phase(1).time,bounds.phase(1).state.lower(3).*ones(1,length(solution.phase(1).time)),'m:','Linewidth',lwB)
plot(solution.phase(1).time,bounds.phase(1).state.upper(3).*ones(1,length(solution.phase(1).time)),'m:','Linewidth',lwB)
plot(solution.phase(1).time,bounds.phase(1).state.lower(4).*ones(1,length(solution.phase(1).time)),'g:','Linewidth',lwB)
plot(solution.phase(1).time,bounds.phase(1).state.upper(4).*ones(1,length(solution.phase(1).time)),'g:','Linewidth',lwB)
plot(solution.phase(1).time,bounds.phase(1).state.lower(5).*ones(1,length(solution.phase(1).time)),'k:','Linewidth',lwB)
plot(solution.phase(1).time,bounds.phase(1).state.upper(5).*ones(1,length(solution.phase(1).time)),'k:','Linewidth',lwB)
plot(solution.phase(2).time,solution.phase(2).state(:,3)','m--','Linewidth',lwF)
plot(solution.phase(2).time,solution.phase(2).state(:,4)','g--','Linewidth',lwF)
plot(solution.phase(2).time,solution.phase(2).state(:,5)','k--','Linewidth',lwF)
plot(solution.phase(2).time,bounds.phase(2).state.lower(3).*ones(1,length(solution.phase(2).time)),'m:','Linewidth',lwB)
plot(solution.phase(2).time,bounds.phase(2).state.upper(3).*ones(1,length(solution.phase(2).time)),'m:','Linewidth',lwB)
plot(solution.phase(2).time,bounds.phase(2).state.lower(4).*ones(1,length(solution.phase(2).time)),'g:','Linewidth',lwB)
plot(solution.phase(2).time,bounds.phase(2).state.upper(4).*ones(1,length(solution.phase(2).time)),'g:','Linewidth',lwB)
plot(solution.phase(2).time,bounds.phase(2).state.lower(5).*ones(1,length(solution.phase(2).time)),'k:','Linewidth',lwB)
plot(solution.phase(2).time,bounds.phase(2).state.upper(5).*ones(1,length(solution.phase(2).time)),'k:','Linewidth',lwB)
xlabel('Time (s)')
ylabel('Joint Angle (rad)')
legend('\theta_1','\theta_2','\theta_3')

subplot(3,2,6) % joint velocity
plot(solution.phase(1).time,solution.phase(1).state(:,8)','m','Linewidth',lwS)
hold on
plot(solution.phase(1).time,solution.phase(1).state(:,9)','g','Linewidth',lwS)
plot(solution.phase(1).time,solution.phase(1).state(:,10)','k','Linewidth',lwS)
plot(solution.phase(1).time,bounds.phase(1).state.lower(8).*ones(1,length(solution.phase(1).time)),'m:','Linewidth',lwB)
plot(solution.phase(1).time,bounds.phase(1).state.upper(8).*ones(1,length(solution.phase(1).time)),'m:','Linewidth',lwB)
plot(solution.phase(1).time,bounds.phase(1).state.lower(9).*ones(1,length(solution.phase(1).time)),'g:','Linewidth',lwB)
plot(solution.phase(1).time,bounds.phase(1).state.upper(9).*ones(1,length(solution.phase(1).time)),'g:','Linewidth',lwB)
plot(solution.phase(1).time,bounds.phase(1).state.lower(10).*ones(1,length(solution.phase(1).time)),'k:','Linewidth',lwB)
plot(solution.phase(1).time,bounds.phase(1).state.upper(10).*ones(1,length(solution.phase(1).time)),'k:','Linewidth',lwB)
plot(solution.phase(2).time,solution.phase(2).state(:,8)','m--','Linewidth',lwF)
plot(solution.phase(2).time,solution.phase(2).state(:,9)','g--','Linewidth',lwF)
plot(solution.phase(2).time,solution.phase(2).state(:,10)','k--','Linewidth',lwF)
plot(solution.phase(2).time,bounds.phase(2).state.lower(8).*ones(1,length(solution.phase(2).time)),'m:','Linewidth',lwB)
plot(solution.phase(2).time,bounds.phase(2).state.upper(8).*ones(1,length(solution.phase(2).time)),'m:','Linewidth',lwB)
plot(solution.phase(2).time,bounds.phase(2).state.lower(9).*ones(1,length(solution.phase(2).time)),'g:','Linewidth',lwB)
plot(solution.phase(2).time,bounds.phase(2).state.upper(9).*ones(1,length(solution.phase(2).time)),'g:','Linewidth',lwB)
plot(solution.phase(2).time,bounds.phase(2).state.lower(10).*ones(1,length(solution.phase(2).time)),'k:','Linewidth',lwB)
plot(solution.phase(2).time,bounds.phase(2).state.upper(10).*ones(1,length(solution.phase(2).time)),'k:','Linewidth',lwB)
xlabel('Time (s)')
ylabel('Joint Velocity (rad/s)')
legend('d\theta_1/dt','d\theta_2/dt','d\theta_3/dt')

%% Control Input
figure()
subplot(2,1,1)
plot(solution.phase(1).time,solution.phase(1).control(:,1)','m','Linewidth',lwS)
hold on
plot(solution.phase(1).time,solution.phase(1).control(:,2)','g','Linewidth',lwS)
plot(solution.phase(1).time,solution.phase(1).control(:,3)','k','Linewidth',lwS)
plot(solution.phase(1).time,bounds.phase(1).control.lower(1).*ones(1,length(solution.phase(1).time)),'m:','Linewidth',lwB)
plot(solution.phase(1).time,bounds.phase(1).control.upper(1).*ones(1,length(solution.phase(1).time)),'m:','Linewidth',lwB)
plot(solution.phase(1).time,bounds.phase(1).control.lower(2).*ones(1,length(solution.phase(1).time)),'g:','Linewidth',lwB)
plot(solution.phase(1).time,bounds.phase(1).control.upper(2).*ones(1,length(solution.phase(1).time)),'g:','Linewidth',lwB)
plot(solution.phase(1).time,bounds.phase(1).control.lower(3).*ones(1,length(solution.phase(1).time)),'k:','Linewidth',lwB)
plot(solution.phase(1).time,bounds.phase(1).control.upper(3).*ones(1,length(solution.phase(1).time)),'k:','Linewidth',lwB)
plot(solution.phase(2).time,solution.phase(2).control(:,1)','m--','Linewidth',lwF)
plot(solution.phase(2).time,solution.phase(2).control(:,2)','g--','Linewidth',lwF)
plot(solution.phase(2).time,solution.phase(2).control(:,3)','k--','Linewidth',lwF)
plot(solution.phase(2).time,bounds.phase(2).control.lower(1).*ones(1,length(solution.phase(2).time)),'m:','Linewidth',lwB)
plot(solution.phase(2).time,bounds.phase(2).control.upper(1).*ones(1,length(solution.phase(2).time)),'m:','Linewidth',lwB)
plot(solution.phase(2).time,bounds.phase(2).control.lower(2).*ones(1,length(solution.phase(2).time)),'g:','Linewidth',lwB)
plot(solution.phase(2).time,bounds.phase(2).control.upper(2).*ones(1,length(solution.phase(2).time)),'g:','Linewidth',lwB)
plot(solution.phase(2).time,bounds.phase(2).control.lower(3).*ones(1,length(solution.phase(2).time)),'k:','Linewidth',lwB)
plot(solution.phase(2).time,bounds.phase(2).control.upper(3).*ones(1,length(solution.phase(2).time)),'k:','Linewidth',lwB)
xlabel('Time (s)')
ylabel('Control Input (Nm)')
legend('\tau_1','\tau_2','\tau_3')

subplot(2,1,2)
kt = p(27); 
R  = p(28);
N  = p(13);
solution.phase(1).V(1,:) = (solution.phase(1).control(:,1)'/N)*R/kt + kt*solution.phase(1).state(:,8)'*N;
solution.phase(1).V(2,:) = (solution.phase(1).control(:,2)'/N)*R/kt + kt*solution.phase(1).state(:,9)'*N;
solution.phase(1).V(3,:) = (solution.phase(1).control(:,3)'/N)*R/kt + kt*solution.phase(1).state(:,10)'*N;
solution.phase(2).V(1,:) = (solution.phase(2).control(:,1)'/N)*R/kt + kt*solution.phase(2).state(:,8)'*N;
solution.phase(2).V(2,:) = (solution.phase(2).control(:,2)'/N)*R/kt + kt*solution.phase(2).state(:,9)'*N;
solution.phase(2).V(3,:) = (solution.phase(2).control(:,3)'/N)*R/kt + kt*solution.phase(2).state(:,10)'*N;

plot(solution.phase(1).time,solution.phase(1).V(1,:),'m','Linewidth',lwS)
hold on;
plot(solution.phase(1).time,solution.phase(1).V(2,:),'g','Linewidth',lwS)
plot(solution.phase(1).time,solution.phase(1).V(3,:),'k','Linewidth',lwS)
plot(solution.phase(1).time,bounds.phase(1).path.lower(3).*ones(1,length(solution.phase(1).time)),'m:','Linewidth',lwB)
plot(solution.phase(1).time,bounds.phase(1).path.upper(3).*ones(1,length(solution.phase(1).time)),'m:','Linewidth',lwB)
plot(solution.phase(1).time,bounds.phase(1).path.lower(4).*ones(1,length(solution.phase(1).time)),'g:','Linewidth',lwB)
plot(solution.phase(1).time,bounds.phase(1).path.upper(4).*ones(1,length(solution.phase(1).time)),'g:','Linewidth',lwB)
plot(solution.phase(1).time,bounds.phase(1).path.lower(5).*ones(1,length(solution.phase(1).time)),'k:','Linewidth',lwB)
plot(solution.phase(1).time,bounds.phase(1).path.upper(5).*ones(1,length(solution.phase(1).time)),'k:','Linewidth',lwB)
plot(solution.phase(2).time,solution.phase(2).V(1,:),'m--','Linewidth',lwF)
plot(solution.phase(2).time,solution.phase(2).V(2,:),'g--','Linewidth',lwF)
plot(solution.phase(2).time,solution.phase(2).V(3,:),'k--','Linewidth',lwF)
xlabel('Time (s)')
ylabel('Voltage (V)')
legend('V_1','V_2','V_3')

%% Ground Reaction Force
solution.phase(1).Fgnd = zeros(2,length(solution.phase(1).time));
solution.phase(2).Fgnd = zeros(2,length(solution.phase(2).time));
for i = 1:length(solution.phase(1).time) % stance
    A = A_stance(solution.phase(1).state(i,:)',p);
    b = b_stance(solution.phase(1).state(i,:)',solution.phase(1).control(i,:)',p);
    x_augmented = A\(b);
    solution.phase(1).Fgnd(:,i) = x_augmented(6:7); % vertical constraint force   
end
figure()
plot(solution.phase(1).time,solution.phase(1).Fgnd(1,:),'r','Linewidth',lwS)
hold on
plot(solution.phase(1).time,solution.phase(1).Fgnd(2,:),'b','Linewidth',lwS)
plot(solution.phase(1).time,bounds.phase(1).path.lower(1).*solution.phase(1).Fgnd(2,:),'r:','Linewidth',lwB)
plot(solution.phase(1).time,bounds.phase(1).path.upper(1).*solution.phase(1).Fgnd(2,:),'r:','Linewidth',lwB)
plot(solution.phase(1).time,bounds.phase(1).path.lower(2).*ones(1,length(solution.phase(1).time)),'b:','Linewidth',lwB)
plot(solution.phase(1).time,bounds.phase(1).path.upper(2).*ones(1,length(solution.phase(1).time)),'b:','Linewidth',lwB)
plot(solution.phase(2).time,solution.phase(2).Fgnd(1,:),'r--','Linewidth',lwF)
plot(solution.phase(2).time,solution.phase(2).Fgnd(2,:),'b--','Linewidth',lwF)
xlabel('Time (s)')
ylabel('Ground Reaction Force (N)')
legend('F_x','F_y')






