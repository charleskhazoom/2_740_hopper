function plot_with_bounds(t,q,qd,f,tau,boom_ang,boom_f,...
    q_min,q_max,u_min,u_max,vmax,mu,N,motor_R,motor_kt,...
    boom_ang_min,boom_ang_max)

lw = 2.5;

%% Plot Body Position and Velocity
figure()
subplot(2,1,1)
plot(t,q(1,:),'r-','Linewidth',lw)
hold on
plot(t,q(2,:),'b-','Linewidth',lw)
plot([t(1) t(end)],[q_min(1) q_min(1)],'r--')
plot([t(1) t(end)],[q_max(1) q_max(1)],'r--')
plot([t(1) t(end)],[q_min(2) q_min(2)],'b--')
plot([t(1) t(end)],[q_max(2) q_max(2)],'b--')
xlabel('Time (s)')
ylabel('Body Position (m)')
legend('x','y')

subplot(2,1,2)
plot(t,qd(1,:),'r-','Linewidth',lw)
hold on
plot(t,qd(2,:),'b-','Linewidth',lw)
plot([t(1) t(end)],[q_min(6) q_min(6)],'r--')
plot([t(1) t(end)],[q_max(6) q_max(6)],'r--')
plot([t(1) t(end)],[q_min(7) q_min(7)],'b--')
plot([t(1) t(end)],[q_max(7) q_max(7)],'b--')
xlabel('Time (s)')
ylabel('Body Velocity (m/s)')
legend('v_x','v_y')

%% Plot Joint Position and Velocity
figure()
subplot(2,1,1)
plot(t,q(3,:),'m-','Linewidth',lw)
hold on
plot(t,q(4,:),'g-','Linewidth',lw)
plot(t,q(5,:),'k-','Linewidth',lw)
plot([t(1) t(end)],[q_min(3) q_min(3)],'m--')
plot([t(1) t(end)],[q_max(3) q_max(3)],'m--')
plot([t(1) t(end)],[q_min(4) q_min(4)],'g--')
plot([t(1) t(end)],[q_max(4) q_max(4)],'g--')
plot([t(1) t(end)],[q_min(5) q_min(5)],'k--')
plot([t(1) t(end)],[q_max(5) q_max(5)],'k--')
xlabel('Time (s)')
ylabel('Joint Position (rad)')
legend('\theta_1','\theta_2','\theta_3')

subplot(2,1,2)
plot(t,qd(3,:),'m-','Linewidth',lw)
hold on
plot(t,qd(4,:),'g-','Linewidth',lw)
plot(t,qd(5,:),'k-','Linewidth',lw)
plot([t(1) t(end)],[q_min(8) q_min(8)],'m--')
plot([t(1) t(end)],[q_max(8) q_max(8)],'m--')
plot([t(1) t(end)],[q_min(9) q_min(9)],'g--')
plot([t(1) t(end)],[q_max(9) q_max(9)],'g--')
plot([t(1) t(end)],[q_min(10) q_min(10)],'k--')
plot([t(1) t(end)],[q_max(10) q_max(10)],'k--')
xlabel('Time (s)')
ylabel('Joint Velocity (rad/s)')
legend('d\theta_1/dt','d\theta_2/dt','d\theta_3/dt')

%% Plot Control Input Data
figure()
subplot(3,1,1) % Motor torque
plot(t,tau(1,:),'m-','Linewidth',lw)
hold on
plot(t,tau(2,:),'g-','Linewidth',lw)
plot(t,tau(3,:),'k-','Linewidth',lw)
plot([t(1) t(end)],[u_min(1) u_min(1)],'m--')
plot([t(1) t(end)],[u_max(1) u_max(1)],'m--')
plot([t(1) t(end)],[u_min(2) u_min(2)],'g--')
plot([t(1) t(end)],[u_max(2) u_max(2)],'g--')
plot([t(1) t(end)],[u_min(3) u_min(3)],'k--')
plot([t(1) t(end)],[u_max(3) u_max(3)],'k--')
xlabel('Time (s)')
ylabel('Motor Torque (Nm)')
legend('\tau_1','\tau_2','\tau_3')

subplot(3,1,2) % voltage
plot(t,(tau(1,:)/N)*motor_R/motor_kt + motor_kt*qd(3,:)*N,'m-','Linewidth',lw)
hold on
plot(t,(tau(2,:)/N)*motor_R/motor_kt + motor_kt*qd(4,:)*N,'g-','Linewidth',lw)
plot(t,(tau(3,:)/N)*motor_R/motor_kt + motor_kt*qd(5,:)*N,'k-','Linewidth',lw)
plot([t(1) t(end)], [vmax vmax],'k--')
plot([t(1) t(end)], -[vmax vmax],'k--')
xlabel('Time (s)')
ylabel('Voltage (V)')
legend('V_1','V_2','V_3')

subplot(3,1,3) % reaction force
plot(t,f(1,:),'r-','Linewidth',lw)
hold on
plot(t,f(2,:),'b-','Linewidth',lw)
plot(t,mu*f(2,:),'r--')
plot(t,-mu*f(2,:),'r--')
xlabel('Time (s)')
ylabel('Reaction Force (N)')
legend('F_x','F_y')

%% Boom
figure()
subplot(2,1,1) % boom angle
plot(t,boom_ang,'k-','Linewidth',lw)
hold on
plot([t(1) t(end)],[boom_ang_max boom_ang_max],'k--')
plot([t(1) t(end)],[boom_ang_min boom_ang_min],'k--')
xlabel('Time (s)')
ylabel('Boom Angle (rad)')

subplot(2,1,2) % boom force
plot(t,boom_f,'k-','Linewidth',lw)
xlabel('Time (s)')
ylabel('Boom Force (N)')



