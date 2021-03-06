function fignb = plot_solution(tout,z_out,u_out,tLO,p,fignb)

%% Energy
E = energy_floating(z_out,p);
figure(fignb); clf
fignb=fignb+1;
plot(tout,E);xlabel('Time (s)'); ylabel('Energy (J)');


%% Ground force
Fgnd = zeros(2,length(tout));
for i = 1:length(z_out)
    A = A_stance(z_out(:,i),p);
    
    if tout(i) <= tLO
        b = b_stance(z_out(:,i),u_out(:,i),p);
        x_augmented = A\(b);
        Fgnd(:,i) = x_augmented(6:7); % vertical constraint force
    else
        Fgnd(:,i)=[0;0];
    end
    
end
figure(fignb);
fignb=fignb+1;
plot(tout,Fgnd)
ylabel('Ground Force')
xlabel('Time (s)');
legend('x','y')

%% force ratios
Force_ratio = Fgnd(1,:)./Fgnd(2,:);

figure(fignb);
fignb=fignb+1;
plot(tout,Force_ratio)
ylabel('Ground Force Ratios')
xlabel('Time (s)');
% legend('x','y')
%% Control Input
figure(fignb);
fignb=fignb+1;
plot(tout,u_out)
ylabel('Input Torque (N)')
xlabel('Time (s)');
legend('\tau_1','\tau_2','\tau_3')


% %% Foot position over time
% rE = zeros(2,length(tout));
% vE = zeros(2,length(tout));
% for i = 1:length(tout)
%     rE(:,i) = position_foot(z_out(:,i),p);
%     vE(:,i) = velocity_foot(z_out(:,i),p);
% end
% 
% figure(fignb);
% fignb=fignb+1;
% clf;
% plot(tout,rE(1,:),'r','LineWidth',2)
% hold on
% plot(tout,rE(2,:),'b','LineWidth',2)
% title('Foot position')
% 
% xlabel('Time (s)'); ylabel('Position (m)'); legend({'x','y'});
% 
% figure(fignb);
% fignb=fignb+1;
% clf;
% plot(tout,vE(1,:),'r','LineWidth',2)
% hold on
% plot(tout,vE(2,:),'b','LineWidth',2)
% title('Foot Velocity')
% 
% xlabel('Time (s)'); ylabel('Velocity (m)'); legend({'vel_x','vel_y'});


%% Body Position, Joint Angles
figure(fignb);
fignb=fignb+1;
ax1(1) = subplot(211);
plot(tout,z_out(1:2,:))
legend('x','y');
xlabel('Time (s)');
ylabel('pos x,y (m)');

ax1(2) = subplot(212);
plot(tout,z_out(3:5,:)*180/pi)
legend('q1','q2','q3');
xlabel('Time (s)');
ylabel('Angle (deg)');

%% Body Velocity, Joint Velocity
figure(fignb);
fignb=fignb+1;

ax1(3) = subplot(211);
plot(tout,z_out(6:7,:))
legend('xdot','ydot');
xlabel('Time (s)');
ylabel('base velocity (m/sec)');

ax1(4) = subplot(212);
plot(tout,z_out(8:10,:)*180/pi)
legend('q1_dot','q2_dot','q3_dot');
xlabel('Time (s)');
ylabel('Ang.Vel (deg/s)');

%% Com position + velocity
for i = 1:length(tout)
    rCOM(:,i) = com_pos(z_out(:,i),p);
    drCOM(:,i) = com_vel(z_out(:,i),p);
end

figure(fignb);
fignb=fignb+1;

plot(tout,rCOM(1,:));
hold on;
plot(tout,rCOM(2,:));
hold off;
legend('x','y')
title('COM position')

figure(fignb);
fignb=fignb+1;
plot(tout,drCOM(1,:));
hold on;
plot(tout,drCOM(2,:));
hold off;
legend('x','y')
title('COM velocity')

%% Voltage
figure(fignb);
fignb=fignb+1;

kt = p(33); 
R  = p(34);
N  = p(13);
V_out(1,:) = (u_out(1,:)/N)*R/kt + kt*z_out(8,:)*N;
V_out(2,:) = (u_out(2,:)/N)*R/kt + kt*z_out(9,:)*N;
V_out(3,:) = (u_out(3,:)/N)*R/kt + kt*z_out(10,:)*N;

plot(tout,V_out(1,:));
hold on;
plot(tout,V_out(2,:));
plot(tout,V_out(3,:));
hold off;
legend('V_1','V_2','V_3')
title('Voltage')


