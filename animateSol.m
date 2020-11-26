function animateSol(tspan, x,p)
    % Prepare plot handles
    hold on
    h_OB = plot([0],[0],'LineWidth',2);
    h_AC = plot([0],[0],'LineWidth',2);
    h_BD = plot([0],[0],'LineWidth',2);
    h_CE = plot([0],[0],'LineWidth',2);
    h_O_arm_base = plot([0],[0],'LineWidth',2);
    h_arm_base_tip = plot([0],[0],'LineWidth',2);
    
    xlabel('x'); ylabel('y');
    h_title = title('t=0.0s');
    
    axis equal
    axis([-.2 0.7 -.01 .25]);
     hold on
     seg1x = [];
     seg1y = [];
     seg2x = [];
     seg2y = [];

     seg3x = [];
     seg3y = [];
     
     seg4x = [];
     seg4y = [];
     seg5x = [];
     seg5y = [];
     
     seg6x = [];
     seg6y = [];
     hip_traj = [];
     col = [0.3 0.3 0.3];
    %Step through and update animation
    for i = 1:1:length(tspan)
        % skip frame.
        if mod(i,50)
            continue;
        end
        t = tspan(i);
        z = x(:,i); 
        keypoints = keypoints_floating(z,p); %keypoints = [rA(1:2) rB(1:2) rC(1:2) rD(1:2) rE(1:2) r_origin(1:2) r_arm_base(1:2) r_arm_tip(1:2)];


        rA = keypoints(:,1); % Vector to base of cart
        rB = keypoints(:,2);
        rC = keypoints(:,3); % Vector to tip of pendulum
        rD = keypoints(:,4);
        rE = keypoints(:,5);
        r_origin = keypoints(:,6);
        r_arm_base =  keypoints(:,7);
        r_arm_tip  =  keypoints(:,8);
        
        hip_traj = [hip_traj r_origin];
        
        set(h_title,'String',  sprintf('t=%.2f',t) ); % update title
        

        set(h_O_arm_base,'XData',[r_origin(1) r_arm_base(1)]);
        set(h_O_arm_base,'YData',[r_origin(2) r_arm_base(2)]);
        
        for j = 1:length(seg1x)/2
            plot(seg1x(2*j-1:2*j),seg1y(2*j-1:2*j),'color',col);hold on
            plot(seg2x(2*j-1:2*j),seg2y(2*j-1:2*j),'color',col);hold on
            plot(seg3x(2*j-1:2*j),seg3y(2*j-1:2*j),'color',col);hold on
            plot(seg4x(2*j-1:2*j),seg4y(2*j-1:2*j),'color',col);hold on
            plot(seg5x(2*j-1:2*j),seg5y(2*j-1:2*j),'color',col);hold on
            plot(seg6x(2*j-1:2*j),seg6y(2*j-1:2*j),'color',col);hold on

        end
        
        set(h_arm_base_tip,'XData',[r_arm_base(1) r_arm_tip(1)]);
        set(h_arm_base_tip,'YData',[r_arm_base(2) r_arm_tip(2)]);
        
        set(h_OB,'XData',[r_origin(1) rB(1)]);
        set(h_OB,'YData',[r_origin(2) rB(2)]);
        
        set(h_AC,'XData',[rA(1) rC(1)]);
        set(h_AC,'YData',[rA(2) rC(2)]);
        
        set(h_BD,'XData',[rB(1) rD(1)]);
        set(h_BD,'YData',[rB(2) rD(2)]);
        
        set(h_CE,'XData',[rC(1) rE(1)]);
        set(h_CE,'YData',[rC(2) rE(2)]);
        
        if mod(i,1500)==0 %keep some frames
            seg1x = [seg1x h_O_arm_base.XData];
            seg1y = [seg1y h_O_arm_base.YData];
            
            seg2x = [seg2x h_arm_base_tip.XData];
            seg2y = [seg2y h_arm_base_tip.YData];
            
            seg3x = [seg3x h_OB.XData];
            seg3y = [seg3y h_OB.YData];
            
            seg4x = [seg4x h_AC.XData];
            seg4y = [seg4y h_AC.YData];
            
            seg5x = [seg5x h_BD.XData];
            seg5y = [seg5y h_BD.YData];
            
            seg6x = [seg6x h_CE.XData];
            seg6y = [seg6y h_CE.YData];            
        end
        
        plot(hip_traj(1,:),hip_traj(2,:),'--k','linewidth',1.4);
        
        pause(.005)
    end
end