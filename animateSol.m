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
    axis([-.2 0.7 -.01 .5]);

    %Step through and update animation
    for i = 1:5:length(tspan)
        % skip frame.
        if mod(i,1)
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

        set(h_title,'String',  sprintf('t=%.2f',t) ); % update title
        
        
        set(h_O_arm_base,'XData',[r_origin(1) r_arm_base(1)]);
        set(h_O_arm_base,'YData',[r_origin(2) r_arm_base(2)]);
        
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

        pause(.0005)
    end
end