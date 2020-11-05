function output = jumpOptContinuous(input)

% Get Aux Data

% Dynamics

%% Phase 1 - Stance
% Dynamics
dz1 = 0*input.phase(1).state;
for i = 1:length(input.phase(1).state(:,1))
    A = A_stance(input.phase(1).state(i,:)', input.auxdata.p);
    b = b_stance(input.phase(1).state(i,:)', input.phase(1).control(i,:)', input.auxdata.p);
    x_augmented = A\(b);
    qdd = x_augmented(1:5);
    
    dz1(i,1:5) = input.phase(1).state(i,1:5);
    dz1(i,6:10) = qdd';
    
    % Need to add reaction force constraints
    F(i,1:2) = x_augmented(6:7)'; % constraint force

end
output(1).dynamics = dz1; % dynamics equality constraint

% inequality path constrain on force ratios (friction)
output(1).path(:,1) = F(:,1)./F(:,2); % abs(horiz_force/vert_force relates)< mu. % force constraint  

% inequality path constraint on force
output(1).path(:,2) = F(:,2); % second path constraint is :vertical force can only be upwards

% voltage ineq constraint on 3 motors
kt = input.auxdata.p(27); 
R = input.auxdata.p(28);
N  =input.auxdata.p(13);

output(1).path(:,3) = (input.phase(1).control(:,1)/N)*R/kt + kt*input.phase(1).state(:,8)*N; % voltage = (R/kt*motor_torque + kt*motor_speed)
output(1).path(:,4) = (input.phase(1).control(:,2)/N)*R/kt + kt*input.phase(1).state(:,9)*N; % voltage = (R/kt*motor_torque + kt*motor_speed)
output(1).path(:,5) = (input.phase(1).control(:,3)/N)*R/kt + kt*input.phase(1).state(:,10)*N;% voltage = (R/kt*motor_torque + kt*motor_speed)

%% Cost Function
output(1).integrand = sum(input.phase(1).control(:,:).*input.phase(1).control(:,:),2); % min effort