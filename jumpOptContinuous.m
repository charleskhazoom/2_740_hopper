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
    %F = x_augmented(6:7); % constraint force
    
    dz1(i,1:5) = input.phase(1).state(i,1:5);
    dz1(i,6:10) = qdd';
    
    % Need to add reaction force constraints
end
output(1).dynamics = dz1;

%% Cost Function
output(1).integrand = sum(input.phase(1).control(:,:).*input.phase(1).control(:,:),2); % min effort