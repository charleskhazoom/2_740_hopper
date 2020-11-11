function output = jumpOptEndpoint(input)

% Cost Function - max body height with minor torque minimization
output.objective = -1*input.phase(2).finalstate(end,1) +...
                   -2*input.phase(2).finalstate(end,2);

% Continuity Constraints
tf = input.phase(1).finaltime;
xf = input.phase(1).finalstate;
t0 = input.phase(2).initialtime;
x0 = input.phase(2).initialstate;
output.eventgroup(1).event = [xf-x0, tf-t0];