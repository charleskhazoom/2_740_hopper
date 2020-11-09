function output = jumpOptEndpoint(input)
output.objective = -input.phase(1).finalstate(end,2); % max body height