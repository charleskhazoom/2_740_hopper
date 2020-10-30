function zero = solve_init_pose(leg_angles,desired_hip_pos,p)
% desired_hip_pos is a column vector on x,y desired coordinates
% solve for leg angles which put hip at desired position

z = [desired_hip_pos;leg_angles]; 
rE = position_foot(z,p); % rE is what we want to be zero

zero = rE;