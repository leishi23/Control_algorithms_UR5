function [joint_pos,error] = invKinMove(start, target, ur5, joint_pos, dx) 

R_start_offset = start(1:3,1:3)*ROTZ(-pi/2)*ROTX(-pi/2);
R_target_offset = target(1:3,1:3)*ROTZ(-pi/2)*ROTX(-pi/2);

init = [start(1:3,4); EULERXYZINV(R_start_offset)];
goal = [target(1:3,4); EULERXYZINV(R_target_offset(1:3,1:3))];
steps = ceil(norm(goal(1:3)-init(1:3))/dx);

%move to target by "steps" times
for i = 1:steps
    pos_i = init + (goal-init)*min(i/steps, 1);
    g_i = get_g(pos_i(1:3), pos_i(4:6));
    joints_i = ur5InvKin(g_i);
    
    %choose the nearest joints trajectory
    [~,index] = min(vecnorm(joints_i-joint_pos));
    joints_i = joints_i(:, index);
end

%Moving joints
ur5.move_joints(joints_i, 5);
pause(5)

g = ur5FwdKin(joints_i- [-pi/2;-pi/2;0;-pi/2;0;0]); % The actual ee position given by joint_pos
dso3 = sqrt(trace((g(1:3,1:3)-target(1:3,1:3)*(g(1:3,1:3)-target(1:3,1:3))')));
dR3 = norm(g(1:3,4)-target(1:3,4));
error = [dR3, dso3]; % The errors
end