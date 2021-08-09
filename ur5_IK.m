% Inverse kinematics
function errors = ur5_IK(g_home_ctrl, Q_home_ctrl,g_pick, g_start, g_place, g_target)
rosshutdown
ur5 = ur5_interface();
dx = 0.015;
errors = zeros(6,2);

tic
% % Move to default home
% ur5.move_joints(ur5.home, 10);
% pause(15);
% Move to home
ur5.move_joints(Q_home_ctrl, 5);
pause(5);
joint_pos = ur5.get_current_joints();
pause(2);
% Go above start
[joint_pos,err] = invKinMove(g_home_ctrl, g_start, ur5, joint_pos, dx);
errors(1,:)=err;
pause(5);
% Go start
[joint_pos,err] = invKinMove(g_start, g_pick, ur5, joint_pos, dx);
errors(2,:)=err;
pause(5);
% Go up again
[joint_pos,err] = invKinMove(g_pick, g_start, ur5, joint_pos, dx);
errors(3,:)=err;
pause(5);
% Go above target
[joint_pos,err] = invKinMove(g_start, g_target, ur5, joint_pos, dx);
errors(4,:)=err;
pause(5);
% Go target
[joint_pos,err] = invKinMove(g_target, g_place, ur5, joint_pos, dx);
errors(5,:)=err;
pause(5);
% Go up again
[joint_pos,err] = invKinMove(g_place, g_target, ur5, joint_pos, dx);
errors(6,:)=err;
pause(5);
% Go back to home
ur5.move_joints(Q_home_ctrl, 5);
pause(5)
toc
end