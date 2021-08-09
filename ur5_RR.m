% Resolved-rate control using differential kinematics
function ur5_RR(Q_home_ctrl, g_pick, g_start, g_place, g_target)
rosshutdown
ur5 = ur5_interface();

tic
% Move to home
ur5.move_joints(Q_home_ctrl,3);
pause(3)
% Go above start
ur5RRcontrol(g_start , 1, ur5);
pause(2)
% Go start
ur5RRcontrol(g_pick, 3, ur5);
pause(2)
% Go up again
ur5RRcontrol(g_start, 1, ur5);
pause(2)
% Go above target
ur5RRcontrol(g_target, 1, ur5);
pause(2)
% Go target
ur5RRcontrol(g_place, 1, ur5);
pause(2)
% Go up again
ur5RRcontrol(g_target, 1, ur5);
pause(2)
% Go back to home
% ur5RRcontrol(g_home_ctrl, 1, ur5);
% pause(5)
ur5.move_joints(Q_home_ctrl, 5);
pause(5)
toc
end
