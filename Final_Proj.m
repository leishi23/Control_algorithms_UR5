clc
clear
%%%%%%%%%%%%%%%%%%%%%%%%%%

%    PLEASE INPUT DESIRED START POSITION   %

%%%%  start and target are provided in class handout %%
g_start = [0 -1 0 0.47; 0 0 1 0.55; -1 0 0 0.12; 0 0 0 1];
g_target = [0 -1 0 -0.3; 0 0 1 0.39;-1 0 0 0.12; 0 0 0 1];

%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initiate ur5
rosshutdown;
ur5                      = ur5_interface();
% Start and target gst
g_pick                  = g_start;
g_start(3,4)           = 0.12;
g_pick(3,4)            = 0.01;
g_place                 = g_target;
g_place(3,4)          = 0.01;
g_target(3,4)         = 0.12;

% Adjust offset between frame T and fram tool0
R_start_offset = g_start(1:3,1:3)*ROTZ(-pi/2)*ROTX(-pi/2);
R_target_offset = g_target(1:3,1:3)*ROTZ(-pi/2)*ROTX(-pi/2);

% Different home for control methods
temp1               = (EULERXYZINV(R_start_offset)+EULERXYZINV(R_target_offset))/2;
temp2               = (g_start(1:3,4)+g_target(1:3,4))/2 + [0 0 0.4]';
g_home_ctrl      = [EULERXYZ(temp1), temp2; [0 0 0 1]];
Q_home             = ur5InvKin(g_home_ctrl);
[~,index] = min(vecnorm(Q_home -ur5.home()));
Q_home_ctrl       = Q_home(:, index);

% Establish start_frame & target_frame
start_frame = tf_frame('base_link','start_frame',g_start);
pause(2);
target_frame = tf_frame('base_link','target_frame',g_target);
pause(2);

% Checking Input Validity
if ~Collision(g_start) || ~Collision(g_target) || ~Collision(g_home_ctrl)
    cont ='Invalid input';
    error(cont);
end
%% 1) Inverse kinematics
ur5_IK(g_home_ctrl, Q_home_ctrl, g_pick, g_start, g_place, g_target)

%% 2) Resolved-rate control using differential kinematics
ur5_RR(Q_home_ctrl, g_pick, g_start, g_place, g_target)

%% 3) Transpose-Jacobian control
ur5_T(Q_home_ctrl, g_pick, g_start, g_place, g_target)

