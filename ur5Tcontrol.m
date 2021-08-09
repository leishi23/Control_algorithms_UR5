function finalerr = ur5Tcontrol( gdesired, K, ur5 )
%ur5Tcontrol takes gdesired, K, and ur5 object produces final error

% Step size
Tstep = 0.1;

% qk needs to be adjusted because the orientation of xyz coordinate
% used in ur5 and ur5FwdKin is different

% extract current joint configuration, qk in ROS
qk_real = ur5.get_current_joints;

home = [-pi/2;-pi/2;0;-pi/2;0;0];

% create q0
qk = qk_real - home;

% create gst0
gst = ur5FwdKin(qk);

xi_k = getXi(gdesired\gst);
v_err = norm(xi_k(1:3));
w_err = norm(xi_k(4:6));

while v_err > 1e-3 || w_err > pi*1e-3
    J = ur5BodyJacobian(qk);
% check singularity
if abs(manipulability(J,'sigmamin')) < 1e-6
        finalerr = -1;                                   % if singular, abort and return -1
        return
end
    dq = - K * Tstep * J' * xi_k;
    qk = qk + dq;    
    gst = ur5FwdKin(qk);
    
    % update configuration
    qk_real = qk + home;
    
    % Avoid unnecessary rotation
    qk_real = rem(qk_real,2*pi);
    
    % Restrict motion between [-pi,pi]
    for i=1:length(qk_real)
        if qk_real(i) > pi
            qk_real(i) = qk_real(i) - 2*pi;
        elseif qk_real(i) < -pi
            qk_real(i) = qk_real(i) + 2*pi;
        end
    end
   
% calculate current error
    xi_k = getXi(gdesired\gst);
    v_err=norm(xi_k(1:3));
    w_err=norm(xi_k(4:6));

end
dq = qk_real - ur5.get_current_joints();
tn = max(abs(dq/ur5.speed_limit));
ur5.move_joints(qk_real,tn);
pause(tn+0.5)
gst_final = ur5FwdKin(qk);
Rd = gdesired(1:3,1:3);
R   = gst_final(1:3,1:3);
finalerr(1,1) = trace(sqrt((Rd-R)*(Rd-R)'));
finalerr(2,1) = norm(gdesired(1:3,4)-gst_final(1:3,4));
disp(['Final error of R = ',num2str(finalerr(1,1))])
disp(['Final error of d = ',num2str(finalerr(2,1))])
end