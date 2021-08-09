function res = EULERXYZINV(R)
theta2 = atan2(R(1,3), sqrt(R(1,1)^2 + R(1,2)^2));
if theta2 < 0
    theta2 = theta2 + pi;
    theta3 = -atan2(-R(1,2)/cos(theta2), R(1,1)/cos(theta2));
    theta1 = -atan2(-R(2,3)/cos(theta2), R(3,3)/cos(theta2));
else
    theta3 = atan2(-R(1,2)/cos(theta2), R(1,1)/cos(theta2));
    theta1 = atan2(-R(2,3)/cos(theta2), R(3,3)/cos(theta2));
end
res = [theta1, theta2, theta3]';
end