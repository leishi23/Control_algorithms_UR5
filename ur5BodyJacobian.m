function J = ur5BodyJacobian(q)
% q=[0.2*pi 0.5*pi 0.3*pi 0.2*pi 0.1*pi 0.3*pi];
e1 = [1; 0; 0];
e2 = [0; 1; 0];
e3 = [0; 0; 1];
e0 = [0; 0; 0];

L0 = 0e-3;
L1 = 425e-3;
L2 = 392e-3;
L3 = 109.3e-3;
L4 = 94.75e-3;
L5 = 82.5e-3;

w=[e3 e1 e1 e1 e3 e1];
sq1=e0;
sq2=L0*e3;
sq3=(L0+L1)*e3;
sq4=(L0+L1+L2)*e3;
sq5=L3*e1;
sq6=(L0+L1+L2+L4)*e3;
sq=[sq1 sq2 sq3 sq4 sq5 sq6];
v=-cross(w,sq);
Xi=[v;w];
g0(1:3,1:3)=eye(3);
g0(1:3,4)=[L3+L5;0;L0+L1+L2+L4];
g0(4,:)=[0 0 0 1];

%To get expm(xi_hat*theta(i))
exp_Xi=zeros(4,4,6);
for i=1:6
    exp_Xi(:,:,i) = expTwist(Xi(:,i),q(i));
end

%Transfer to JB Xi
body_exp_Xi=zeros(6);
for i=1:6
    gst_pre=eye(4);
    for j=i:6
        gst_pre=gst_pre*exp_Xi(:,:,j);
    end
    gst=gst_pre*g0;
    body_exp_Xi(:,i)=inv(Adjoint(gst))*Xi(:,i);
end

J=body_exp_Xi;
end

function output=expTwist(Xi,theta)
v=Xi(1:3);
w=Xi(4:6);
w_hat=SKEW3(w);
R=expm(w_hat*theta);
p=(eye(3)-R)*cross(w,v)+w*w'*v*theta;
output=[R p;0 0 0 1];
end

function output=Adjoint(gst)
R=gst(1:3,1:3);
p=gst(1:3,4);
p_hat=SKEW3(p);
output=[R p_hat*R;zeros(3) R];
end
