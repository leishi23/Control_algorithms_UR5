function g0 = ur5FwdKin(q)
e1 = [1; 0; 0];
e2 = [0; 1; 0];
e3 = [0; 0; 1];
e0 = [0; 0; 0];
I = eye(3);

L0 = 89.2e-3;
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

g0(1:3,1:3)=eye(3);
g0(1:3,4)=[L3+L5;0;L0+L1+L2+L4];
g0(4,:)=[0 0 0 1];

for i = 6:-1:1
   wi = w(:,i);
   vi = v(:,i);
   qi = q(i);
   wi_hat = SKEW3(wi);
   
   expi= [expm(wi_hat*qi), (I-expm(wi_hat*qi))*(cross(wi,vi));0,0,0,1];
   g0 = expi*g0;
   
end
end
