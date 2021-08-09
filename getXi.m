function xi = getXi(g)
    rot = g(1:3,1:3);
    w_hat = logm(rot);
    w(1) = -w_hat(2,3);
    w(2) = w_hat(1,3);
    w(3) = -w_hat(1,2);
    sum=zeros(3);
    I=eye(3);
    a=1;
    for i = 1:100
        a = a*i;
        w_hat_part = w_hat^(i-1);
        sum = sum+w_hat_part/a;
    end
    p_part_parameter = sum;
    p_part = g(1:3,4);
    v = p_part_parameter \ p_part;
    xi(1:3,1) = v';
    xi(4:6,1) = w'';
end
