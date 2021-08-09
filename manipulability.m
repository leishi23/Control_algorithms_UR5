function mu=manipulability(J,measure)
% J= [    -0.5827   -0.6621   -0.6621   -0.3701    1.1940         0
%    -0.2558   -1.5374   -2.0255   -1.8487    0.0715   -1.2224
%     0.3521    1.4020    1.0473    0.0648   -0.0984   -0.3972
%     0.0000    0.9511    0.9511    0.9511    0.0000    1.0000
%    -0.8090   -0.1816   -0.1816   -0.1816    0.8090         0
%    -0.5878    0.2500    0.2500    0.2500    0.5878         0
% ]
% measure='sigmamin';
A=J*J';
[V,D]=eig(A);
D=diag(D);
sigma_max=max(D)^0.5;
sigma_min=min(D)^0.5;
switch measure
    case 'sigmamin'
        mu=sigma_min;
    case 'detjac'
        mu=det(J);
    case 'invcond'
        mu=sigma_min/sigma_max;
end
    