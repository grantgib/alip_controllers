function [Xdot] = ode_lipm(t,X,info)
%% Extract Inputs
g = info.g;
m = info.m;
z_H = info.z_H;
kx = info.kx;
ky = info.ky;

xc = X(1);
yc = X(2);
Lx = X(3);
Ly = X(4);
Lz = X(5);

%% state derivative
Xdot = [...
    (1/(m*z_H))*Ly + (ky/(m*z_H))*Lz; 
    -Lx/(m*z_H) - (kx/(m*z_H))*Lz;
    -m*g*yc
    m*g*xc;
    0];
end

