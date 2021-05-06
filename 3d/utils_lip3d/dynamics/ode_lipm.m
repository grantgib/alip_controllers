function [Xdot] = ode_lipm(t,X,info)
%% Extract Inputs
g = info.sym_info.g;
m = info.sym_info.m;
z_H = info.gait_info.z_H;

xc = X(1);
yc = X(2);
Lx = X(3);
Ly = X(4);

%% state derivative
Xdot = [...
    Ly/(m*z_H); 
    -Lx/(m*z_H);
    -m*g*yc
    m*g*xc];

end

