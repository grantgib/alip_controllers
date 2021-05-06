function [Xdot] = ode_lipm(t,X,info)
%% Extract Inputs
g = info.sym_info.g;
m = info.sym_info.m;
z_H = info.gait_info.z_H;

xc = X(1);
Lst = X(2);

%% state derivative
Xdot = [...
    Lst/(m*z_H); 
    m*g*(xc)];

end

