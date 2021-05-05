function [Xdot] = ode_lipm(t,X,info)
%% Extract Inputs
g = info.sym_info.g;
m = info.sym_info.m;
z_H = info.gait_info.z_H;

xc = X(1);
L_st = X(2);

%% state derivative
Xdot = [L_st/(m*z_H); % zc_dot would need to be estimated
        m*g*(xc)];

end

