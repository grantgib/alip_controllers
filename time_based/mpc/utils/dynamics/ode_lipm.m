function [Xdot] = ode_lipm(t,X,info)
%% Extract Inputs
x = X(1);
xdot = X(2);
g = info.sym_info.g;
zc = info.gait_info.z_const;

%% Compute acceleration
xddot = (g/zc)*x;

%% Output state vector time derivative
Xdot = [xdot;
        xddot];

end

