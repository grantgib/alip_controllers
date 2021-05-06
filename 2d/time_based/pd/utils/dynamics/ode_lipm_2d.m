function [Xdot] = ode_lipm_2d(t,X,params)
%% Extract Inputs
x = X(1);
xdot = X(2);
g = params.g;
zc = params.z_const;

%% Compute acceleration
xddot = (g/zc)*x;

%% Output state vector time derivative
Xdot = [xdot;
        xddot];

end

