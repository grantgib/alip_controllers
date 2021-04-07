function [info] = formulate_mpc(info)
import casadi.*

%% Extract Inputs
% sym_info
g = info.sym_info.g;
n_x = info.sym_info.n_x;

% gait_info
zc = info.gait_info.z_const;

% ctrl_info
dt = info.ctrl_info.mpc.dt;         % time interval
t_step = info.gait_info.t_step;     % step period
k_step = info.ctrl_info.mpc.k_step;
N_k = info.ctrl_info.mpc.N_k;
N_steps = info.ctrl_info.mpc.N_steps;
N_fp = info.ctrl_info.mpc.N_fp;
Q = info.ctrl_info.mpc.Q;

%% Dynamics
% Declare System Variables
x_com_rel = SX.sym('x_com_rel');  % relative position of center of mass
xdot_com = SX.sym('xdot_com');
x = [x_com_rel; xdot_com];
xdot = [xdot_com; (g/zc)*(x_com_rel)];     % lip model

% Continuous time dynamics
fc = Function('fc',{x},{xdot});   % f = dx/dt

% Discrete time dynamics
M = 4;
dt_rk = dt / M;
X0 = MX.sym('X0',2);
X = X0;
for j = 1:M
    k1 = fc(X);
    k2 = fc(X + (dt_rk/2)*k1);
    k3 = fc(X + (dt_rk/2)*k2);
    k4 = fc(X + dt_rk*k3);
    X = X + (dt_rk/6) * (k1 + 2*k2 + 2*k3 + k4);
end
fd = Function('fd',{X0},{X});

%% Formulate Optimization Problem
% Initialize variables
n = 1;      % foot step iteration

% opt vars
X_traj = cell(1,N_k);
for i = 1:N_k
    X_traj_i = MX.sym(['X_' num2str(i)],2);
    X_traj(i) = {X_traj_i};
end
Ufp_traj = cell(1,N_fp);
for i = 1:N_fp
    Ufp_traj_i = MX.sym(['Ufp_' num2str(i)]);
    Ufp_traj(i) = {Ufp_traj_i};
end

% constraints
x_constr = {};
delta_fp_constr = {};

% parameters
P_xdot_des = MX.sym('P_xdot_des');

% cost
opt_cost = cell(1,N_fp);

% initial conditions
Xk = X_traj{1};

% Loop through discrete trajectory
for k = 1:N_k-1
    k_init = (n-1)*k_step + 1;
    k_end = k_init + k_step-1;
    if (k == k_init)
        % init state of n-th step
        Xk_end = fd(Xk);
        Xk = X_traj{k+1};
    elseif (k == k_end)
        % add cost
        if n > 1 
            opt_cost(n-1) = {(Xk(2) - P_xdot_des)' * Q(n) * (Xk(2) - P_xdot_des)};
        end
        Xk_end = fd(Xk);
        Xk_end = [-Ufp_traj{n}; Xk_end(2)];     % update init position to reflect foot placement
        Xk = X_traj{k+1};
        n = n + 1;      % increase step counter
    else
        Xk_end = fd(Xk);
        Xk = X_traj{k+1};
    end
    x_constr = [x_constr, {Xk_end-Xk}];
end
opt_cost(n-1) = {(Xk(2) - P_xdot_des)' * Q(n) * (Xk(2) - P_xdot_des)};  % add cost for velocity at end of prediction horizon

% Rate limiter (only consider when there is more than one fp)
if N_fp > 1
    for n = 1:N_fp-1
        Ufp = Ufp_traj{n};
        Ufp_next = Ufp_traj{n+1};
        delta_fp_constr = [delta_fp_constr, {Ufp_next - Ufp}];   
    end
end

% Combine cost, vars, constraints, parameters
opt_cost = sum(vertcat(opt_cost{:}));
opt_var = [vertcat(X_traj{:}); vertcat(Ufp_traj{:})];
opt_constr = [vertcat(x_constr{:}); vertcat(delta_fp_constr{:})];
opt_params = P_xdot_des;

%% Create an OPT solver
prob = struct(...
    'f',    opt_cost,...
    'x',    opt_var,...
    'g',    opt_constr,...
    'p',    opt_params);
options = struct();
% options.ipopt.opt.print_level = 0;
options.ipopt.print_timing_statistics = 'yes';
options.ipopt.linear_solver = 'ma57';
% options.ipopt.acceptable_tol = 1e-8;
% options.ipopt.acceptable_obj_change_tol = 1e-8;
% options.ipopt.acceptable_constr_viol_tol = 1e-8;
% options.ipopt.hessian_constant = 'yes';
% options.ipopt.jac_d_constant = 'yes';  % Indicates whether all inequality constraints are linear
% options.ipopt.max_iter = 500;
% options.ipopt.max_cpu_time = 0.25;

solver = nlpsol('solver','ipopt',prob,options);

%% Return symbolics and solver
info.sym_info.fd = fd;
info.ctrl_info.mpc.solver = solver;







