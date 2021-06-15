function [info] = formulate_mpc(info)
import casadi.*

%% Extract Inputs
% sym_info
g = info.sym_info.g;
m = info.sym_info.m;
n_x = info.sym_info.n_x;
k_slope = info.sym_info.k_slope;

% ctrl_info
dt = info.ctrl_info.mpc.dt;         % time interval
t_step = info.gait_info.t_step;     % step period
N_steps = info.ctrl_info.mpc.N_steps;
Q = info.ctrl_info.mpc.Q;
sol_type = info.ctrl_info.mpc.sol_type;

% gait_info
z_H = info.gait_info.z_H;

%% Dynamics
% Declare System Variables
xc = SX.sym('xc');  % relative position of center of mass w.r.t stance contact point
Lst = SX.sym('Lst');          % Angular momentum about contact point (stance foot)
x = [xc; Lst];

xdot = [...
    (Lst)/(m*z_H); % zc_dot would need to be estimated
    m*g*(xc)];     % lip model

% Continuous time dynamics
fc = Function('fc',{x},{xdot});   % f = dx/dt

% Discrete time dynamics
rk_explicit = false;
if rk_explicit
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
else
    opts_intg = struct(...
        'tf',                           dt,...
        'simplify',                     true,...
        'number_of_finite_elements',    4);
    dae = struct(...
        'x',    x,...
        'ode',  xdot);
    intg = integrator('intg','rk',dae,opts_intg);
    res = intg('x0',x);
    x_next = res.xf;
    fd = Function('fd',{x},{x_next});
end

%% Formulate Optimization Problem
% Intermediate optimization variables
N_fp = N_steps;
k_step = t_step / dt;
N_k = N_steps * k_step;

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
P_m = MX.sym('m');
P_zc = MX.sym('zc');

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
            xdot_pseudo = Xk(2)/(P_m*P_zc);
            opt_cost(n-1) = {(xdot_pseudo-P_xdot_des)' * Q(n) * (xdot_pseudo-P_xdot_des)};
        end
        Xk_end = fd(Xk);
        Xk_end = [Xk_end(1)-Ufp_traj{n}; Xk_end(2)];     % update init position to reflect foot placement
        Xk = X_traj{k+1};
        n = n + 1;      % increase step counter
    else
        Xk_end = fd(Xk);
        Xk = X_traj{k+1};
    end
    x_constr = [x_constr, {Xk_end-Xk}];
end
xdot_pseudo = Xk(2)/(P_m*P_zc);
opt_cost(n-1) = {(xdot_pseudo-P_xdot_des)' * Q(n) * (xdot_pseudo-P_xdot_des)}; % add cost for velocity at end of prediction horizon

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
opt_params = [P_xdot_des; P_m; P_zc];

%% Create an OPT solver
prob = struct(...
    'f',    opt_cost,...
    'x',    opt_var,...
    'g',    opt_constr,...
    'p',    opt_params);

if sol_type == "ipopt"
    opts = struct();
    opts.ipopt.print_level = 0;
%     opts.ipopt.print_time = 'false';
    % options.ipopt.print_timing_statistics = 'yes';
    opts.ipopt.linear_solver = 'ma57';
    % options.ipopt.acceptable_tol = 1e-8;
    % options.ipopt.acceptable_obj_change_tol = 1e-8;
    % options.ipopt.acceptable_constr_viol_tol = 1e-8;
    % options.ipopt.hessian_constant = 'yes';
    % options.ipopt.jac_d_constant = 'yes';  % Indicates whether all inequality constraints are linear
    % options.ipopt.max_iter = 500;
    % options.ipopt.max_cpu_time = 0.25;
    solver = nlpsol('solver','ipopt',prob,opts);
elseif sol_type == "qrqp"
    % C++ Code Generation
    opts = struct(...
        'qpsol',            'qrqp',...
        'print_header',     false,...
        'print_iteration',  false,...
        'print_time',       false);    % osqp, qrqp (not as robust joris says on google groups)
    opts.qpsol_options = struct(...
        'print_iter',       false,...
        'print_header',     false,...
        'print_info',       false);
    solver = nlpsol('solver','sqpmethod',prob,opts);
    
    gen_opts = struct(...
        'mex',          true,...
        'cpp',          true,...
        'main',         true,...
        'with_header',  false);
    solver.generate('mpctest.c',gen_opts)
elseif sol_type == "qpoases"
    solver = qpsol('solver','qpoases',prob);
%     gen_opts = struct(...
%         'mex',  true,...
%         'cpp',  true,...
%         'main', true);
%     solver.generate('mpc_qp_test.cpp',gen_opts);
%     mex mpc_qp_test.cpp -largeArrayDims
end

%% Return symbolics and solver
info.sym_info.fd = fd;
info.ctrl_info.mpc.solver = solver;



