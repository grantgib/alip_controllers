function [info] = formulate_mpc_opti(info)
import casadi.*

%% Extract Inputs
% sym_info
g = info.sym_info.g;
m = info.sym_info.m;
n_x = info.sym_info.n_x;
n_ufp = info.sym_info.n_ufp;

% gait_info
t_step = info.gait_info.t_step;     % step period

% ctrl_info
dt = info.ctrl_info.mpc.dt;         % time interval
N_steps = info.ctrl_info.mpc.N_steps;
Q = info.ctrl_info.mpc.Q;
sol_type = info.ctrl_info.mpc.sol_type;


%% Dynamics
% Declare System Variables: constraint zc = kx*xc + ky*yc + z_H
xc = SX.sym('xc');  % relative position of center of mass w.r.t stance contact point
yc = SX.sym('yc');
Lx = SX.sym('Lx');
Ly = SX.sym('Ly');          % Angular momentum about contact point (stance foot)
Lz = SX.sym('Lz');
x = [xc; yc; Lx; Ly; Lz];

% params
z_H_sym = SX.sym('z_H_sym');
kx_sym = SX.sym('kx_sym');
ky_sym = SX.sym('ky_sym');
k_sym = [kx_sym;ky_sym];

xcdot = (Ly + ky_sym*Lz)/(m*z_H_sym);
ycdot = (-Lx - kx_sym*Lz)/(m*z_H_sym);
Lxdot = -m*g*yc;
Lydot = m*g*xc;     % lip model
Lzdot = 0;
xdot = [xcdot; ycdot; Lxdot; Lydot; Lzdot];

% Continuous time dynamics
fc = Function('fc',{x,k_sym,z_H_sym},{xdot});   % f = dx/dt

% Discrete time dynamics
opts_intg = struct(...
    'tf',                           dt,...
    'simplify',                     true,...
    'number_of_finite_elements',    4);
dae = struct(...
    'x',    x,...
    'p',    [k_sym;z_H_sym],...
    'ode',  xdot);
intg = integrator('intg','rk',dae,opts_intg);
result = intg('x0',x,'p',[k_sym;z_H_sym]);
x_next = result.xf;
fd = Function('fd',{x,k_sym,z_H_sym},{x_next});

%% Formulate Optimization Problem
% Intermediate optimization variables
N_fp = N_steps;
k_step = t_step / dt;
N_k = N_steps * k_step;

% Opti Stack
opti = casadi.Opti();

% opt vars
X_traj = opti.variable(n_x,N_k);
Ufp_traj = opti.variable(n_ufp,N_fp);

% parameters
p_x_init = opti.parameter(n_x,1);
p_xcdot_des = opti.parameter(1,1);
p_ycdot_des = opti.parameter(1,1);
p_z_H = opti.parameter(1,1);        % nominal z height of com
p_ufp_max = opti.parameter(n_ufp,1);
p_ufp_min = opti.parameter(n_ufp,1);
p_k = opti.parameter(2,1); % [kx; ky]
p_mu = opti.parameter(1,1); % friction coefficient
p_kx = p_k(1);
p_ky = p_k(2);


% cost
opt_cost_vel = {};
opt_cost_avgvel = {};

% initial conditions
X_k = X_traj(:,1);
n = 0;      % foot step iteration
xst_abs = 0;
yst_abs = 0;
x_eos = {X_k};
x_bos = {};
xc_slip_limit = (p_mu + p_kx)*p_z_H / (1 - p_mu*p_kx);
yc_slip_limit = (p_mu + p_ky)*p_z_H / (1 - p_mu*p_ky);

% Loop through discrete trajectory
for k = 1:N_k-1
    k_pre = n*(k_step+1)+1;       % iterate pre-impact
    k_post = k_pre + 1;             % iterate post-impact
    xc_k = X_k(1);
    yc_k = X_k(2);
    Lx_k = X_k(3);
    Ly_k = X_k(4);
    Lz_k = X_k(5);
    xcdot_k = (Ly_k+p_ky*Lz_k)/(m*p_z_H);
    ycdot_k = (-Lx_k-p_kx(1)*Lz_k)/(m*p_z_H);
    if (k == k_pre)
        if n > 0 % initial condition fixed
            x_eos = [x_eos, {X_k}];
%             avgxvel_k = (x_eos{end}(1)-x_bos{end}(1))/t_step;
%             avgyvel_k = (x_eos{end}(2)-x_bos{end}(2))/t_step;
            
            
            avgxvel_k = Ufp_traj(1,n+1)/t_step;
            avgyvel_k = Ufp_traj(2,n+1)/t_step;
            
            avgvel_error = [...
                avgxvel_k-p_xcdot_des;
                avgyvel_k-p_ycdot_des];
            opt_cost_avgvel = [opt_cost_avgvel, {avgvel_error'*Q(n)*avgvel_error}];
            
            vel_error = [...
                xcdot_k-p_xcdot_des;
                ycdot_k-p_ycdot_des];
            opt_cost_vel = [opt_cost_vel, {vel_error'*Q(n)*vel_error}];
        end      
        Xk_end = [...
            xc_k-Ufp_traj(1,n+1);
            yc_k-Ufp_traj(2,n+1);
            Lx_k;
            Ly_k;
            Lz_k];     % update init position to reflect foot placement
        xst_abs = xst_abs + Ufp_traj(1,n+1);
        yst_abs = yst_abs + Ufp_traj(2,n+1);
        opti.subject_to(Ufp_traj(3,n+1) == p_kx*xst_abs + p_ky*yst_abs)
        X_k = X_traj(:,k+1);      
    elseif (k == k_post)
        % init state of n-th step
        x_bos = [x_bos, {X_k}];
        Xk_end = fd(X_k,p_k,p_z_H);
        X_k = X_traj(:,k+1);
        n = n + 1;      % increase step counter
    else
        Xk_end = fd(X_k,p_k,p_z_H);
        X_k = X_traj(:,k+1);
    end
    opti.subject_to(Xk_end == X_k);
    
    % GRF constraint
    opti.subject_to(xc_k <= xc_slip_limit);
    opti.subject_to(yc_k <= yc_slip_limit);
end
xc_k = X_k(1);
yc_k = X_k(2);
Lx_k = X_k(3);
Ly_k = X_k(4);
Lz_k = X_k(5);
xcdot_k = (Ly_k+p_ky*Lz_k)/(m*p_z_H);
ycdot_k = (-Lx_k-p_kx*Lz_k)/(m*p_z_H);
avgvel_error = [...
    xcdot_k-p_xcdot_des;
    ycdot_k-p_ycdot_des];
opt_cost_avgvel = [opt_cost_avgvel, {avgvel_error'*Q(n)*avgvel_error}];
% GRF constraint
opti.subject_to(-xc_slip_limit <= xc_k <= xc_slip_limit);
opti.subject_to(-yc_slip_limit <= yc_k <= yc_slip_limit);

% initial condition constraint
opti.subject_to(X_traj(:,1) == p_x_init);

% Foot Placement limits (only consider when there is more than one fp)
z_prev = 0;
for n = 1:N_fp
    Ufp_xy = Ufp_traj(1:2,n);
    Ufp_z = Ufp_traj(3,n);
    opti.subject_to(-p_ufp_max(1:2) <= Ufp_xy <= p_ufp_max(1:2))
    opti.subject_to(-p_ufp_max(3) <= Ufp_z - z_prev <= p_ufp_max(3));
    z_prev = Ufp_z;
end

% Combine cost, vars, constraints, parameters
final_angmom_error = x_eos{end}(3:end) - x_eos{end-1}(3:end);
opt_cost_stab = final_angmom_error' * 100*Q(n) * final_angmom_error;
% opt_cost_vel_total = sum(vertcat(opt_cost_vel{:}));
opt_cost_avgvel_total = sum(vertcat(opt_cost_avgvel{:}));

opti.minimize(opt_cost_avgvel_total);
% opti.minimize(opt_cost_vel_total + opt_cost_stab);
% opti.minimize(opt_cost_avgvel_total + opt_cost_stab);

%% Create an OPT solver
if sol_type == "qrqp"
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
    opti.solver('sqpmethod',opts);
    f_opti = opti.to_function('F_sqp',{p_x_init,p_xcdot_des,p_ycdot_des,p_z_H,p_ufp_max},{Ufp_traj});
    
    % test
%     opti.set_value(p_x_init,[0.1;0;0;0;0])
%     opti.set_value(p_xcdot_des,1);
%     opti.set_value(p_ycdot_des,1);
%     opti.set_value(p_z_H,0.8);
%     opti.set_value(p_ufp_delta,10);
%     opti.set_value(p_k,[tan(deg2rad(5)); tan(deg2rad(5))]);
%     sol = opti.solve();
%     xsol = sol.value(X_traj)
%     ufpsol = sol.value(Ufp_traj)
    
    % code generation
    %     F_sqp.save('F_sqp.casadi');
    %     F_sqp.generate('sqptest',struct('mex',true))
    %     disp('Compiling...')
    %     mex sqptest.c -DMATLAB_MEX_FILE
    %     disp('done')
    %     format long
end

%% Return symbolics and solver
info.sym_info.fd = fd;
info.ctrl_info.mpc.opti = opti;
info.ctrl_info.mpc.f_opti = f_opti;
info.ctrl_info.mpc.p_x_init = p_x_init;
info.ctrl_info.mpc.p_xcdot_des = p_xcdot_des;
info.ctrl_info.mpc.p_ycdot_des = p_ycdot_des;
info.ctrl_info.mpc.p_z_H = p_z_H;
info.ctrl_info.mpc.p_ufp_max = p_ufp_max;
info.ctrl_info.mpc.p_ufp_min = p_ufp_min;
info.ctrl_info.mpc.p_k = p_k;
info.ctrl_info.mpc.p_mu = p_mu;

