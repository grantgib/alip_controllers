function [info] = simulate_lipm_mpc(info)
%% Extract Input
% sym_info
n_x = info.sym_info.n_x;
m = info.sym_info.m;
 

% gait_info
zc = info.gait_info.z_const;
x_init = info.gait_info.x_init;
phase_type = info.gait_info.phase_type;
t_step = info.gait_info.t_step;
num_steps = info.gait_info.num_steps;
num_steps_change_vel = info.gait_info.num_steps_change_vel;
x_stance = info.gait_info.x_stance;
increase_vel = info.gait_info.increase_vel;
decrease_vel = info.gait_info.decrease_vel;
xdot_des = info.gait_info.xdot_com_des;

% ctrl_info
ufp_type = info.ctrl_info.type;

x_min = info.ctrl_info.mpc.x_min;
x_max = info.ctrl_info.mpc.x_max;

ufp = info.ctrl_info.mpc.ufp;
ufp_max = info.ctrl_info.mpc.ufp_max;
ufp_min = info.ctrl_info.mpc.ufp_min;
ufp_delta = info.ctrl_info.mpc.ufp_delta;
N_k = info.ctrl_info.mpc.N_k;
N_fp = info.ctrl_info.mpc.N_fp;

solver = info.ctrl_info.mpc.solver;

%% Initialize
ode_params = struct(...
    'phase_type',   phase_type,...
    't_step',       t_step);
options = odeset('RelTol',1e-8,'AbsTol',1e-8,'Events',@(t,x) event_impact_step(t,x,ode_params));
tspan = 0:0.01:2;

% storage variables
t_traj = [];
x_mpc_traj = [];
ufp_traj = [];
x_traj = [];
x_abs_traj = [];
x_st_traj = [];
xdot_des_traj = [];
impact_traj = [];

%% Simulate steps
for i = 1:num_steps
    % Compute next footplacement based on desired COM velocity 
    %   * Would be computed in parallel with ode45
    %   * Must compute initial fp for first step
    ufp_params = struct(...
        'type',         ufp_type,...
        'm',            m,...
        'zc',           zc,...
        'x_init',       x_init,...
        'x_min',        x_min,...
        'x_max',        x_max,...
        'xdot_des',     xdot_des,...
        'ufp',          ufp,...
        'ufp_max',      ufp_max,...
        'ufp_min',      ufp_min,...
        'ufp_delta',    ufp_delta,...
        'n_x',          n_x,...
        'N_k',          N_k,...
        'N_fp',         N_fp,...
        'solver',       solver);
    [ufp,ufp_next] = compute_footplacement(ufp_params); % Select foot placement for next step
    
    % Forward simulate step
    [tsol,xsol] = ode45(@(t,x) ode_lipm(t,x,info), tspan, x_init, options);
    tsol = tsol';
    xsol = xsol';
    
    % Store data
    if i > 1
        tsol = tsol + t_traj(end);
    end
    impact_traj = [impact_traj, length(tsol)+length(t_traj)];
    t_traj = [t_traj, tsol];
    x_traj = [x_traj, xsol];
    x_abs_traj = [x_abs_traj, xsol + [x_stance; 0]];
    x_st_traj = [x_st_traj, x_stance*ones(1,length(tsol))];
    xdot_des_traj = [xdot_des_traj, xdot_des*ones(1,length(tsol))];
    ufp_traj = [ufp_traj, ufp];
    
    % Update state & stance leg
    x_stance = x_stance +  ufp;
    x_init = [xsol(1,end)-ufp; xsol(2,end)];
    
    % Change desired velocity
    if mod(i,num_steps_change_vel) == 0 && increase_vel
        xdot_des = xdot_des + 1;
        if xdot_des > 11
            decrease_vel = true;
            increase_vel = false;
        end
    end
    if mod(i,num_steps_change_vel) == 0 && decrease_vel
        xdot_des = xdot_des - 1;
        if xdot_des < 8
            decrease_vel = false;
            increase_vel = true;
        end
    end
end

%% Update Solution structure
sol_info.t_traj = t_traj;
sol_info.x_mpc_traj = x_mpc_traj;
sol_info.fp_traj = ufp_traj;
sol_info.x_traj = x_traj;
sol_info.x_abs_traj = x_abs_traj;
sol_info.x_st_traj = x_st_traj;
sol_info.xdot_com_des_traj = xdot_des_traj;
sol_info.impact_traj = impact_traj;
sol_info.x_st_traj = x_st_traj;

%% Update info struct
info.sol_info = sol_info;
end