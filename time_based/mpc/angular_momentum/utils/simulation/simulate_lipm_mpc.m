function [info] = simulate_lipm_mpc(info)
%% Extract Input
% sym_info
n_x = info.sym_info.n_x;
m = info.sym_info.m;
 
% gait_info
z_H = info.gait_info.z_H;
x_init = info.gait_info.x_init;
phase_type = info.gait_info.phase_type;
t_step = info.gait_info.t_step;
num_steps = info.gait_info.num_steps;
num_steps_change_vel = info.gait_info.num_steps_change_vel;
xst = info.gait_info.x_stance;
increase_vel = info.gait_info.increase_vel;
decrease_vel = info.gait_info.decrease_vel;
xcdot_des = info.gait_info.xcdot_des;

% ctrl_info
ufp_type = info.ctrl_info.type;

x_min = info.ctrl_info.mpc.x_min;
x_max = info.ctrl_info.mpc.x_max;

ufp_max = info.ctrl_info.mpc.ufp_max;
ufp_min = info.ctrl_info.mpc.ufp_min;
ufp_delta = info.ctrl_info.mpc.ufp_delta;
N_steps = info.ctrl_info.mpc.N_steps;

opti = info.ctrl_info.mpc.opti;
f_opti = info.ctrl_info.mpc.f_opti;
p_x_init = info.ctrl_info.mpc.p_x_init;
p_xcdot_des = info.ctrl_info.mpc.p_xcdot_des;
p_z_H = info.ctrl_info.mpc.p_z_H;
p_ufp_delta = info.ctrl_info.mpc.p_ufp_delta;


%% Initialize
ode_params = struct(...
    'phase_type',   phase_type,...
    't_step',       t_step);
options = odeset('RelTol',1e-8,'AbsTol',1e-8,'Events',@(t,x) event_impact_step(t,x,ode_params));
tspan = 0:0.005:2;

% storage variables
t_traj = 0;
x_traj = {x_init};
x_abs_traj = {x_init(1)+xst};
x_st_traj = {xst};
xcdot_des_traj = {xcdot_des};
iter_impact_traj = {};
x_sol_traj = {};
ufp_sol_traj = {};
cost_sol_traj = {};
ufp_traj = {};

%% Simulate steps
for i = 1:num_steps
    % Compute next footplacement based on desired COM velocity 
    %   * Would be computed in parallel with ode45
    %   * Must compute initial fp for first step
    ufp_params = struct(...
        'type',         ufp_type,...
        'm',            m,...
        'z_H',          z_H,...
        'x_init',       x_init,...
        'x_min',        x_min,...
        'x_max',        x_max,...
        'xcdot_des',    xcdot_des,...
        'ufp_max',      ufp_max,...
        'ufp_min',      ufp_min,...
        'ufp_delta',    ufp_delta,...
        'N_steps',      N_steps,...
        'n_x',          n_x,...
        'opti',         opti,...
        'p_x_init',     p_x_init,...
        'p_xcdot_des',  p_xcdot_des,...
        'p_z_H',        p_z_H,...
        'p_ufp_delta',  p_ufp_delta);
    [ufp_sol,x_sol,cost_sol] = compute_footplacement(ufp_params); % Select foot placement for next step
    ufp = ufp_sol(1);
        
    % Update
    xst = xst + ufp;
    x_init = [...
        x_init(1)-ufp;
        x_init(2)];
    
    % Update 
    % Forward simulate step
    [tsol,xsol] = ode45(@(t,x) ode_lipm(t,x,info), tspan, x_init, options);
    tsol = tsol';
    xsol = xsol';
    
    % Store data
    if i > 1
        tsol = tsol + t_traj(end);
    end
    iter_impact_traj = [iter_impact_traj, {length(tsol)+length(t_traj)}];
    t_traj = [t_traj, tsol];
    x_traj = [x_traj, {xsol}];
    x_abs_traj = [x_abs_traj, {xsol(1,:)+xst}];
    x_st_traj = [x_st_traj, {xst*ones(1,length(tsol))}];
    xcdot_des_traj = [xcdot_des_traj, {xcdot_des*ones(1,length(tsol))}];
    ufp_sol_traj = [ufp_sol_traj, {ufp_sol}];
    x_sol_traj = [x_sol_traj, {x_sol}];
    cost_sol_traj = [cost_sol_traj, {cost_sol}];
    ufp_traj = [ufp_traj,{ufp}];
    
    % Update
    x_init = xsol(:,end);
    
    % Change desired velocity
    if mod(i,num_steps_change_vel) == 0 && increase_vel
        xcdot_des = xcdot_des + 2;
        if xcdot_des >= 1
            decrease_vel = true;
            increase_vel = false;
        end
    end
    if mod(i,num_steps_change_vel) == 0 && decrease_vel
        xcdot_des = xcdot_des - 1;
        if xcdot_des <= -2
            decrease_vel = false;
            increase_vel = true;
        end
    end
end

%% Update Solution structure
sol_info.t_traj = t_traj;
sol_info.x_mpc_traj = x_sol_traj;
sol_info.ufp_sol_traj = ufp_sol_traj;
sol_info.x_sol_traj = x_sol_traj;
sol_info.ufp_traj = ufp_traj;
sol_info.x_traj = x_traj;
sol_info.x_abs_traj = x_abs_traj;
sol_info.x_st_traj = x_st_traj;
sol_info.xcdot_des_traj = xcdot_des_traj;
sol_info.iter_impact_traj = iter_impact_traj;
sol_info.x_st_traj = x_st_traj;

%% Update info struct
info.sol_info = sol_info;
end