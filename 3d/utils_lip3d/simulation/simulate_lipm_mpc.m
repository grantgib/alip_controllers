function [info] = simulate_lipm_mpc(info)
%% Extract Input
% sym_info
n_x = info.sym_info.n_x;
n_ufp = info.sym_info.n_ufp;
m = info.sym_info.m;
g = info.sym_info.g;
kx = info.sym_info.kx;
ky = info.sym_info.ky;
 
% gait_info
z_H = info.gait_info.z_H;
x_init = info.gait_info.x_init;
phase_type = info.gait_info.phase_type;
t_step = info.gait_info.t_step;
num_steps = info.gait_info.num_steps;
num_steps_change_vel = info.gait_info.num_steps_change_vel;
xyzst = info.gait_info.xyzst;
increase_vel = info.gait_info.increase_vel;
decrease_vel = info.gait_info.decrease_vel;
xcdot_des = info.gait_info.xcdot_des;
ycdot_des = info.gait_info.ycdot_des;

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
p_ycdot_des = info.ctrl_info.mpc.p_ycdot_des;
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
xy_abs_traj = {x_init(1:2)+xyzst(1:2)};
z_curr = kx*x_init(1) + ky*x_init(2) + z_H;
z_abs_traj = {z_curr};
xyzst_traj = {xyzst};
xcdot_des_traj = {xcdot_des};
ycdot_des_traj = {ycdot_des};
iter_impact_traj = {1};
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
        'ycdot_des',    ycdot_des,...
        'ufp_max',      ufp_max,...
        'ufp_min',      ufp_min,...
        'ufp_delta',    ufp_delta,...
        'N_steps',      N_steps,...
        'n_x',          n_x,...
        'n_ufp',        n_ufp,...
        'opti',         opti,...
        'p_x_init',     p_x_init,...
        'p_xcdot_des',  p_xcdot_des,...
        'p_ycdot_des',  p_ycdot_des,...
        'p_z_H',        p_z_H,...
        'p_ufp_delta',  p_ufp_delta);
    [ufp_sol,x_sol,cost_sol] = compute_footplacement(ufp_params); % Select foot placement for next step
    ufp = ufp_sol(:,1);
        
    % Update
    xyzst = xyzst + ufp;
    x_init = [...
        x_init(1:2)-ufp(1:2);
        x_init(3:5)];
    
    % Update 
    % Forward simulate step
    ode_info = struct(...
        'g',    g,...
        'm',    m,...
        'z_H',  z_H,...
        'kx',   kx,...
        'ky',   ky);
    [tsol,xsol] = ode45(@(t,x) ode_lipm(t,x,ode_info), tspan, x_init, options);
    tsol = tsol';
    xsol = xsol';
    zcsol = 0.*xsol(1,:);
    for j = 1:length(tsol)
        zcsol(:,j) = kx*(xsol(1,j)+xyzst(1)) + ky*(xsol(2,j)+xyzst(2)) + z_H;
    end
    
    
    % Store data
    if i > 1
        tsol = tsol + t_traj(end);
    end
    iter_impact_traj = [iter_impact_traj, {length(tsol)+length(t_traj)}];
    t_traj = [t_traj, tsol];
    x_traj = [x_traj, {xsol}];
    xy_abs_traj = [xy_abs_traj, {xsol(1:2,:)+xyzst(1:2)}];
    z_abs_traj = [z_abs_traj, {zcsol}];
    xyzst_traj = [xyzst_traj, {xyzst.*ones(3,length(tsol))}];
    xcdot_des_traj = [xcdot_des_traj, {xcdot_des.*ones(1,length(tsol))}];
    ycdot_des_traj = [ycdot_des_traj, {ycdot_des.*ones(1,length(tsol))}];
    ufp_sol_traj = [ufp_sol_traj, {ufp_sol}];
    x_sol_traj = [x_sol_traj, {x_sol}];
    cost_sol_traj = [cost_sol_traj, {cost_sol}];
    ufp_traj = [ufp_traj,{ufp}];
    
    % Update
    x_init = xsol(:,end);
    
    % Change desired velocity
    if mod(i,num_steps_change_vel) == 0 && increase_vel
        xcdot_des = xcdot_des + 5;
        ycdot_des = ycdot_des + 2;
        if ycdot_des >= 1
            decrease_vel = true;
            increase_vel = false;
        end
    end
    if mod(i,num_steps_change_vel) == 0 && decrease_vel
        xcdot_des = xcdot_des - 5;
        ycdot_des = ycdot_des - 2;
        if ycdot_des <= -2
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
sol_info.xy_abs_traj = xy_abs_traj;
sol_info.z_abs_traj = z_abs_traj;
sol_info.xyzst_traj = xyzst_traj;
sol_info.xcdot_des_traj = xcdot_des_traj;
sol_info.ycdot_des_traj = ycdot_des_traj;
sol_info.iter_impact_traj = iter_impact_traj;

%% Update info struct
info.sol_info = sol_info;
end