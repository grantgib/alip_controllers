% Planar Linear Inverted Pendulum Mode w/ COM velocity footplacement
clear; clc; close all;
restoredefaultpath;
addpath(genpath('utils/'));

%% Initialize
set(0,'DefaultFigureWindowStyle','docked');
visualize = struct(...
    'plot',     1,...
    'anim',     1);
params = struct(...
    'gait_type',            "time",... % phase, time
    'g',                    9.81,...
    'z_const',              1,...
    't_step',               0.25,...     % time-based gait
    'x_stance',             0,...
    'com_vel_des',          10,...
    'num_steps',            500,...
    'num_steps_change_vel', 50,...
    'increase_vel',         false,...
    'decrease_vel',         true);
params.fp = struct(...
    'type',   	"comvel",... % comvel, mpc
    'x_fp',     2,...
    'Kp_fp',    0.3,...
    'Kd_fp',    0.45);
options = odeset('RelTol',1e-8,'AbsTol',1e-8,'Events',@(t,x) event_impact_step(t,x,params));
tspan = 0:0.01:2;
x_init = [-1; 10];
x_init_prev = x_init;

% storage trajectories
t_traj = [];
x_traj = [];
x_st_traj = [];
vel_des_traj = [];
impact_traj = [];


%% Simulate steps
for i = 1:params.num_steps
    [t,x,t_end,x_end] = ode45(@(t,x) ode_lipm_2d(t,x,params), tspan, x_init, options);
    t = t';
    x = x';
    if i == 1
        x_end_prev = x_end; % initialize x_end_prev on first step
    end
    
    % store data
    if i > 1
        t = t + t_traj(end);
    end
    impact_traj = [impact_traj, length(t)+length(t_traj)];
    t_traj = [t_traj, t];
    x_traj = [x_traj, x + [params.x_stance; 0]];
    x_st_traj = [x_st_traj, params.x_stance*ones(1,length(t))];
    vel_des_traj = [vel_des_traj, params.com_vel_des*ones(1,length(t))];
    
    % Update footplacement based on desired COM velocity
    x_fp = compute_footplacement(x_end,x_end_prev,params);
    params.fp.x_fp = x_fp;
    
    % Update state & stance leg
    params.x_stance = params.x_stance + x(1,end) + params.fp.x_fp;
    x_end_prev = x_end;
    x_init = [-params.fp.x_fp;
              x(2,end)];
    
    % Change desired velocity
    if mod(i,params.num_steps_change_vel) == 0 && params.increase_vel
        params.com_vel_des = params.com_vel_des + 1;
        if params.com_vel_des > 11
            params.decrease_vel = true;
            params.increase_vel = false;
        end
    end
    if mod(i,params.num_steps_change_vel) == 0 && params.decrease_vel
        params.com_vel_des = params.com_vel_des - 1;
        if params.com_vel_des < 8
            params.decrease_vel = false;
            params.increase_vel = true;
        end
    end      
end
params.vel_des_traj = vel_des_traj;
params.impact_traj = impact_traj;
params.x_st_traj = x_st_traj;

% Plot
if visualize.plot
    plot_lipm_2d(t_traj,x_traj,params);
end

% Animate
if visualize.anim
    animate_lipm_2d(t_traj,x_traj,params)
end

