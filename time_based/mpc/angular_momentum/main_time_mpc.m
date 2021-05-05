% Planar Linear Inverted Pendulum Mode w/ COM velocity footplacement
clear; clc; close all;
restoredefaultpath;
addpath(genpath('utils/'));
addpath(genpath('/home/grantgib/workspace/toolbox/casadi-linux-matlabR2014b-v3.5.5'));
set(0,'DefaultFigureWindowStyle','docked');

%% Initialize Data Structures
% Higher level structure
info = struct(...
    'vis_info',     struct(),...
    'sym_info',     struct(),...
    'ctrl_info',    struct(),...
    'gait_info',    struct(),...
    'sol_info',     struct());

% Visualization info
info.vis_info = struct(...
    'plot', 1,...
    'anim', 0);

% Symbolic variables
info.sym_info = struct(...
    'n_x',      2,...
    'g',        9.81,...
    'm',        32,...
    'k_slope',  0);

% Gait parameters
z_H = 0.8;
xc_init = 0.1;
xcdot_init = 1;
Lst_init = info.sym_info.m * z_H * xcdot_init;
x_init = [xc_init; Lst_init];
info.gait_info = struct(...
    'phase_type',           "time",... % time
    'x_init',               x_init,... % [x; L] ~ x; m*H*xcom_dot
    'z_H',                  z_H,...
    't_step',               0.4,...
    'x_stance',             0,...
    'xcdot_des',            1,...
    'num_steps',            6,...
    'num_steps_change_vel', 10,...
    'increase_vel',         false,...
    'decrease_vel',         true);

% Control parameters
info.ctrl_info.type = "mpc";
N_steps = 5;    % look ahead
q = 10;   
for i = 1:N_steps
    Q(i) = i*q^i;     % increase exponentially for each step (pseudo terminal cost)
end
info.ctrl_info.mpc = struct(...
    'sol_type',         'qrqp',...
    'x_min',            [-inf; -inf],...
    'x_max',            [inf; inf],...
    'ufp_max',          100,...
    'ufp_min',          -100,...
    'ufp_delta',        100,...
    'dt',               0.005,...
    'N_steps',          N_steps,...
    'Q',                Q);
      
% Solution stuct
info.sol_info = struct();

%% Formulate Optimization
disp("Formulating Optimization...");
% info = formulate_mpc(info);
info = formulate_mpc_opti(info);
disp("Optimization Solver Created!");
% test_solver(); % test solver

%% Initialize Gait/Simulation Parameters
disp("Simulating...");
info = simulate_lipm_mpc(info);
disp("Simulation Complete");

%% Visualization
% Plot
if info.vis_info.plot
    plot_lipm(info);
end

% Animate
if info.vis_info.anim
    animate_lipm(info)
end

%% Extras
x_traj = info.sol_info.x_traj;
x_sol_traj = info.sol_info.x_sol_traj;


