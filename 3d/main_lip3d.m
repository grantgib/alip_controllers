% Planar Linear Inverted Pendulum Mode w/ COM velocity footplacement
clear; clc; close all;
restoredefaultpath;
addpath(genpath('utils_lip3d/'));
if isunix
    addpath(genpath('/home/grantgib/workspace/toolbox/casadi-linux-matlabR2014b-v3.5.5'));
else
    disp('Add casadi windows path');
end
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
    'anim', 1);

% Symbolic variables
m = 32;
info.sym_info = struct(...
    'n_x',      5,...
    'n_ufp',    3,...
    'g',        9.81,...
    'm',        m);

% Gait parameters
user_set_params = input("Enter (1) if you want to set your own gait paramters and \nEnter (0) if you want to use default params: ");

if user_set_params
    t_step_period = input("Desired step period (Defualt = 0.4): ");
    num_steps = input("Desired number of steps (Defualt = 8): ");
    vel_des = input("Desired CoM velocity (Default = 1): ");
    mu_des = input("Desired Coefficient of Friction (Default = 0.8): ");
    angle_x = input("Desired Ground x slope angle in deg (Default = 10): ");
    angle_y = input("Desired Ground y slope angle in deg (Default = 5): ");
else
    t_step_period = 0.4;
    num_steps = 8;
    vel_des = 1.0; % m/s
    mu_des = 0.8;
    angle_x = 10; % in degrees
    angle_y = 5;
    disp("t_step_period (s): " + t_step_period);
    disp("num_steps: " + num_steps);
    disp("vel_des (m/s): " + vel_des);
    disp("mu_des: " + mu_des);
    disp("ground x slope angle (degrees): " + angle_x);
    disp("ground y slope angle (degrees): " + angle_y);
end

z_H = 0.6;
[kx,ky] = ang_to_slope(angle_x,angle_y);
xc_init = 0;
yc_init = 0;
vel_init = 0;
[xcdot_init,ycdot_init] = extract_vel_des(vel_init,kx,ky);
Lz_init = m *(-xcdot_init*yc_init + xc_init*ycdot_init);
Lx_init = -m*z_H*ycdot_init - kx*Lz_init;
Ly_init = m*z_H*xcdot_init - ky*Lz_init;
x_init = [xc_init; yc_init; Lx_init; Ly_init; Lz_init];
info.gait_info = struct(...
    'phase_type',               "time",... % time
    'x_init',                   x_init,... % [x; L] ~ x; m*H*xcom_dot
    'z_H',                      z_H,...
    't_step',                   t_step_period,...
    'xyzst',                    [0;0;0],...
    'vel_des',                  vel_des,...
    'angle_x',                  angle_x,...
    'angle_y',                  angle_y,...
    'mu',                       mu_des,...
    'num_steps',                num_steps,... %8,...
    'num_steps_change_vel',     inf,...
    'num_steps_change_slope',   inf,... 
    'vel_increase',             false,...
    'vel_decrease',             false,...
    'slope_increase',           false,...
    'slope_decrease',           false);

% Control parameters
info.ctrl_info.type = "mpc";
N_steps = 5;    % look ahead
q = 10;   
for i = 1:N_steps
    Q(i) = q; %q^i    % increase exponentially for each step (pseudo terminal cost)
end
info.ctrl_info.mpc = struct(...
    'sol_type',         'qrqp',...
    'ufp_max',          [0.5; 0.5; 0.3],...
    'ufp_min',          [-0.5; -0.5; 0],...
    'dt',               0.005,...
    'N_steps',          N_steps,...
    'Q',                Q);
      
% Solution stuct
info.sol_info = struct();

%% Formulate Optimization
disp("Formulating Optimization...");
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
% disp("***** xcdot des traj ***");
% cell2mat(info.sol_info.xcdot_des_traj)
% disp("***** ycdot des traj ***");
% cell2mat(info.sol_info.ycdot_des_traj)
% 
% disp("***** k traj ****");
% cell2mat(info.sol_info.k_traj)
