function [info] = simulate_lipm_mpc(info)
%% Extract Input
% sym_info
n_x = info.sym_info.n_x;
n_ufp = info.sym_info.n_ufp;
m = info.sym_info.m;
g = info.sym_info.g;


% gait_info
z_H = info.gait_info.z_H;

x_init = info.gait_info.x_init;
phase_type = info.gait_info.phase_type;
t_step = info.gait_info.t_step;
num_steps = info.gait_info.num_steps;
num_steps_change_vel = info.gait_info.num_steps_change_vel;
num_steps_change_slope = info.gait_info.num_steps_change_slope;
xyzst = info.gait_info.xyzst;
vel_increase = info.gait_info.vel_increase;
vel_decrease = info.gait_info.vel_decrease;
slope_increase = info.gait_info.slope_increase;
slope_decrease = info.gait_info.slope_decrease;
vel_des = info.gait_info.vel_des;
angle_x = info.gait_info.angle_x;
angle_y = info.gait_info.angle_y;
[kx,ky] = ang_to_slope(angle_x,angle_y);
[xcdot_des,ycdot_des] = extract_vel_des(vel_des,kx,ky);

% ctrl_info
ufp_type = info.ctrl_info.type;
ufp_max = info.ctrl_info.mpc.ufp_max;
N_steps = info.ctrl_info.mpc.N_steps;
opti = info.ctrl_info.mpc.opti;
f_opti = info.ctrl_info.mpc.f_opti;
p_x_init = info.ctrl_info.mpc.p_x_init;
p_xcdot_des = info.ctrl_info.mpc.p_xcdot_des;
p_ycdot_des = info.ctrl_info.mpc.p_ycdot_des;
p_z_H = info.ctrl_info.mpc.p_z_H;
p_ufp_max = info.ctrl_info.mpc.p_ufp_max;
p_k = info.ctrl_info.mpc.p_k;

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
zc_init = kx*x_init(1) + ky*x_init(2) + z_H;
z_abs_traj = {zc_init};
xyzst_traj = {xyzst};
vel_des_traj = {vel_des};
xcdot_des_traj = {xcdot_des};
ycdot_des_traj = {ycdot_des};
iter_impact_traj = {1};
iter_vel_change_traj = {};
iter_slope_change_traj = {};
x_sol_traj = {};
ufp_sol_traj = {};
cost_sol_traj = {};
ufp_traj = {};
k_traj = {[kx; ky]};
t_impact_traj = {};
avgvel_traj = {};

%% Simulate steps
for iter = 1:num_steps
    % Compute next footplacement based on desired COM velocity 
    %   * Would be computed in parallel with ode45
    %   * Must compute initial fp for first step
    ufp_params = struct(...
        'type',         ufp_type,...
        'm',            m,...
        'z_H',          z_H,...
        'x_init',       x_init,...
        'xcdot_des',    xcdot_des,...
        'ycdot_des',    ycdot_des,...
        'ufp_max',      ufp_max,...
        'k',            [kx;ky],...
        'N_steps',      N_steps,...
        'n_x',          n_x,...
        'n_ufp',        n_ufp,...
        'opti',         opti,...
        'p_x_init',     p_x_init,...
        'p_xcdot_des',  p_xcdot_des,...
        'p_ycdot_des',  p_ycdot_des,...
        'p_z_H',        p_z_H,...
        'p_ufp_max',    p_ufp_max,...
        'p_k',          p_k);
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
    if iter > 1
        tsol = tsol + t_traj(end);
    end
    iter_impact_traj = [iter_impact_traj, {length(tsol)+length(t_traj)}];
    t_traj = [t_traj, tsol];
    x_traj = [x_traj, {xsol}];
    xy_abs_traj = [xy_abs_traj, {xsol(1:2,:)+xyzst(1:2)}];
    z_abs_traj = [z_abs_traj, {zcsol}];
    xyzst_traj = [xyzst_traj, {xyzst.*ones(3,length(tsol))}];
    vel_des_traj = [vel_des_traj, {vel_des.*ones(1,length(tsol))}];
    xcdot_des_traj = [xcdot_des_traj, {xcdot_des.*ones(1,length(tsol))}];
    ycdot_des_traj = [ycdot_des_traj, {ycdot_des.*ones(1,length(tsol))}];
    k_traj = [k_traj, {[kx;ky].*ones(1,length(tsol))}];
    ufp_sol_traj = [ufp_sol_traj, {ufp_sol}];
    x_sol_traj = [x_sol_traj, {x_sol}];
    cost_sol_traj = [cost_sol_traj, {cost_sol}];
    ufp_traj = [ufp_traj,{ufp}];
    t_impact_traj = [t_impact_traj, {tsol(end)}];
    avgvel_traj = [avgvel_traj, {[(xsol(1,end)-xsol(1,1))/t_step; (xsol(2,end)-xsol(2,1))/t_step]}];
%     avgvel_traj = [avgvel_traj, {[(xyzst_traj{end}(1,1)-xyzst_traj{end-1}(1,1))/t_step; (xyzst_traj{end}(2,1)-xyzst_traj{end-1}(2,1))/t_step]}];
%     avgvel_traj = [avgvel_traj, {[(ufp_traj{end}(1))/t_step; (ufp_traj{end}(2))/t_step]}];
    
    % Update
    x_init = xsol(:,end);
    
    % Change desired velocity
    if mod(iter,num_steps_change_vel) == 0 && vel_increase
        vel_des = vel_des + 0.5;
        [xcdot_des,ycdot_des] = extract_vel_des(vel_des,kx,ky);
        if ycdot_des >= 1
            vel_decrease = true;
            vel_increase = false;
        end
        iter_vel_change_traj = [iter_vel_change_traj, {length(tsol)+length(t_traj)}];
    end
    if mod(iter,num_steps_change_vel) == 0 && vel_decrease
        vel_des = vel_des + 0.5;
        [xcdot_des,ycdot_des] = extract_vel_des(vel_des,kx,ky);
        if ycdot_des <= -2
            vel_decrease = false;
            vel_increase = true;
        end
        iter_vel_change_traj = [iter_vel_change_traj, {length(tsol)+length(t_traj)}];
    end
    
    % Change desired slope
    if mod(iter,num_steps_change_slope) == 0 && slope_increase
        angle_x = angle_x + 2;
%         angle_y = angle_y + 2;
        [kx,ky] = ang_to_slope(angle_x,angle_y);
        [xcdot_des,ycdot_des] = extract_vel_des(vel_des,kx,ky);
        iter_slope_change_traj = [iter_slope_change_traj, {length(tsol)+length(t_traj)}];
    end
    if mod(iter,num_steps_change_slope) == 0 && slope_decrease
        angle_x = angle_x - 1;
%         angle_y = angle_y - 1;
        [kx,ky] = ang_to_slope(angle_x,angle_y);
        [xcdot_des,ycdot_des] = extract_vel_des(vel_des,kx,ky);
        iter_slope_change_traj = [iter_slope_change_traj, {length(tsol)+length(t_traj)}];
    end
end

%% Post Create Swing Trajectories
% pst_traj = {};
% if length(xyzst_traj) >= 3
%     for i = 1:length(xyzst_traj)-2
%         pst_prev = xyzst_traj{i}(:,1);
%         pst_next = xyzst_traj{i+1}(:,1);
%         vst = pst_next - pst_prev;
% %         free = [0 0 1 1 0 0];
% %         M = length(free);
% %         alphas = bezfit([0 0.2 0.8 1],[pst_prev pst_prev+vst/5 pst_next+vst/5 pst_next],free);
%         M = 3;
%         alphas = [pst_prev pst_prev+vst/M pst_next-vst/M pst_next];
%         % plot check
%         s = linspace(0,1,100);
%         amp = 10;
%         for j = 1:length(s)
% %             check(:,j) = bezier(alphas,s(j));
%             check2(:,j) = 0.5*((1+amp*cos(pi*s(j)))*pst_prev + (1-amp*cos(pi*s(j)))*pst_next); 
%         end
%         figure
%         hold on; grid on;
%         line([pst_prev(1) pst_next(1)],[pst_prev(2) pst_next(2)],[pst_prev(3) pst_next(3)],'color','k')
%         scatter3(check2(1,:),check2(2,:),check2(3,:));
% %         scatter3(alphas(1,:),alphas(2,:),alphas(3,:),36,'r','filled')
%     end
%     
% end


%% Update Solution structure
sol_info.t_traj = t_traj;
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
sol_info.iter_vel_change_traj = iter_vel_change_traj;
sol_info.iter_slope_change_traj = iter_slope_change_traj;
sol_info.k_traj = k_traj;
sol_info.t_impact_traj = t_impact_traj;
sol_info.avgvel_traj = avgvel_traj;

%% Update info struct
info.sol_info = sol_info;
end

