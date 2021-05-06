disp("Solving Test...");
%% Initial vars
x_init = [-2;10];
x_min = [-inf; -inf];
x_max = [inf; inf];
ufp_init = 2;
ufp_min = -inf;
ufp_max = inf;
delta_fp = 2;
p_xdot_des = 0;

%% Opt varss
opt_params = p_xdot_des;

opt_var_guess = [...
    repmat([0;0],N_k,1);
    repmat(2,N_fp,1)];

%% Bounds
% "lift" x and fp initial conditions
lb_opt_var = [...
    x_init;
    repmat(x_min,N_k-1,1);
    ufp_init;
    repmat(ufp_min,N_fp-1,1)];
ub_opt_var = [...
    x_init;
    repmat(x_max,N_k-1,1);
    ufp_init;
    repmat(ufp_max,N_fp-1,1)];

lb_constr = repmat([0;0],N_k-1,1);
ub_constr = repmat([0;0],N_k-1,1);

if N_fp > 1
    lb_constr = [lb_constr; repmat(-delta_fp,N_fp-1,1)]; 
    ub_constr = [ub_constr; repmat(delta_fp,N_fp-1,1)];
end

%% Solve
sol = info.ctrl_info.mpc.solver(...
    'x0',   opt_var_guess,...
    'lbx',  lb_opt_var,...
    'ubx',  ub_opt_var,...
    'lbg',  lb_constr,...
    'ubg',  ub_constr,...
    'p',    opt_params);

%% Results
disp('============== RESULTS ==================');
n_x = info.sym_info.n_x;
idx_fp = n_x*(N_k)+1;
time = 0:dt:(N_k-1)*dt;
cost_sol = sol.f;
x_sol = full(reshape(sol.x(1:n_x*(N_k)),n_x,N_k));
fp_sol = full(reshape(sol.x(idx_fp:end),1,N_fp));
figure('NumberTitle','off','Name','x_{com} - x_{st}');
plot(time,x_sol(1,:));
grid on;
figure('Numbertitle','off','Name','xdot_{com}');
plot(time,x_sol(2,:));
grid on;
figure('Numbertitle','off','Name','fp_{traj}');
scatter(1:1:N_fp,fp_sol); hold on; plot(fp_sol);
grid on;
disp('FP SOL'); disp(fp_sol);
disp('End of Test!');