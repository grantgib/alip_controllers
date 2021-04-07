function [ufp,ufp_next] = compute_footplacement(p)
    %% Extract Inputs
    
    % state
    x_init = p.x_init;
    x_min = [-inf; -inf]; %p.x_min;
    x_max = [inf; inf]; % p.x_max;
    xdot_des = p.xdot_des;
    
    % control
    ufp = p.ufp;
    ufp_init = ufp;
    ufp_min = -10; % p.ufp_min;
    ufp_max = 10; % p.ufp_max;
    ufp_delta = 0.5; % p.ufp_delta;
    
    % mpc
    n_x = p.n_x;
    N_k = p.N_k;
    N_fp = p.N_fp;
    solver = p.solver;
    
    %% Foot Placement
    if p.type == "mpc"
        %% MPC
        % Opt Variables
        opt_var_guess = [...
            repmat([0;0],N_k,1);
            repmat(2,N_fp,1)];

        opt_params = xdot_des;
        
        % Bounds
        lb_opt_var = [...
            x_init;
            repmat(x_min,N_k-1,1)];
        ub_opt_var = [...
            x_init;
            repmat(x_max,N_k-1,1)];
        if true || ~(ufp_init)  % fp_init is false for initial foot placement selection
            lb_opt_var = [lb_opt_var; repmat(ufp_min,N_fp,1)];
            ub_opt_var = [ub_opt_var; repmat(ufp_max,N_fp,1)];
        else
            lb_opt_var = [lb_opt_var; ufp_init; repmat(ufp_min,N_fp-1,1)];
            ub_opt_var = [ub_opt_var; ufp_init; repmat(ufp_max,N_fp-1,1)];
        end
        
        lb_constr = repmat([0;0],N_k-1,1);
        ub_constr = repmat([0;0],N_k-1,1);
        if N_fp > 1
            lb_constr = [lb_constr; repmat(-ufp_delta,N_fp-1,1)];
            ub_constr = [ub_constr; repmat(ufp_delta,N_fp-1,1)];
        end
        
        % Solve
        sol = solver(...
            'x0',   opt_var_guess,...
            'lbx',  lb_opt_var,...
            'ubx',  ub_opt_var,...
            'lbg',  lb_constr,...
            'ubg',  ub_constr,...
            'p',    opt_params);
        
        % Extract Solution
        idx_fp = n_x*(N_k)+1;
        cost_sol = sol.f;
        x_sol = full(reshape(sol.x(1:n_x*(N_k+1)),n_x,N_k+1));
        fp_sol = full(reshape(sol.x(idx_fp:end),1,N_fp));
        
        ufp = fp_sol(1);
        ufp_next = fp_sol(1);

    end
end

