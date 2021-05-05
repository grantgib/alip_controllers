function [ufp_sol] = compute_footplacement(p)
    %% Extract Inputs
    import casadi.*
    % parameters
    m = p.m;
    z_H = p.z_H;
    
    % state
    xc_init = p.xc_init;
    x_min = [-inf; -inf]; %p.x_min;
    x_max = [inf; inf]; % p.x_max;
    xcdot_des = p.xcdot_des;
    
    % control
    ufp = p.ufp;
    ufp_init = ufp;
    ufp_min = p.ufp_min; % p.ufp_min;
    ufp_max = p.ufp_max; % p.ufp_max;
    ufp_delta = p.ufp_delta; % p.ufp_delta;
    
    % mpc
    n_x = p.n_x;
    N_k = p.N_k;
    N_fp = p.N_fp;
    opti = p.opti;
    p_xcinit = p.p_xcinit;
    p_xcdot_des = p.p_xcdot_des;
    p_z_H = p.p_z_H;
    p_ufp_delta = p.p_ufp_delta;
    
    
    
    %% Foot Placement
    if p.type == "mpc"
        %% MPC        
        % Initial guess
%         opti.set_initial(X_traj,) 
%         opti.set_initial(Ufp_traj,)
        
        % params        
        opti.set_value(p_xcinit,xc_init);
        opti.set_value(p_xcdot_des,xcdot_des);
        opti.set_value(p_z_H,z_H);
        opti.set_value(p_ufp_delta,ufp_delta);
        
        % Solve
        sol = opti.solve();
        
        % Extract Solution
        optvar = opti.x;
        x_sol = opti.value(optvar(1:end-N_fp));
        x_sol = reshape(x_sol,2,length(x_sol)/2);
        ufp_sol = opti.value(optvar(end-(N_fp-1):end));
        cost_sol = sol.value(opti.f);


    end
end

