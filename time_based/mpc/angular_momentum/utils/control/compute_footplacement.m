function [ufp_sol,x_sol,cost_sol] = compute_footplacement(p)
    %% Extract Inputs
    import casadi.*
    % parameters
    m = p.m;
    z_H = p.z_H;
    
    % state
    x_init = p.x_init;
    x_min = [-inf; -inf]; %p.x_min;
    x_max = [inf; inf]; % p.x_max;
    xcdot_des = p.xcdot_des;
    
    % control
    ufp_min = p.ufp_min; % p.ufp_min;
    ufp_max = p.ufp_max; % p.ufp_max;
    ufp_delta = p.ufp_delta; % p.ufp_delta;
    
    % mpc
    n_x = p.n_x;
    N_steps = p.N_steps;
    opti = p.opti;
    p_x_init = p.p_x_init;
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
        opti.set_value(p_x_init,x_init);
        opti.set_value(p_xcdot_des,xcdot_des);
        opti.set_value(p_z_H,z_H);
        opti.set_value(p_ufp_delta,ufp_delta);
        
        % Solve
        sol = opti.solve();
        
        % Extract Solution
        optvar = opti.x;
        x_sol = opti.value(optvar(1:end-N_steps));
        x_sol = reshape(x_sol,2,length(x_sol)/2);
        ufp_sol = opti.value(optvar(end-(N_steps-1):end));
        cost_sol = sol.value(opti.f);


    end
end

