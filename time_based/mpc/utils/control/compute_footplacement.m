function x_fp = compute_footplacement(x_end,x_end_prev,p)
    %% Extract Inputs
    if p.fp.type == "comvel"
        x_fp_current = p.fp.x_fp;
        Kp_fp = p.fp.Kp_fp;
        Kd_fp = p.fp.Kd_fp;
        x_fp = x_fp_current + Kp_fp*(x_end(2) - p.com_vel_des) + Kd_fp*(x_end(2)-x_end_prev(2));
    elseif p.fp.type == "mpc"
        % solve optimization problem
    end
end

