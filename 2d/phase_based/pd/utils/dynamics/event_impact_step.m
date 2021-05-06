function [value,isterminal,direction] = event_impact_step(t,x,p)
%% Options
% value(i) is the value of the ith event function.
% 
% isterminal(i) = 1 if the integration is to terminate at a zero of this
%       event function. Otherwise, it is 0.
%     
% direction(i) = 0 if all zeros are to be located (the default). 
%       A value of +1 locates only zeros where the event function
%       is increasing, and -1 locates only zeros where the event 
%       function is decreasing.

%% Output
if p.gait_type == "phase"
    value = x(1) - p.x_step;
    isterminal = 1;
    direction = 1;
elseif p.gait_type == "time"
    value = t - p.t_step;
    isterminal = 1;
    direction = 1;
else
    disp("Restart and define gait type correctly")
    pause
end


end

