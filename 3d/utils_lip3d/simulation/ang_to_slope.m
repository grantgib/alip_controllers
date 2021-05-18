function [kx,ky] = ang_to_slope(angle_x,angle_y)
% Assume angles are in degrees
kx = tan(deg2rad(angle_x));
ky = tan(deg2rad(angle_y));
end

