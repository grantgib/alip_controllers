function [xcdot_des,ycdot_des] = extract_vel_des(vel_des,kx,ky)
% convert desired velocity to component velocities based on slope
if ( ky == 0 || (ky == 0 && kx == 0) )
    xcdot_des = vel_des;
    ycdot_des = 0;
elseif ( kx == 0 && ky ~= 0 )
    xcdot_des = 0;
    ycdot_des = vel_des;
else
r = ky/kx;
xcdot_des = vel_des * (1/sqrt(1+r^2));
ycdot_des = r * xcdot_des;
end
end

