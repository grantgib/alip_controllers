function [] = plot_lipm(info)
%% Extract Input
m = info.sym_info.m;
k_traj = cell2mat(info.sol_info.k_traj);
z_H = info.gait_info.z_H;
t = info.sol_info.t_traj;
x = cell2mat(info.sol_info.x_traj);
xyabs = cell2mat(info.sol_info.xy_abs_traj);
zabs = cell2mat(info.sol_info.z_abs_traj);
xcdot_des = cell2mat(info.sol_info.xcdot_des_traj);
ycdot_des = cell2mat(info.sol_info.ycdot_des_traj);
ufp = cell2mat(info.sol_info.ufp_traj);
ufp_sol = info.sol_info.ufp_sol_traj;
t_impact = cell2mat(info.sol_info.t_impact_traj);
avgvel = cell2mat(info.sol_info.avgvel_traj);
xc_slip_traj = cell2mat(info.sol_info.xc_slip_traj);
yc_slip_traj = cell2mat(info.sol_info.yc_slip_traj);

[n_x,n_length] = size(x);

%% State plots
headers = {'$xc$ [m]','$yc$ [m]', '$\dot{yc}$ [m/s]', '$\dot{xc}$ [m/s]', '$L^z\ [kg-m^2/s]$'};
sz = 30;
for i = 1:n_x
    figure(i);
    if i == 1
        hold on;
        plot(t,x(i,:),'color',[0 0.4470 0.7410]);
        plot(t,xc_slip_traj,':r');
        plot(t,-xc_slip_traj,':r');
        xlabel('time [sec]','interpreter','latex','FontSize',sz);
        ylabel(headers(i),'interpreter','latex','FontSize',sz);
        grid on;
    elseif i == 2
        hold on;
        plot(t,x(i,:),'color',[0 0.4470 0.7410]);
        plot(t,yc_slip_traj,':r');
        plot(t,-yc_slip_traj,':r');
        xlabel('time [sec]','interpreter','latex','FontSize',sz);
        ylabel(headers(i),'interpreter','latex','FontSize',sz);
        grid on;
    elseif i == 3
        hold on;
        plot(t,ycdot_des,':r','LineWidth',4);
        plot(t,-x(i,:)/(m*z_H),'color',[0 0.4470 0.7410]);
        plot(t_impact,avgvel(2,:),'k', 'LineWidth',2);
        xlabel('time [sec]','interpreter','latex','FontSize',sz);
        ylabel(headers(i),'interpreter','latex','FontSize',sz);
        grid on;
        legend('desired y velocity','actual y velocity','average y vel');
    elseif i == 4
        hold on;
        plot(t,xcdot_des,':r','LineWidth',4);
        plot(t,x(i,:)/(m*z_H),'color',[0 0.4470 0.7410]);
        plot(t_impact,avgvel(1,:),'k', 'LineWidth',2);
        xlabel('time [sec]','interpreter','latex','FontSize',sz);
        ylabel(headers(i),'interpreter','latex','FontSize',sz);
        grid on;
        legend('desired x velocity','actual x velocity', 'average x vel');
    elseif i == 5
        hold on;
        plot(t,x(i,:));
        xlabel('time [sec]','interpreter','latex','FontSize',sz);
        ylabel(headers(i),'interpreter','latex','FontSize',sz);
        grid on;
    end
    
    
end

figure
hold on;
plot(xyabs(1,:),xyabs(2,:),'LineWidth',4);
xlabel('xc_{abs}');
ylabel('yc_{abs}');
title('X-Y Plane View');
grid on; axis equal;

% figure
% hold on;
% plot(xcdot_des,ycdot_des,':r','LineWidth',4);
% plot(x(4,:)/(m*z_H),-x(3,:)/(m*z_H),'color',[0 0.4470 0.7410]);
% grid on;
% xlabel('xcdot');
% ylabel('ycdot');
% legend('desired velocity','actual velocity')

figure
plot(t,xyabs(1,:));
grid on;
xlabel('time [sec]','interpreter','latex','FontSize',sz);
ylabel('$xc_{abs}$','interpreter','latex','FontSize',sz);

figure 
plot(t,xyabs(2,:));
grid on;
xlabel('time [sec]','interpreter','latex','FontSize',sz);
ylabel('$yc_{abs}$','interpreter','latex','FontSize',sz);

figure;
hold on;
plot(t,zabs);
grid on;
xlabel('time [sec]','interpreter','latex','FontSize',sz);
ylabel('$zc_{abs}$','interpreter','latex','FontSize',sz);

figure;
hold on; view(45,10)
plot3(xyabs(1,:),xyabs(2,:),zabs);
zabs_des = k_traj(1,:).*xyabs(1,:) + k_traj(2,:).*xyabs(2,:) + z_H;
plot3(xyabs(1,:),xyabs(2,:),zabs_des,'LineStyle',':','LineWidth',2,'color','r');
grid on;
xlabel('$xc_{abs}$','interpreter','latex','FontSize',sz);
ylabel('$yc_{abs}$','interpreter','latex','FontSize',sz);
zlabel('$zc_{abs}$','interpreter','latex','FontSize',sz);
axis equal;

%% Foot Placement

[~,len_ufp] = size(ufp);
n = 1:1:len_ufp;

figure
hold on;
scatter(n,ufp(1,:));
plot(ufp(1,:));
title('x foot placment');
xlabel('time [sec]');
grid on;

figure
hold on;
scatter(n,ufp(2,:));
plot(ufp(2,:));
title('y foot placment');
xlabel('time [sec]');
grid on;

figure
hold on;
scatter(n,ufp(3,:));
plot(ufp(3,:));
title('z foot placement');
xlabel('time [sec]');
grid on;

