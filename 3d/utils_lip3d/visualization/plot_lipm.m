function [] = plot_lipm(info)
%% Extract Input
m = info.sym_info.m;
z_H = info.gait_info.z_H;
t = info.sol_info.t_traj;
x = cell2mat(info.sol_info.x_traj);
xabs = cell2mat(info.sol_info.x_abs_traj);
xcdot_des = cell2mat(info.sol_info.xcdot_des_traj);
ycdot_des = cell2mat(info.sol_info.ycdot_des_traj);
ufp = cell2mat(info.sol_info.ufp_traj);
ufp_sol = info.sol_info.ufp_sol_traj;
[n_x,n_length] = size(x);

%% State plots
headers = {'$xc$ [m]','$yc$ [m]', '$\dot{yc}$ [m/s]', '$\dot{xc}$ [m/s]'};
sz = 30;
for i = 1:n_x
    figure(i);
    if i == 1
        plot(t,x(i,:),'color',[0 0.4470 0.7410]);
        xlabel('time [sec]','interpreter','latex','FontSize',sz);
        ylabel(headers(i),'interpreter','latex','FontSize',sz);
        grid on;
    elseif i == 2
        plot(t,x(i,:),'color',[0 0.4470 0.7410]);
        xlabel('time [sec]','interpreter','latex','FontSize',sz);
        ylabel(headers(i),'interpreter','latex','FontSize',sz);
        grid on;
    elseif i == 3
        hold on;
        plot(t,ycdot_des,':r','LineWidth',4);
        plot(t,-x(i,:)/(m*z_H),'color',[0 0.4470 0.7410]);
        xlabel('time [sec]','interpreter','latex','FontSize',sz);
        ylabel(headers(i),'interpreter','latex','FontSize',sz);
        grid on;
        legend('desired y velocity','actual y velocity');
    elseif i == 4
        hold on;
        plot(t,xcdot_des,':r','LineWidth',4);
        plot(t,x(i,:)/(m*z_H),'color',[0 0.4470 0.7410]);
        xlabel('time [sec]','interpreter','latex','FontSize',sz);
        ylabel(headers(i),'interpreter','latex','FontSize',sz);
        grid on;
        legend('desired x velocity','actual x velocity');
    end
    
    
end

figure
hold on;
plot(xabs(1,:),xabs(2,:),'LineWidth',4);
xlabel('xc_{abs}');
ylabel('yc_{abs}');
grid on;

figure
hold on;
plot(xcdot_des,ycdot_des,':r','LineWidth',4);
plot(x(4,:)/(m*z_H),-x(3,:)/(m*z_H),'color',[0 0.4470 0.7410]);
grid on;
xlabel('xcdot');
ylabel('ycdot');
legend('desired velocity','actual velocity')

figure
plot(t,xabs(1,:));
grid on;
xlabel('time [sec]','interpreter','latex','FontSize',sz);
ylabel('$xc_{abs}$','interpreter','latex','FontSize',sz);

figure 
plot(t,xabs(2,:));
grid on;
xlabel('time [sec]','interpreter','latex','FontSize',sz);
ylabel('$yc_{abs}$','interpreter','latex','FontSize',sz);

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

