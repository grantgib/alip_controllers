function [] = plot_lipm(info)
%% Extract Input
t = info.sol_info.t_traj;
x = info.sol_info.x_traj;
xabs = info.sol_info.x_abs_traj;
ufp = info.sol_info.fp_traj;
[n_x,n_length] = size(x);

%% State plots
headers = {'$x$ [m]','$\dot{x}$ [m/s]'};
sz = 30;
for i = 1:n_x
    figure(i);
    if i == 2
        hold on;
        plot(t,info.sol_info.xdot_com_des_traj,':r','LineWidth',4);
    end
    plot(t,x(i,:),'color',[0 0.4470 0.7410]);
    xlabel('time [sec]','interpreter','latex','FontSize',sz);
    ylabel(headers(i),'interpreter','latex','FontSize',sz);
    grid on;
    if i == 2
        legend('desired velocity','actual velocity');
    end
end

figure
plot(t,xabs(1,:));
grid on;
xlabel('time [sec]','interpreter','latex','FontSize',sz);
ylabel('$x_{abs}$','interpreter','latex','FontSize',sz);

%% Foot Placement
figure
n = 1:1:length(ufp);
scatter(n,ufp);
hold on; 
plot(ufp);
title('foot placment');
xlabel('time [sec]');
grid on;

