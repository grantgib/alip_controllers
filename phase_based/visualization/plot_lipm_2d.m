function [] = plot_lipm_2d(t,x,params)
[n_x,n_length] = size(x);

headers = {'$x$ [m]','$\dot{x}$ [m/s]'};
sz = 30;
for i = 1:n_x
    figure(i);
    if i == 2
        hold on;
        plot(t,params.vel_des_traj,':r','LineWidth',4);
    end
    plot(t,x(i,:),'color',[0 0.4470 0.7410]);
    xlabel('time [sec]','interpreter','latex','FontSize',sz);
    ylabel(headers(i),'interpreter','latex','FontSize',sz);
    grid on;
    if i == 2
        legend('actual velocity','desired velocity');
    end
end

