function [] = animate_lipm_2d(t,x,p)
%% Extract Inputs
x_st = p.x_st_traj;
len = length(t);
iter_impact = p.impact_traj;

%% Initialize Figure
figure;
p_st = [x_st(1); 0];
p_com = [x(1,1); p.z_const];
wd = 15;
sz = 5000;
alpha = 0.1;
blue = [0 0.4470 0.7410];
green = [0.4660 0.6740 0.1880];
red = [0.6350 0.0780 0.1840];
yellow = [0.9290 0.6940 0.1250];
leg = line([p_st(1) p_com(1)],[p_st(2) p_com(2)],...
    'LineWidth',wd,...
    'Color',blue);
hold on;
com = scatter(p_com(1),p_com(2),...
    sz,red,'filled');
hold on;
grid on;
axis([-5 5 0 1.5])
xlabel('x [m]');
ylabel('z [m]');
k = 1;

% inital transparent state
hold on;
line([p_st(1) p_com(1)],[p_st(2) p_com(2)],...
    'LineWidth',wd,...
    'Color',[blue,alpha]);
hold on;
scatter(p_com(1),p_com(2),sz,red,'filled',...
    'MarkerFaceAlpha',alpha)
hold on;

%% Animate Figure
for i = 1:len
    p_st = [x_st(i); 0];
    p_com = [x(1,i); p.z_const];
    set(leg,'XData',[p_st(1) p_com(1)],'YData',[p_st(2) p_com(2)])
    set(com,'XData',p_com(1),'YData',p_com(2));
    axis([-5+p_com(1) 5+p_com(1) 0 1.5])
    
    if i == iter_impact(k)+1
        hold on;
        line([p_st(1) p_com(1)],[p_st(2) p_com(2)],...
            'LineWidth',wd,...
            'Color',[blue,alpha]);
        hold on;
        scatter(p_com(1),p_com(2),sz,red,'filled',...
            'MarkerFaceAlpha',alpha)
        hold on;
        k = k + 1;
    end
    
    drawnow;
    pause(0.01);
end




end
