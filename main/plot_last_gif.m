%% 本程序在运行完simulink得到mpc的跟踪结果后才可运行
figure;
%% 数据初始化
% load out_1.mat;

t_sim = out.tout;
x_sim = out.x_mpc.data;
y_sim = out.y_mpc.data;
yaw_sim = out.yaw_mpc.data;
x_sim = x_sim + 80;
y_sim = 600 - (y_sim + 80);

load env.mat;
% load path_6.mat;

pic_num = 1;

l_ego = 24;
w_ego = 12;
h_ego = 0;
%% 开始绘图
% path_last_show = flip(path_last, 1);
path_last_show = traj_gaobo';
path_length = length(x_sim);
for i = 1:20:path_length
    hold off;
    % 判断是否是最后一帧
    if path_length - i < 20
        i = path_length;
    end
    % 地图
    imagesc(map);
    colormap(flipud(gray));
    hold on;
    axis equal;
    axis off;
    % 起点
    plot(80, 520, 'ro', 'markersize', 5);
    hold on;
    % 终点
    plot(1130, 150, 'ro', 'markersize', 5);
    hold on;
    title("时间："+num2str(t_sim(i))+"s");
    % 规划轨迹
    plot(path_last_show(:,1), 600-path_last_show(:,2), 'g-', 'linewidth', 1.2);
    hold on;
    % 实际轨迹
    plot(x_sim(1:i), y_sim(1:i), 'b-', 'linewidth', 1.5);
    hold on;
    % 车辆
    view(2);
    [verts_ego, facs_ego] = drawCuboid([l_ego, w_ego, h_ego]', [x_sim(i), y_sim(i), h_ego / 2]', [yaw_sim(i), 0, 0]', 'g', '0.1');
    patch('Faces',facs_ego,'Vertices',verts_ego','FaceColor','g','FaceAlpha',0.1);
    drawnow;
    % gif绘制
    F=getframe(gcf);
    I=frame2im(F);
    [I,map_1]=rgb2ind(I,256);
    if pic_num == 1
        imwrite(I,map_1,'result_5_跟踪轨迹.gif','gif', 'Loopcount',inf,'DelayTime',0.1);
    else
        imwrite(I,map_1,'result_5_跟踪轨迹.gif','gif','WriteMode','append','DelayTime',0.1);
    end
    pic_num = pic_num + 1;
end

