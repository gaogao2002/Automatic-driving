%% ��������������simulink�õ�mpc�ĸ��ٽ����ſ�����
figure;
%% ���ݳ�ʼ��
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
%% ��ʼ��ͼ
% path_last_show = flip(path_last, 1);
path_last_show = traj_gaobo';
path_length = length(x_sim);
for i = 1:20:path_length
    hold off;
    % �ж��Ƿ������һ֡
    if path_length - i < 20
        i = path_length;
    end
    % ��ͼ
    imagesc(map);
    colormap(flipud(gray));
    hold on;
    axis equal;
    axis off;
    % ���
    plot(80, 520, 'ro', 'markersize', 5);
    hold on;
    % �յ�
    plot(1130, 150, 'ro', 'markersize', 5);
    hold on;
    title("ʱ�䣺"+num2str(t_sim(i))+"s");
    % �滮�켣
    plot(path_last_show(:,1), 600-path_last_show(:,2), 'g-', 'linewidth', 1.2);
    hold on;
    % ʵ�ʹ켣
    plot(x_sim(1:i), y_sim(1:i), 'b-', 'linewidth', 1.5);
    hold on;
    % ����
    view(2);
    [verts_ego, facs_ego] = drawCuboid([l_ego, w_ego, h_ego]', [x_sim(i), y_sim(i), h_ego / 2]', [yaw_sim(i), 0, 0]', 'g', '0.1');
    patch('Faces',facs_ego,'Vertices',verts_ego','FaceColor','g','FaceAlpha',0.1);
    drawnow;
    % gif����
    F=getframe(gcf);
    I=frame2im(F);
    [I,map_1]=rgb2ind(I,256);
    if pic_num == 1
        imwrite(I,map_1,'result_5_���ٹ켣.gif','gif', 'Loopcount',inf,'DelayTime',0.1);
    else
        imwrite(I,map_1,'result_5_���ٹ켣.gif','gif','WriteMode','append','DelayTime',0.1);
    end
    pic_num = pic_num + 1;
end

