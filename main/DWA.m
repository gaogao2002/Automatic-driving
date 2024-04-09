disp('Dynamic Window Approach sample program start!!')
 
%% �����˵ĳ���״̬[x(m),y(m),yaw(Rad),v(m/s),w(rad/s)]
global path_last;
global path_s;
% load path_last_special.mat;
load path_last.mat;
% path_last = [path_last(:,1), 600 - path_last(:,2)];
path_last_show = flip(path_last, 1);

path_s = calc_s(path_last);

% x=[0 0 pi/2 0 0]'; % 5x1�����о���  λ�� 0��0 ���� pi/2 ,�ٶȡ����ٶȾ�Ϊ0
x = [path_last(1,1) path_last(1,2) 0 0 0]'; 
start_state = x;
% �±�궨�� ״̬[x,y,yaw,v,w]
POSE_X      = 1;  %���� X
POSE_Y      = 2;  %���� Y
YAW_ANGLE   = 3;  %�����˺����
V_SPD       = 4;  %�������ٶ�
W_ANGLE_SPD = 5;  %�����˽��ٶ� 
 
goal = [path_last(end,1) path_last(end,2)];   % Ŀ���λ�� [,]
load env.mat;
env = flip([x_env, y_env], 1);
% x_env = x_env(1:100:end);
% x_env = x_env / 100;
% y_env = y_env(1:20:end);
% y_env = y_env / 100;
% �ϰ���λ���б� [x(m) y(m)]
obstacle=[env(1:20:end,1), env(1:20:end,2)];
 
      
obstacleR = 5.5;% ��ͻ�ж��õ��ϰ���뾶
global dt; 
dt = 0.1;% ʱ��

% �������˶�ѧģ�Ͳ���
% ����ٶ�,�����ת�ٶ�,���ٶ�,��ת���ٶ�,
% �ٶȷֱ���,ת�ٷֱ���]
Kinematic = [20000.0,toRadian(100.0),500.0,toRadian(140.0),15,toRadian(3)];
%����Kinematic���±꺬��
MD_MAX_V    = 1;%   ����ٶ�
MD_MAX_W    = 2;%   �����ת�ٶ�
MD_ACC      = 3;%   ���ٶ�
MD_VW       = 4;%   ��ת���ٶ�
MD_V_RESOLUTION  = 5;%  �ٶȷֱ���
MD_W_RESOLUTION  = 6;%  ת�ٷֱ���

l_ego = 24;
w_ego = 12;
h_ego = 0;
 
% ���ۺ������� [heading,dist,velocity,predictDT]
% ����÷ֵı��ء�����÷ֵı��ء��ٶȵ÷ֵı��ء���ǰģ��켣��ʱ��
evalParam = [0.8, 2000000, 100, 0.75];
 
area      = [0 1200 0 600];% ģ������Χ [xmin xmax ymin ymax]
 
% ģ��ʵ��Ľ��
result.x=[];   %�ۻ��洢�߹��Ĺ켣���״ֵ̬ 
tic; % �����������ʱ�俪ʼ
% global s_now;
% s_now = 0;    
figure;
map_ori=im2bw(imread('map_obs.png')); 
% ���Ҫ���Ը�����ͼ����Ҫע�͵�66�д��룬���ҽ��ע�͵�6��
map_ori = ~map;

imagesc(map_ori);
hold on;
plot(path_last_show(:,1), path_last_show(:,2));
hold on;
traj_last = [];
pic_num = 1;
DrawObstacle_plot(obstacle,obstacleR);

%% Main loop   ѭ������ 5000�� ָ���ﵽĿ�ĵ� ���� 5000�����н���
for i = 1:5000
    % DWA�������� ���ؿ����� u = [v(m/s),w(rad/s)] �� �켣
    [u,traj] = DynamicWindowApproach(x,Kinematic,goal,evalParam,obstacle,obstacleR);
    x = f(x,u);% �������ƶ�����һ��ʱ�̵�״̬�� ���ݵ�ǰ�ٶȺͽ��ٶ��Ƶ� ��һ�̵�λ�úͽǶ�
    
    % ��ʷ�켣�ı���
    result.x = [result.x; x'];  %���½�� ���е���ʽ ��ӵ�result.x
    disp(u);
%     s_traj = calc_s([result.x(:,1), result.x(:,2)]);
%     s_now = s_traj(end);
    % �Ƿ񵽴�Ŀ�ĵ�
    if norm(x(POSE_X:POSE_Y)-goal')<12   % norm��������������ϵ�������֮��ľ���
        disp('Arrive Goal!!');
        break;
    end

    %====Animation====
    hold off;               % �ر�ͼ�α��ֹ��ܡ� ��ͼ����ʱ��ȡ��ԭͼ����ʾ��

%     ArrowLength = 12;      % ��ͷ����
%     map_ori=im2bw(imread('map_obs.png')); 
    % ���Ҫ���Ը�����ͼ����Ҫע�͵�98�д��룬���ҽ��ע�͵�100��
    map_ori = ~map;
    imagesc(map_ori);
    hold on;
    title("ʱ�䣺"+num2str(i*0.1)+"s");
    plot(path_last_show(:,1), path_last_show(:,2));
    hold on;
    % ������
%     quiver(x,y,u,v) %�� x �� y ��ÿ����ӦԪ�ض�����ָ�������괦����������Ϊ��ͷ
%     aa = quiver(x(POSE_X), x(POSE_Y), ArrowLength*cos(x(YAW_ANGLE)), ArrowLength*sin(x(YAW_ANGLE)), 'ok'); % ���ƻ����˵�ǰλ�õĺ����ͷ
%     hold on;                                                     %����ͼ�α��ֹ��ܣ���ǰ�������ͼ�ζ������֣��Ӵ˻��Ƶ�ͼ�ζ�����������ͼ�εĻ����ϣ����Զ�����������ķ�Χ
%     delete(aa);
    plot(result.x(:,POSE_X),result.x(:,POSE_Y),'-r');hold on;    % �����߹�������λ�� ������ʷ���ݵ� X��Y����
    plot(goal(1),goal(2),'*r');hold on;                          % ����Ŀ��λ��
    
%     plot(obstacle(:,1),obstacle(:,2),'*k');hold on;              % ���������ϰ���λ��
    
    view(2);
    [verts_ego, facs_ego] = drawCuboid([l_ego, w_ego, h_ego]', [x(1), x(2), h_ego / 2]', [-x(3), 0, 0]', 'g', '0.1');
    patch('Faces',facs_ego,'Vertices',verts_ego','FaceColor','g','FaceAlpha',0.1);
    
    % ̽���켣 ���������۵Ĺ켣
    if ~isempty(traj) %�켣�ǿ�
        for it=1:length(traj(:,1))/5    %�������й켣��  traj ÿ5������ ��ʾһ���켣��
            ind = 1+(it-1)*5; %�� it ���켣��Ӧ��traj�е��±� 
            bb = plot(traj(ind,:),traj(ind+1,:),'-g');hold on;  %����һ���켣�ĵ㴮�����켣   traj(ind,:) ��ʾ��ind���켣������x����ֵ  traj(ind+1,:)��ʾ��ind���켣������y����ֵ
        end
    end
    
    axis(area); %����area���õ�ǰͼ�ε����귶Χ���ֱ�Ϊx�����С�����ֵ��y�����С���ֵ
    grid on;
    drawnow;  %ˢ����Ļ. ������ִ��ʱ�䳤����Ҫ����ִ��plotʱ��Matlab���򲻻����ϰ�ͼ�񻭵�figure�ϣ���ʱ��Ҫ��ʵʱ����ͼ���ÿһ���仯�������Ҫʹ�������䡣
    F=getframe(gcf);
    I=frame2im(F);
    [I,map_1]=rgb2ind(I,256);
    if pic_num == 1
        imwrite(I,map_1,'result_5_���Ź켣.gif','gif', 'Loopcount',inf,'DelayTime',0.1);
    else
        imwrite(I,map_1,'result_5_���Ź켣.gif','gif','WriteMode','append','DelayTime',0.1);
    end
    pic_num = pic_num + 1;
end
toc  %�����������ʱ��  ��ʽ��ʱ���ѹ� ** �롣

path_last_special = path_last;
path_last_special = path_last_special';
% path_last_special = [[start_state(1), start_state(2)]; path_last_special];
path_last_special(2,:) = 600 - path_last_special(2,:);
xi_start = [path_last_special(1,1):0.1:path_last_special(1,2), path_last_special(1,2)];
yi_start = interp1(path_last_special(1,1:2),path_last_special(2,1:2),xi_start,'linear');

xi_end = [min(path_last_special(1,end-1),path_last_special(1,end)):0.1:max(path_last_special(1,end-1),path_last_special(1,end)), max(path_last_special(1,end-1),path_last_special(1,end))];
yi_end = interp1(path_last_special(1,end-1:end),path_last_special(2,end-1:end),xi_end,'linear');
traj_gaobo = [start_state(1),600-start_state(2)];
path_last_special(:,1:2) = [];
path_last_special(:,end-1:end) = [];
path_last_special = [[xi_start; yi_start], path_last_special, [xi_end; yi_end]];
traj_last_x = result.x(:,1);
traj_last_y = result.x(:,2);
traj_last_y = 600 - traj_last_y;
traj_gaobo = [traj_gaobo;[traj_last_x,traj_last_y]];
traj_gaobo = traj_gaobo';
traj_last_x = traj_last_x - traj_last_x(1);
traj_last_y = traj_last_y - traj_last_y(1);
traj_last_yaw = -1 * result.x(:,3);
% traj_last_yaw = traj_last_yaw;
traj_last_v = result.x(:,4);
traj_last_x = [traj_last_x;goal(1)];
traj_last_y = [traj_last_y;goal(2)];

traj_last_yaw = [traj_last_yaw;traj_last_yaw(end)];
traj_last_v = [traj_last_v;0];

T = 0:dt:i * dt;
traj_last_x = [T', traj_last_x];
traj_last_y = [T', traj_last_y];
traj_last_yaw = [T',traj_last_yaw];
traj_last_v = [T', traj_last_v];

