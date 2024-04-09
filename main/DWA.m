disp('Dynamic Window Approach sample program start!!')
 
%% 机器人的初期状态[x(m),y(m),yaw(Rad),v(m/s),w(rad/s)]
global path_last;
global path_s;
% load path_last_special.mat;
load path_last.mat;
% path_last = [path_last(:,1), 600 - path_last(:,2)];
path_last_show = flip(path_last, 1);

path_s = calc_s(path_last);

% x=[0 0 pi/2 0 0]'; % 5x1矩阵行矩阵  位置 0，0 航向 pi/2 ,速度、角速度均为0
x = [path_last(1,1) path_last(1,2) 0 0 0]'; 
start_state = x;
% 下标宏定义 状态[x,y,yaw,v,w]
POSE_X      = 1;  %坐标 X
POSE_Y      = 2;  %坐标 Y
YAW_ANGLE   = 3;  %机器人航向角
V_SPD       = 4;  %机器人速度
W_ANGLE_SPD = 5;  %机器人角速度 
 
goal = [path_last(end,1) path_last(end,2)];   % 目标点位置 [,]
load env.mat;
env = flip([x_env, y_env], 1);
% x_env = x_env(1:100:end);
% x_env = x_env / 100;
% y_env = y_env(1:20:end);
% y_env = y_env / 100;
% 障碍物位置列表 [x(m) y(m)]
obstacle=[env(1:20:end,1), env(1:20:end,2)];
 
      
obstacleR = 5.5;% 冲突判定用的障碍物半径
global dt; 
dt = 0.1;% 时间

% 机器人运动学模型参数
% 最高速度,最高旋转速度,加速度,旋转加速度,
% 速度分辨率,转速分辨率]
Kinematic = [20000.0,toRadian(100.0),500.0,toRadian(140.0),15,toRadian(3)];
%定义Kinematic的下标含义
MD_MAX_V    = 1;%   最高速度
MD_MAX_W    = 2;%   最高旋转速度
MD_ACC      = 3;%   加速度
MD_VW       = 4;%   旋转加速度
MD_V_RESOLUTION  = 5;%  速度分辨率
MD_W_RESOLUTION  = 6;%  转速分辨率

l_ego = 24;
w_ego = 12;
h_ego = 0;
 
% 评价函数参数 [heading,dist,velocity,predictDT]
% 航向得分的比重、距离得分的比重、速度得分的比重、向前模拟轨迹的时间
evalParam = [0.8, 2000000, 100, 0.75];
 
area      = [0 1200 0 600];% 模拟区域范围 [xmin xmax ymin ymax]
 
% 模拟实验的结果
result.x=[];   %累积存储走过的轨迹点的状态值 
tic; % 估算程序运行时间开始
% global s_now;
% s_now = 0;    
figure;
map_ori=im2bw(imread('map_obs.png')); 
% 如果要测试给定地图，就要注释掉66行代码，并且解除注释第6行
map_ori = ~map;

imagesc(map_ori);
hold on;
plot(path_last_show(:,1), path_last_show(:,2));
hold on;
traj_last = [];
pic_num = 1;
DrawObstacle_plot(obstacle,obstacleR);

%% Main loop   循环运行 5000次 指导达到目的地 或者 5000次运行结束
for i = 1:5000
    % DWA参数输入 返回控制量 u = [v(m/s),w(rad/s)] 和 轨迹
    [u,traj] = DynamicWindowApproach(x,Kinematic,goal,evalParam,obstacle,obstacleR);
    x = f(x,u);% 机器人移动到下一个时刻的状态量 根据当前速度和角速度推导 下一刻的位置和角度
    
    % 历史轨迹的保存
    result.x = [result.x; x'];  %最新结果 以列的形式 添加到result.x
    disp(u);
%     s_traj = calc_s([result.x(:,1), result.x(:,2)]);
%     s_now = s_traj(end);
    % 是否到达目的地
    if norm(x(POSE_X:POSE_Y)-goal')<12   % norm函数来求得坐标上的两个点之间的距离
        disp('Arrive Goal!!');
        break;
    end

    %====Animation====
    hold off;               % 关闭图形保持功能。 新图出现时，取消原图的显示。

%     ArrowLength = 12;      % 箭头长度
%     map_ori=im2bw(imread('map_obs.png')); 
    % 如果要测试给定地图，就要注释掉98行代码，并且解除注释第100行
    map_ori = ~map;
    imagesc(map_ori);
    hold on;
    title("时间："+num2str(i*0.1)+"s");
    plot(path_last_show(:,1), path_last_show(:,2));
    hold on;
    % 机器人
%     quiver(x,y,u,v) %在 x 和 y 中每个对应元素对组所指定的坐标处将向量绘制为箭头
%     aa = quiver(x(POSE_X), x(POSE_Y), ArrowLength*cos(x(YAW_ANGLE)), ArrowLength*sin(x(YAW_ANGLE)), 'ok'); % 绘制机器人当前位置的航向箭头
%     hold on;                                                     %启动图形保持功能，当前坐标轴和图形都将保持，从此绘制的图形都将添加在这个图形的基础上，并自动调整坐标轴的范围
%     delete(aa);
    plot(result.x(:,POSE_X),result.x(:,POSE_Y),'-r');hold on;    % 绘制走过的所有位置 所有历史数据的 X、Y坐标
    plot(goal(1),goal(2),'*r');hold on;                          % 绘制目标位置
    
%     plot(obstacle(:,1),obstacle(:,2),'*k');hold on;              % 绘制所有障碍物位置
    
    view(2);
    [verts_ego, facs_ego] = drawCuboid([l_ego, w_ego, h_ego]', [x(1), x(2), h_ego / 2]', [-x(3), 0, 0]', 'g', '0.1');
    patch('Faces',facs_ego,'Vertices',verts_ego','FaceColor','g','FaceAlpha',0.1);
    
    % 探索轨迹 画出待评价的轨迹
    if ~isempty(traj) %轨迹非空
        for it=1:length(traj(:,1))/5    %计算所有轨迹数  traj 每5行数据 表示一条轨迹点
            ind = 1+(it-1)*5; %第 it 条轨迹对应在traj中的下标 
            bb = plot(traj(ind,:),traj(ind+1,:),'-g');hold on;  %根据一条轨迹的点串画出轨迹   traj(ind,:) 表示第ind条轨迹的所有x坐标值  traj(ind+1,:)表示第ind条轨迹的所有y坐标值
        end
    end
    
    axis(area); %根据area设置当前图形的坐标范围，分别为x轴的最小、最大值，y轴的最小最大值
    grid on;
    drawnow;  %刷新屏幕. 当代码执行时间长，需要反复执行plot时，Matlab程序不会马上把图像画到figure上，这时，要想实时看到图像的每一步变化情况，需要使用这个语句。
    F=getframe(gcf);
    I=frame2im(F);
    [I,map_1]=rgb2ind(I,256);
    if pic_num == 1
        imwrite(I,map_1,'result_5_最优轨迹.gif','gif', 'Loopcount',inf,'DelayTime',0.1);
    else
        imwrite(I,map_1,'result_5_最优轨迹.gif','gif','WriteMode','append','DelayTime',0.1);
    end
    pic_num = pic_num + 1;
end
toc  %输出程序运行时间  形式：时间已过 ** 秒。

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

