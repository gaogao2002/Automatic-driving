 figure;
%% 获取障碍物边界坐标
load map.mat;
% obs = flip(map - map_ori,1);
%% 如果要测试给定地图，不要注释这段代码,并且注释第三行代码
% load('sysu_standard_new.mat');
% img = imread('sysu6001200.png');
% % 将图片转换为灰度图像
% gray_img = rgb2gray(img);
% % 将亮度值二值化为0和1
% map_ori = imbinarize(gray_img);
%% 
obs = map - map_ori;

imwrite(obs,'obs.png')
fig_obs = imread('obs.png');
B_obs = bwboundaries(fig_obs, 'noholes');
x_obs = [];
y_obs = [];
for i = 1:length(B_obs)
    eval(['x_obs_', num2str(i),' = B_obs{i}(:,2)']);
    eval(['y_obs_', num2str(i),' = B_obs{i}(:,1)']);
    eval(['plot(x_obs_', num2str(i), ',y_obs_', num2str(i), ')']);
    hold on;
    eval(['x_obs = [x_obs; ','x_obs_', num2str(i), ']']);
    eval(['y_obs = [y_obs; ','y_obs_', num2str(i), ']']);
end

%% 获取道路边界坐标
% map = flip(map,1);
env = imfill(~map, 'holes'); 
% B_road_1 = bwboundaries(~env,8,'noholes');
B_road_2 = bwboundaries(env,8,'noholes');
% x_road1 = B_road_1{1}(:,2);
% y_road1 = B_road_1{1}(:,1);
x_road2 = B_road_2{1}(:,2);
y_road2 = B_road_2{1}(:,1);
% plot(x_road1,y_road1);
% hold on;
figure;
plot(x_road2,y_road2);
hold off;
x_road = [x_road2];
y_road = [y_road2];

%% 数据合并
x_env = [x_road; x_obs];
y_env = [y_road; y_obs];
save('env.mat', 'x_env', 'y_env');