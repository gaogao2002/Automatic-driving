 figure;
%% ��ȡ�ϰ���߽�����
load map.mat;
% obs = flip(map - map_ori,1);
%% ���Ҫ���Ը�����ͼ����Ҫע����δ���,����ע�͵����д���
% load('sysu_standard_new.mat');
% img = imread('sysu6001200.png');
% % ��ͼƬת��Ϊ�Ҷ�ͼ��
% gray_img = rgb2gray(img);
% % ������ֵ��ֵ��Ϊ0��1
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

%% ��ȡ��·�߽�����
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

%% ���ݺϲ�
x_env = [x_road; x_obs];
y_env = [y_road; y_obs];
save('env.mat', 'x_env', 'y_env');