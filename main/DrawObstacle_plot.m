%% ���������ϰ���λ��
% ���������obstacle �����ϰ��������   obstacleR �ϰ���İ뾶
function [] = DrawObstacle_plot(obstacle,obstacleR)
r = obstacleR; 
theta = 0:pi/20:2*pi;
for id=1:length(obstacle(:,1))
 x = r * cos(theta) + obstacle(id,1); 
 y = r  *sin(theta) + obstacle(id,2);
 plot(x,y,'-m');hold on; 
end
% plot(obstacle(:,1),obstacle(:,2),'*m');hold on;              % ���������ϰ���λ��
    
 
end