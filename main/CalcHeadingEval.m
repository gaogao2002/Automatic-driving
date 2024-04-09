%% heading的评价函数计算
% 输入参数：当前位置、目标位置
% 输出参数：航向参数得分  当前车的航向和相对于目标点的航向 偏离程度越小 分数越高 最大180分
function [heading, target_idx] = CalcHeadingEval(x, goal)
% global target_x;
% global target_y;
global path_last;
% global target_idx;

if x(1)>1000 && x(2)<160
    x(1) = path_last(end - 1,1);
    x(2) = path_last(end - 1,2);
end

theta = toDegree(x(3));% 机器人朝向
target_idx = dsearchn([path_last(:,1), 600 - path_last(:,2)], [x(1), 600 - x(2)]);

if target_idx >= length(path_last(:,1)) - 15
    target_idx = length(path_last(:,1)) - 15;
end

target_x = path_last(target_idx + 15, 1);
target_y = path_last(target_idx + 15, 2);
goalTheta = toDegree(atan2(target_y-x(2),target_x-x(1)));% 目标点相对于机器人本身的方位 
if goalTheta > theta
    targetTheta = goalTheta-theta;% [deg]
else
    targetTheta = theta-goalTheta;% [deg]
end

lastTheta = toDegree(atan2(goal(2)-x(2),goal(1)-x(1)));% 终点相对于机器人本身的方位 
if lastTheta > theta
    targetTheta2 = lastTheta-theta;% [deg]
else
    targetTheta2 = theta-lastTheta;% [deg]
end

% figure;
% plot(path_last(:,1), 600 - path_last(:,2));
% hold on;
% plot(x(1), 600 - x(2), 'ro');
% hold on;
% plot(path_last(target_idx,1), 600 - path_last(target_idx,2), 'gx');
% hold on;
% 
% if norm(x(1:2)-goal')<150
%     heading = 1e8 * (180 - targetTheta2);
% else
    heading = 180 - targetTheta;
% end
 
end