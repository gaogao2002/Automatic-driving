%% ��������
%% ���Ҫ���Ը�����ͼ����Ҫע����δ��룬���Ұѵ�11�д���ע�͵�
% map_ori_shrink = imread("map_shrink.png");
% gray_map = rgb2gray(map_ori_shrink);
% im2map = imbinarize(gray_map);
% im2map = ~im2map;
% map_shrink = obs + im2map;
% map_shrink = ~map_shrink;
% map_shrink = im2bw(map_shrink);
%% 
map_shrink=im2bw(imread('map_obs_shrink.png')); 
source=[520 80]; %�������
goal=[150 1130]; %�յ�����
stepsize = 40;  % RRT��ÿ�������Ĳ���
threshold = 50; % �ڵ������ֵ��С�ڸþ��뼴����Ϊ��ͬ
maxFailedAttempts = 1000000;% ����Դ���
display = true; % RRT�㷨�Ŀ���
%%�ж������յ�����λ���Ƿ����ϰ���
if ~feasiblePoint(source,map_shrink), error('source lies on an obstacle or outside map'); end
if ~feasiblePoint(goal,map_shrink), error('goal lies on an obstacle or outside map'); end
if display,imshow(map_shrink);%���Ƶ�ͼ
end
hold on;
%���
plot(source(2),source(1),'o');
plot(goal(2),goal(1),'o');
tic;  % tic-toc: Functions for Elapsed Time
% RRT�㷨�����岿��
RRTree = double([source -1]);%��������� 
i = 1;
pathFound = false;
lowestCost=1000;pl=zeros(1,maxFailedAttempts);
while i <= maxFailedAttempts  
    pl(i)=lowestCost;
    i=i+1;
    
    %���ݸ���������ɵ�
    if rand < 0.5
        x_rand = rand(1,2) .* size(map_shrink);   % ����������������
    else
        x_rand = goal; % �����յ�
    end
	% �ҵ�����������������������̵Ľڵ�
    [~, I] = min( distanceCost(RRTree(:,1:2),x_rand) ,[],1); 
    x_nearest = RRTree(I(1),1:2);
    % �������ɵĵ��λ��
    thedis = atan2(x_rand(1)-x_nearest(1),x_rand(2)-x_nearest(2));  
    newPoint = double(int32(x_nearest(1:2) + stepsize * [sin(thedis)  cos(thedis)]));
    % �ж���չ���Ƿ���У����Ƿ���map����
    if ~checkPath(x_nearest(1:2), newPoint, map_shrink) 
        i = i + 1;
        continue;
    end
	%�����յ�
    if distanceCost(newPoint,goal) < threshold, pathFound = true; break; end 
	%���²����Ľڵ� newPoint �����Զ���İ뾶��Χ��Ѱ�ҡ����ڡ�,����Ϊ�滻newPoint���ڵ�ı�ѡ
    x_near_list = NearestVertices(newPoint,RRTree);
    %��û���ڽ��Ľڵ�ʱ��ȡ������о�����̵Ľڵ�
    if  size(x_near_list)==[0,0]
        [~, I1]=min(distanceCost(RRTree(:,1:2),newPoint) ,[],1); 
        x_near_list = [RRTree(I1,1:3),I1];
    end
    %���μ��㡰���ڡ��ڵ㵽����·�����ۼ��� newPoint ��ÿ�������ڡ���·������
     sortlist=GetList(newPoint,x_near_list,RRTree);
     %�� x_near_list ѡ��x_min��Ϊ���ڵ�
    x_min=ChooseBestParent(sortlist,map_shrink);
    if ~size(x_min)==[0 0] 
        %�����µĽڵ�
        RRTree=[RRTree;[newPoint(1:2),x_min(4)]];
       %��������
        RRTree=RewireVertices(x_min,newPoint,sortlist,RRTree,map_shrink);
        % ����·��
        if display
            plot([x_min(2);newPoint(2)],[x_min(1);newPoint(1)],'LineWidth',1);
        end
        if AllCost(RRTree,RRTree(end,:))+distanceCost(RRTree(end,1:2),goal)<lowestCost && distanceCost(newPoint,goal)<thedis
            lowestCost=AllCost(RRTree,RRTree(end,:))+distanceCost(RRTree(end,1:2),goal);
            tree1=RRTree;
            pl(i)=lowestCost;
        end
    else
        continue;
    end
   
end

if display && pathFound,plot([x_min(2);goal(2)],[x_min(1);goal(1)],'LineWidth',1);%������ʾ�ٶ�
end

if display, disp('click/press any key'); waitforbuttonpress; end
if ~pathFound, error('no path found. maximum attempts reached'); end

%% ��������������յ㿪ʼ�����ҵ����·��
path = [goal];
prev = I(1);
while prev > 0
    path = [RRTree(prev,1:2); path];
    prev = RRTree(prev,3);
end

pathLength = 0;
% ����·������
for i=1:length(path)-1, pathLength = pathLength + distanceCost(path(i,1:2),path(i+1,1:2)); end 
fprintf('processing time=%d \nPath Length=%d \n\n', toc, pathLength); 
imshow(map_shrink);
plot(path(:,2),path(:,1),'LineWidth',2,'color','r');
path_last = [path(:,2),600-path(:,1)];

%% B spline smooth

figure;
map_ori=im2bw(imread('map_ori.png')); 
imagesc(map_ori);
hold on;
axis equal;
p =[path(:,2), path(:,1)];    %���ٵ�����㣬��Ϊ�������Ĵ�����
% plot(p(:,1),p(:,2),'k-o');  

%���ξ���b����
re2=[path(1,2), path(1,1)];
for i=1:length(p)-2  
    for t=0:0.01:1   
        
        b0 = 1/2*(1-t)^2;
        b1 = 1/2*(-2*t^2+2*t+1);
        b2 = 1/2*t^2;
        
        x=b0*p(i,1)+b1*p(i+1,1)+b2*p(i+2,1);
        y=b0*p(i,2)+b1*p(i+1,2)+b2*p(i+2,2);        
        
        re2=[re2;x y];
    end
end
re2=[re2;path(end,2) path(end,1)];

% %���ξ���b����
% re3=[];
% for i=1:length(p)-3  
%     for t=0:0.01:1   
%   
%         b0=1/6*(1-t)^3;                       
%         b1=1/6*(3.*t^3-6*t^2+4);        
%         b2=1/6*(-3*t^3+3*t^2+3*t+1);     
%         b3=1/6*t^3;                     
%   
%         x=b0*p(i,1)+b1*p(i+1,1)+b2*p(i+2,1)+b3*p(i+3,1);    
%         y=b0*p(i,2)+b1*p(i+1,2)+b2*p(i+2,2)+b3*p(i+3,2);     
% 
%         re3=[re3;x y];
%     end
% end
% 
% %�Ĵξ���b����
% re4=[];
% for i=1:length(p)-4  
%     for t=0:0.01:1   
%   
%         b0=1/24*(t^4-4*t^3+6*t^2-4*t+1);                     
%         b1=1/24*(-4*t^4+12*t^3-6*t^2-12*t+11);      
%         b2=1/24*(6*t^4-12*t^3-6*t^2+12*t+11);    
%         b3=1/24*(-4*t^4+4*t^3+6*t^2+4*t+1);                     
%         b4=1/24*t^4;
%         
%         x=b0*p(i,1)+b1*p(i+1,1)+b2*p(i+2,1)+b3*p(i+3,1)+b4*p(i+4,1);    
%         y=b0*p(i,2)+b1*p(i+1,2)+b2*p(i+2,2)+b3*p(i+3,2)+b4*p(i+4,2);     
% 
%         re4=[re4;x y];
%     end
% end

hold on;  
plot(re2(:,1),re2(:,2),'r');

path_last = re2;
save('path_last.mat', 'path_last');
save('path.mat', 'path');
% plot(re3(:,1),re3(:,2),'g');
% plot(re4(:,1),re4(:,2),'b');
