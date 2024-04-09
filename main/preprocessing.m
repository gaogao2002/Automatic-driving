img = imread('sysu6001200.png');
% ��ͼƬת��Ϊ�Ҷ�ͼ��
gray_img = rgb2gray(img);
% ������ֵ��ֵ��Ϊ0��1
sysu = imbinarize(gray_img);
map_ori = sysu;

% figure;
% imagesc(map_ori);%imagesc ���Խ�����������ֵӳ�䵽��ɫͼ����
% colormap(flipud(gray));%colormap ������ֵӳ�䵽��ɫ�ռ��� colormap(flipud(gray)) ��������ɫӳ���Ϊ�Ҷ�ͼ��
% hold on;
% axis equal;
% axis off;
car_long=0.12;
car_wide=0.06;
resolution=0.01;
% coder.varsize( 'map' );
% map=expand_race_track(map,12);

tmpmap=shrink_race_track(map_ori,14);
% figure;
% imagesc(tmpmap);%imagesc ���Խ�����������ֵӳ�䵽��ɫͼ����
% colormap(flipud(gray));%colormap ������ֵӳ�䵽��ɫ�ռ��� colormap(flipud(gray)) ��������ɫӳ���Ϊ�Ҷ�ͼ��



[map,map_shrink]=gen_obstacle(tmpmap,map_ori,12,round(car_wide/resolution),20,5);%���ߣ�shrink���map,map,����ϰ���߳���Ӧ���ǳ��İ���߳���Ӧ�����������ȫ�ı߳�
% size(map)

figure;
imagesc(map);
hold on;
axis equal;
axis off;

figure;
imagesc(map_shrink);
hold on;
axis equal;
axis off;

obs = map - map_ori;
obs_shrink = obs + tmpmap;

out_obs_shrink=zeros(600,1200,3);%ȫ��
[row,col] = find(obs_shrink == 0);%��û��ռ�ݵ�
for i=1:length(row)
        out_obs_shrink(row(i),col(i),:)=[1,1,1];%��
end

out_=zeros(600,1200,3);%ȫ��
[row,col] = find(map == 0);%��û��ռ�ݵ�
for i=1:length(row)
        out_(row(i),col(i),:)=[1,1,1];%��
end

out_ori=zeros(600,1200,3);%ȫ��
[row,col] = find(map_ori == 0);%��û��ռ�ݵ�
for i=1:length(row)
        out_ori(row(i),col(i),:)=[1,1,1];%��
end

out_shrink=zeros(600,1200,3);%ȫ��
[row,col] = find(tmpmap == 0);%��û��ռ�ݵ�
for i=1:length(row)
        out_shrink(row(i),col(i),:)=[1,1,1];%��
end


% figure;
% imagesc(map_shrink);
% figure;
% imagesc(map);%imagesc ���Խ�����������ֵӳ�䵽��ɫͼ����
% colormap(flipud(gray));%colormap ������ֵӳ�䵽��ɫ�ռ��� colormap(flipud(gray)) ��������ɫӳ���Ϊ�Ҷ�ͼ��
% hold on;
% save('sysu_standard.mat','map','out');


% out_new = zeros(60, 120, 3);
% for i = 1:3
%     out_new(:,:,i) = out(1:10:end,1:10:end,i);
% end


imwrite(out_shrink,'map_shrink.png');
imwrite(out_ori,'map_ori.png');
imwrite(out_,'map_obs.png');
imwrite(out_obs_shrink,'map_obs_shrink.png');
save('map.mat', 'map');

% load('sysu_standard.mat');
% axis equal;
% axis off;
% figure;
% imagesc(map);%imagesc ���Խ�����������ֵӳ�䵽��ɫͼ����
% colormap(flipud(gray));%colormap ������ֵӳ�䵽��ɫ�ռ��� colormap(flipud(gray)) ��������ɫӳ���Ϊ�Ҷ�ͼ��
% axis equal;
% axis off;
% data=num2cell(map);
% size(data)

% dsm = get_param('Copy_of_car_sim/track', 'RuntimeObject');
% write(dsm, {data});

% track = timeseries(map);
% save('sysu2.mat','track', '-v7.3');


function [new_map, map_shrink]=gen_obstacle(tmpmap,map,obstacle_wide,car_wide,safe_distance,obstacle_num_max)
%����obstacle_wide+car_wide+safe_distance֮��û��occ
    obstacle_num=0;
    [row,col] = find(tmpmap == 0);
    while obstacle_num<obstacle_num_max
        idx = randperm(length(row), 1);
        if(col(idx)<100)%���������ڳ�ʼλ��
            continue;
        end
        safe_wide=obstacle_wide+car_wide+safe_distance;
        have_at_least_one_neigb_occ=0;%��һ����ռ��˵������ȫ���޷�����
           for l= row(idx)-safe_wide : row(idx)+safe_wide
               if (l>600)||(l<1)
                  continue;
               end
               for m= col(idx)-safe_wide : col(idx)+safe_wide
                   if (m>1200)||(m<1)
                      continue;
                   end
                   this_neighbor_occ=tmpmap(l,m);
                   if this_neighbor_occ==1
                       have_at_least_one_neigb_occ=1;
                       break;
                   end
               end
               if  have_at_least_one_neigb_occ==1
                   break;
               end
           end
           if have_at_least_one_neigb_occ==0
               for l= row(idx)-obstacle_wide :row(idx)+obstacle_wide
                   if (l>600)||(l<1)
                      continue;
                   end
                   for m= col(idx)-obstacle_wide : col(idx)+obstacle_wide
                       if (m>1200)||(m<1)
                          continue;
                       end
                       map(l,m)=1;
                       tmpmap(l,m)=1;
                   end
               end
                obstacle_num=obstacle_num+1;
               [row,col] = find(map == 0);
           end
    end
    new_map=map;
    map_shrink = tmpmap;
end
function new_map=expand_race_track(map,index)
%����˸�����դ����һ���Ǳ�ռ�ݣ���˵�����դ���ڱ�Ե,��������  %�ĸ�����Ҳ��������
    new_map=map;
    for i= 1: size(map,1) 
        for j=1:size(map,2)
           have_at_least_one_neigb_occ=0;
           this_grid_occ=map(i,j);
           if this_grid_occ==1
                continue;
           end
           for l= i-1 : i+1
               if (l>600)||(l<1)
                  continue;
               end
               for m= j-1 : j+1
                   if (m>1200)||(m<1)
                      continue;
                   end
                   this_neighbor_occ=map(l,m);
                   if this_neighbor_occ==1
                       have_at_least_one_neigb_occ=1;
                       break;
                   end
               end
               if  have_at_least_one_neigb_occ==1
                   break;
               end
           end
           %����have_at_least_one_neigb_occ��ֵ�����������
           if  have_at_least_one_neigb_occ==1 
               for l= i-index : i+index
                   if (l>600)||(l<1)
                      continue;
                   end
                   for m= j-index : j+index
                       if (m>1200)||(m<1)
                          continue;
                       end
                       new_map(l,m)=0;
                   end
               end
           end
        end
    end
end
function new_map=shrink_race_track(map,index)
%����˸�����դ����һ���Ǳ�ռ�ݣ���˵�����դ���ڱ�Ե,��������  %�ĸ�����Ҳ��������
    new_map=map;
    for i= 1: size(map,1) 
        for j=1:size(map,2)
           have_at_least_one_neigb_occ=0;
           this_grid_occ=map(i,j);
           if this_grid_occ==1
                continue;
           end
           for l= i-1 : i+1
               if (l>600)||(l<1)
                  continue;
               end
               for m= j-1 : j+1
                   if (m>1200)||(m<1)
                      continue;
                   end
                   this_neighbor_occ=map(l,m);
                   if this_neighbor_occ==1
                       have_at_least_one_neigb_occ=1;
                       break;
                   end
               end
               if  have_at_least_one_neigb_occ==1
                   break;
               end
           end
           %����have_at_least_one_neigb_occ��ֵ�����������
           if  have_at_least_one_neigb_occ==1 
               for l= i-index : i+index
                   if (l>600)||(l<1)
                      continue;
                   end
                   for m= j-index : j+index
                       if (m>1200)||(m<1)
                          continue;
                       end
                       new_map(l,m)=1;
                   end
               end
           end
        end
    end
end