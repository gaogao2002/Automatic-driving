%% DWA�㷨ʵ�� 
% model  �������˶�ѧģ��  ����ٶ�m/s],�����ת�ٶ�[rad/s],���ٶ�[m/ss],��ת���ٶ�[rad/ss], �ٶȷֱ���[m/s],ת�ٷֱ���[rad/s]]
% �����������ǰ״̬��ģ�Ͳ�����Ŀ��㡢���ۺ����Ĳ������ϰ���λ�á��ϰ���뾶
% ���ز����������� u = [v(m/s),w(rad/s)] �� �켣���� N * 31  ��N�����õĹ켣����
% ѡȡ���Ų������������壺�ھֲ����������У�ʹ�û����˱ܿ��ϰ������Ŀ���ԽϿ���ٶ���ʻ��
function [u,trajDB] = DynamicWindowApproach(x,model,goal,evalParam,ob,R)
% Dynamic Window [vmin,vmax,wmin,wmax] ��С�ٶ� ����ٶ� ��С���ٶ� �����ٶ��ٶ�
% global path_last;
% global target_x;
% global target_y;
% global target_idx;

Vr = CalcDynamicWindow(x,model);  % ���ݵ�ǰ״̬ �� �˶�ģ�� ���㵱ǰ�Ĳ�������Χ



% plot(path_last(:,1), 600 - path_last(:,2));
% hold on;
% plot(x(1), x(2), 'ro');
% hold on;
% plot(path_last(target_idx,1), path_last(target_idx,2), 'gx');
% hold on;

% ���ۺ����ļ��� evalDB N*5  ÿ��һ����ò��� �ֱ�Ϊ �ٶȡ����ٶȡ�����÷֡�����÷֡��ٶȵ÷�
%               trajDB      ÿ5��һ���켣 ÿ���켣����״̬x�㴮���
[evalDB,trajDB]= Evaluation(x,Vr,goal,ob,R,model,evalParam);  %evalParam ���ۺ������� [heading,dist,velocity,predictDT]
 
if isempty(evalDB)
    disp('no path to goal!!');
    u=[0;0];return;
end
 
% �����ۺ�������
evalDB = NormalizeEval(evalDB);
 
% �������ۺ����ļ���
feval=[];
for id=1:length(evalDB(:,1))
    feval = [feval;evalParam(1:3)*evalDB(id,3:5)']; %�������ۺ������� ǰ�������������Ȩ�� ����ÿһ����õ�·��������Ϣ�ĵ÷�
end
evalDB = [evalDB feval]; % ���һ��

[maxv,ind] = max(feval);% ѡȡ������ߵĲ��� ��Ӧ�������ظ� maxv  ��Ӧ�±귵�ظ� ind
u = evalDB(ind,1:2)';% �������Ų������ٶȡ����ٶ�  
end