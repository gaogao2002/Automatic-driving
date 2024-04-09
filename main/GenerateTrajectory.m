%% �����켣���ɡ��켣���ݺ���
% ��������� ��ǰ״̬��vt��ǰ�ٶȡ�ot���ٶȡ�evaldt ǰ��ģ��ʱ�䡢������ģ�Ͳ�����û�õ���
% ���ز���; 
%           x   : ������ģ��ʱ������ǰ�˶� Ԥ����յ�λ��(״̬); 
%           traj: ��ǰʱ�� �� Ԥ��ʱ��֮�� �����е�λ�˼�¼��״̬��¼�� ��ǰģ��Ĺ켣  
%                  �켣��ĸ���Ϊ evaldt / dt + 1 = 3.0 / 0.1 + 1 = 31
%           
function [x,traj] = GenerateTrajectory(x,vt,ot,evaldt,model)
global dt;
time = 0;
u = [vt;ot];% ����ֵ
traj = x;   % �����˹켣
while time <= evaldt   
    time = time+dt; % ʱ�����
    x = f(x,u);     % �˶����� ǰ��ģ��ʱ���� �ٶȡ����ٶȺ㶨
    traj = [traj x]; % ÿһ�д���һ���켣�� һ��һ�е����
end
end