%% �����ƶ����� 
%�����˶�ѧģ�ͼ����ƶ�����, Ҳ���Կ��ǳ���һ�ζ�Բ�����ۻ� �򻯿��Ե�һ�ζ�Сֱ�ߵ��ۻ�
function stopDist = CalcBreakingDist(vel,model)
global dt;
MD_ACC   = 3;% 
stopDist=0;
while vel>0   %�������ٶȵ������� �ٶȼ���0���ߵľ���
    stopDist = stopDist + vel*dt;% �ƶ�����ļ��� 
    vel = vel - model(MD_ACC)*dt;% 
end
end