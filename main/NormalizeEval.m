%% ��һ������ 
% ÿһ���켣�ĵ���÷ֳ��Ա������з�����
function EvalDB=NormalizeEval(EvalDB)
% ���ۺ�������
if sum(EvalDB(:,3))~= 0
    EvalDB(:,3) = EvalDB(:,3)/sum(EvalDB(:,3));  %���������  ���о����ÿԪ�طֱ���Ա����������ݵĺ�
end
if sum(EvalDB(:,4))~= 0
    EvalDB(:,4) = EvalDB(:,4)/sum(EvalDB(:,4));
end
if sum(EvalDB(:,5))~= 0
    EvalDB(:,5) = EvalDB(:,5)/sum(EvalDB(:,5));
end
end