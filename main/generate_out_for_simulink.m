load sysu_standard_new.mat;

out_=zeros(600,1200,3);%ȫ��
[row,col] = find(map == 0);%��û��ռ�ݵ�
for i=1:length(row)
        out_(row(i),col(i),:)=[1,1,1];%��
end