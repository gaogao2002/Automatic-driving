load sysu_standard_new.mat;

out_=zeros(600,1200,3);%全黑
[row,col] = find(map == 0);%找没被占据的
for i=1:length(row)
        out_(row(i),col(i),:)=[1,1,1];%白
end