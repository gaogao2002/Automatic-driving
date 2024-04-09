function [sys,x0,str,ts] = MPC_car1(t,x,u,flag)
switch flag
  case 0
    [sys,x0,str,ts]=mdlInitializeSizes;
  case 2
    sys=mdlUpdate(t,x,u);
  case 3
    sys=mdlOutputs(t,x,u);
  case {1,4,9}
    sys=[];
  otherwise
    error.(['Unhandled flag=',num2str(flag)]);

end

function [sys,x0,str,ts]=mdlInitializeSizes
sizes = simsizes;
sizes.NumContStates  = 0;
sizes.NumDiscStates  = 3;
sizes.NumOutputs     = 2;
sizes.NumInputs      = 7;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;   % at least one sample time is needed
sys = simsizes(sizes);
x0  = [0, 0, 0.0367]';
str = [];
ts  = [0.01 0];
global U
U=[0.0;0.0];

function sys=mdlUpdate(t,x,u)

sys = x;

function sys=mdlOutputs(t,x,u)
%% 定义全局变量
global A1 B1 u_piao U kesi
Nx=3;%状�?�量3个，x，y，yaw
Nu=2;%控制�?2个v，前轮转�?
Np=120;%预测步长50
Nc=80;%控制步长30
Row=10;%松弛因子
T=0.01;%采用时间0.01
xo=u(1);
yo=u(2);
yaw=u(3);
x_ref=u(4);
y_ref=u(5);
yaw_ref=u(6);
vd1=u(7);%参数车�??
vd2=0.020779541831518;%参�?�前轮转�?
L=0.8;%轴距2.9m
kesi=zeros(Nx+Nu,1);
kesi(1)=xo-x_ref;
kesi(2)=yo-y_ref;
kesi(3)=yaw-yaw_ref;
kesi(4)=U(1);
kesi(5)=U(2);
Q=eye(Nx*Np);
% Q(1,1) = 5;
R=eye(Nc*Nu);
% R(1,1) = 5;
A1=[1 0 -vd1*T*sin(yaw);
    0 1 vd1*T*cos(yaw);
    0 0 1];
B1=[cos(yaw)*T 0;
    sin(yaw)*T 0;
    tan(vd2)*T/L vd1*T/L/(cos(vd2))^2];
A_cell=cell(2,2);
A_cell{1,1}=A1;
A_cell{1,2}=B1;
A_cell{2,1}=zeros(Nu,Nx);
A_cell{2,2}=eye(Nu);
A=cell2mat(A_cell);
B_cell=cell(2,1);
B_cell{1,1}=B1;
B_cell{2,1}=eye(Nu);
B=cell2mat(B_cell);
C_cell=cell(1,2);
C_cell{1,1}=eye(Nx);
C_cell{1,2}=zeros(Nx,Nu);
C=cell2mat(C_cell);
%% 预测时域矩阵升维
PHI_cell=cell(Np,1);
for i=1:1:Np
    PHI_cell{i,1}=C*A^i;
end
PHI=cell2mat(PHI_cell);
THETA_cell=cell(Np,Nc);
for i=1:1:Np
    for j=1:1:Nc
        if  i>=j
            THETA_cell{i,j}=C*A^(i-j)*B;
        else
            THETA_cell{i,j}=zeros(3,2);
        end
    end
end
THETA=cell2mat(THETA_cell);
H_cell=cell(2,2);
H_cell{1,1}=THETA'*Q*THETA+R;
H_cell{1,2}=zeros(Nc*Nu,1);
H_cell{2,1}=zeros(1,Nc*Nu);
H_cell{2,2}=Row;
H=cell2mat(H_cell);
error=PHI*kesi;
f_cell=cell(2,1);
f_cell{1,1}=2*error'*Q*THETA;
f_cell{1,2}=0;
f=cell2mat(f_cell);
A_t=zeros(Nc,Nc);%工具人矩�?
for i=1:1:Nc
    for j=1:1:Nc
        if i>=j
            A_t(i,j)=1;
        else
            A_t(i,j)=0;
        end
    end
end
A_I=kron(A_t,eye(Nu));
Ut=kron(ones(Nc,1),U);
umax=[0.2;0.44];
umin=[-0.;-0.44];
umax_dt=[0.01;0.01];
umin_dt=[-0.01;-0.01];
Umax=kron(ones(Nc,1),umax);
Umin=kron(ones(Nc,1),umin);
Umax_dt=kron(ones(Nc,1),umax_dt);
Umin_dt=kron(ones(Nc,1),umin_dt);
A_cons_cell=cell(2,2);
A_cons_cell{1,1}=A_I;
A_cons_cell{1,2}=zeros(Nu*Nc,1);
A_cons_cell{2,1}=-A_I;
A_cons_cell{2,2}=zeros(Nu*Nc,1);
A_cons=cell2mat(A_cons_cell);
B_cons_cell=cell(2,1);
B_cons_cell{1,1}=Umax-Ut;
B_cons_cell{2,1}=-Umin+Ut;
B_cons=cell2mat(B_cons_cell);
lb=[Umin_dt;0];
ub=[Umax_dt;10];
%% 二次规划问题
options=optimset('Algorithm','interior-point-convex');

[X,fval,exitflag] =quadprog(H,f,A_cons,B_cons,[],[],lb,ub,[],options);
%% 赋�?�输�?
u_piao(1)=X(1);
u_piao(2)=X(2);
U(1)=kesi(4)+u_piao(1);
U(2)=kesi(5)+u_piao(2);
u_real(1)=U(1)+vd1;
u_real(2)=U(2)+vd2;

sys = u_real;
