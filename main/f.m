%% Motion Model ���ݵ�ǰ״̬������һ���������ڣ�dt����״̬
% u = [vt; wt];��ǰʱ�̵��ٶȡ����ٶ� x = ״̬[x(m),y(m),yaw(Rad),v(m/s),w(rad/s)]
function x = f(x, u)
global dt;
F = [1 0 0 0 0
     0 1 0 0 0
     0 0 1 0 0
     0 0 0 0 0
     0 0 0 0 0];
 
B = [dt*cos(x(3)) 0
    dt*sin(x(3)) 0
    0 dt
    1 0
    0 1];
 
x= F*x+B*u;  
end