% m1 = 2500;
% m2 = 320;
% k1 = 80000;
% k2 = 500000;
% b1 = 350;
% b2 = 15020;
% 
% nump=[(m1+m2) b2 k2];
% denp=[(m1*m2) (m1*(b1+b2))+(m2*b1) (m1*(k1+k2))+(m2*k1)+(b1*b2) (b1*k2)+(b2*k1) k1*k2];
% G1=tf(nump,denp);
% 
% num1=[-(m1*b2) -(m1*k2) 0 0];
% den1=[(m1*m2) (m1*(b1+b2))+(m2*b1) (m1*(k1+k2))+(m2*k1)+(b1*b2) (b1*k2)+(b2*k1) k1*k2];
% G2=tf(num1,den1);
% 
% numf=num1;
% denf=nump;
% F=tf(numf,denf);
% 
% Kd = 208025;
% Kp = 832100;
% Ki = 624075;
% C = pid(Kp,Ki,Kd);
% 
% sys_cl=F*feedback(G1,C);
% 
% t=0:0.05:5;
% step(0.1*sys_cl,t)
% title('Response to a 0.1-m Step under PID Control')

m1 = 2500;
m2 = 320;
k1 = 80000;
k2 = 500000;
b1 = 350;
b2 = 15020;

Aa=[0                 1   0                                              0         0
   -(b1*b2)/(m1*m2)   0   ((b1/m1)*((b1/m1)+(b1/m2)+(b2/m2)))-(k1/m1)   -(b1/m1)   0
    b2/m2             0  -((b1/m1)+(b1/m2)+(b2/m2))                      1         0
    k2/m2             0  -((k1/m1)+(k1/m2)+(k2/m2))                      0         0
    0                 0   1                                              0         0];
Ba=[0                 0
    1/m1              (b1*b2)/(m1*m2)
    0                -(b2/m2)
    (1/m1)+(1/m2)    -(k2/m2)
    0                 0];
Ca=[0 0 1 0 0];
Da=[0 0];
sys=ss(Aa,Ba,Ca,Da);

K = [0 2.3e6 5e8 0 8e6];

t = 0:0.01:2;
sys_cl = ss(Aa-Ba(:,1)*K,-0.1*Ba,Ca,Da);
step(sys_cl*[0;1],t)
title('Closed-Loop Response to a 0.1-m Step')