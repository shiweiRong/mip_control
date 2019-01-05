clc
clear
load('mIpModel.mat')
% raw physical parameters
g = 9.81;
Ib = 4e-4;
Iw = 1.2230e-4;
l = 0.036;
mb = 0.263;
mw = 0.027;
r = 0.034;
RL = 1500;
kvktL = 6.4697;
ktL = 32.0130;

Axx = Ib + mb*l*l;
Bxx = mb*g*l;
Cxx = mb*r*l;
Dxx = Iw+(mw+mb)*r*r;
Exx = Cxx;
Fxx = Cxx;

% highly derived quantities
BExx = Bxx*Exx;
AFxx = Axx*Fxx;
ADxx = Axx*Dxx;
CExx = Cxx*Exx;
CFxx = Cxx*Fxx;
BDxx = Bxx*Dxx;

Ts = 1/200;
% pole placement
eigABK = [0.0006
0.98 + 0.1i
0.98 - 0.1i
0.98
0.99
];
K = place(ALDmIp,BLDmIp,eigABK)
eigALC = [ 0.0006
0.8533
0.995
0.3821
0.95
];
L = place(ALDmIp',CLDmIp',eigALC)'

KLDmIp=K; % Note that the minus sign is taken account in the feedback block
LLDmIp=L;
ALCBKLDmIp =ALDmIp-BLDmIp*KLDmIp-LLDmIp*CLDmIp;

% subplot(3,1,1)
% plot(xhat.Time, xhat.Data);
% title('state estimates');
% legend('tau', 'theta', 'thetadot', 'phi', 'phidot');
% subplot(3,1,2)
% plot(x.Time, x.Data);
% title('states');
% legend('tau', 'theta', 'thetadot', 'phi', 'phidot');
% subplot(3,1,3)
% plot(xerr.Time, xerr.Data);
% title('state errors');
% legend('tau', 'theta', 'thetadot', 'phi', 'phidot');
% xlabel('time');
% 
bias = [0 5	10	15	20	30]';
theta_ss = [0 -0.0572	-0.1144	-0.1717	-0.2290	-0.3439]';
phi_ss = [0 7.4202 14.8455 22.2808 29.7314 44.7005]';
subplot(2,1,1)
plot(bias,theta_ss)
legend('theta ss');
title('The steady states of theta and phi under different wind biases');
subplot(2,1,2)
plot(bias,phi_ss)
xlabel('time');
legend('phi ss');

