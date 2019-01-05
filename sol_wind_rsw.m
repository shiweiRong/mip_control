clc
clear
load('mIpModel.mat')
%% ==================== Part 1 ====================
Ap = [zeros(1,6); zeros(5,1) ALCmIp]; Ap(4, 1) = 100;
Bp = [0; BLCmIp];
Cp = [zeros(2, 1) CLCmIp];
Dp = zeros(2, 1);

Re = ctrb(Ap, Bp);
rank(Re) % not reachable
Ob = obsv(Ap, Cp);
rank(Ob) % but observable
eig(Ap)
% % observable from phi alone 
% Cp = Cp(2, :);
% Dp = Dp(2, :);
% Ob = obsv(Ap, Cp);
% rank(Ob) 
%% ==================== Part 2 ====================
Ts = 1/200;
sysc = ss(Ap, Bp, Cp, Dp); % CT
sysd = c2d(sysc, Ts); % DT

% plant of DT
Ad = sysd.A;
Bd = sysd.B;
Cd = sysd.C;
Dd = sysd.D;

K = [-0.5484  -10.5420   -0.4730   -0.4594   -0.5523];
Kd = [0 K];
eig(Ad-Bd*Kd);

eigALC = [0.0006
0.8533
0.999
0.3821
0.95
0.5 %%%
];
Ld = place(Ad',Cd',eigALC)';

%% ==================== Part 3 ====================

% controller
Kd(1,1) = -10; %% modify Kd
Ac = Ad - Ld*Cd -Bd*Kd;
Bc = Ld;
Cc = [-Kd; eye(6,6)];
Dc = zeros(7, 2);
controller = ss(Ac,Bc,Cc,Dc,Ts);

% observer
Ao = Ad - Ld*Cd;
Bo = [Bd Ld];
Co = eye(6);
Do = zeros(6,3);
observer = ss(Ao,Bo,Co,Do,Ts);

%% ==================== Part 4 ====================
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

% wind = x.Data(:,1);
% theta = x.Data(:,3);
% phi = x.Data(:,5);
% voltage = u.Data;
% subplot(2,1,1)
% plot(x.Time, [wind theta phi]);
% title('states');
% legend('wind','theta', 'phi');
% xlabel('time');
% % axis([0 30 -2 80])
% 
% subplot(2,1,2)
% plot(u.Time, u.Data)
% xlabel('time');
% legend('voltage');

