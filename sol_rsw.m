% sole_rsw.m
%% Initialization
clear ; close all; clc

%% ==================== Part 1 ====================
% check re and ob of DT LTI
load('mIpModel.mat');

% generate discrete-time reachability matrix
Re = ctrb(ALDmIp, BLDmIp);
% Number of unreachable states
unco = length(ALDmIp) - rank(Re)
if unco == 0
    fprintf('The system is reachable.\n');
else
    fprintf('The system is not reachable.\n');
end

fprintf('Program paused. Press enter to continue.\n');
pause;

% generate discrete-time observability matrix
Ob = obsv(ALDmIp, CLDmIp);
% Number of unobservable states
unob = length(ALDmIp)-rank(Ob)
if unob == 0
    fprintf('The system is observable.\n');
else
    fprintf('The system is not observable.\n');
end

fprintf('Program paused. Press enter to continue.\n');
pause;

%% ==================== Part 2 ====================
% check the stability of A
lambdaA = eig(ALDmIp);
if max(abs(lambdaA)) >=1
    fprintf('A is not stable.\n');
else
    fprintf('A is stable.\n');
end

% pole placement
eigABK = [0.0006
0.98 + 0.1i
0.98 - 0.1i
0.98
0.99
];
K = place(ALDmIp,BLDmIp,eigABK)
eigALC = [0.0006
0.8533
0.999
0.3821
0.95
];
L = place(ALDmIp',CLDmIp',eigALC)'

fprintf('Program paused. Press enter to continue.\n');
pause;

%% ==================== Part 3 ====================
KLDmIp = K; % Note that the minus sign is taken account in the feedback block
LLDmIp = L;

% DT LTI system
% mIp
Ts = 1/200;
% plant
sysd1 = ss(ALDmIp, BLDmIp, CLDmIp, DLDmIp, Ts);
% controller
sysd2 = ss(ALDmIp-BLDmIp*KLDmIp-LLDmIp*CLDmIp, LLDmIp,...
            -KLDmIp, zeros(1,2), Ts);
sysd = feedback(sysd1, sysd2, +1);  

% simulate DT LTI model
[u,t] = gensig('sqaure',5, 10, Ts);
u = u*0; 

% zero input
x0 = [0 0.1 0 0.1 zeros(1, 6)]; % intial states
yd = lsim(sysd, u, t, x0);
figure
fprintf('figure 1: u = 0 (DT)\n');
plot(t, yd);
legend('theta', 'phi');
xlabel('time');
title('DT system (zero input)');

fprintf('Program paused. Press enter to continue.\n');
pause;

% sqaure signal input
[u,t] = gensig('sqaure',5, 10, Ts); 
x0 = zeros(1, 10);
yd = lsim(sysd, u, t, x0);
figure 
fprintf('figure 2 : u is square or sine signal. (DT)\n');
subplot(2,1,1)
plot(t, [u yd]);
legend('square signal', 'theta', 'phi');
title('DT system (square signals)');

% sine signal input
[u,t] = gensig('sin',5, 10, Ts); 
subplot(2,1,2)
plot(t, [u yd]);
legend('sine signals', 'theta', 'phi');
xlabel('time');
title('DT system (sine signals)');

fprintf('Program paused. Press enter to continue.\n');
pause;

% convert DT to CT
sysc = d2c(sysd, 'zoh');

% simulate CT LTI model
x0 = [0 0.1 0 0.1 zeros(1, 6)]; % intial states
[u,t] = gensig('sqaure',5, 10, Ts);
u = u*0; % zero input
yc = lsim(sysc, u, t, x0);
yd = lsim(sysd, u, t, x0);
figure
fprintf('figure 3 : u = 0 (CT).\n');
plot(t, [yc yd]);
legend( 'thetaDT', 'phiDT', 'thetaCT', 'phiCT');
xlabel('time');
title('CT system vs DT system (zero input)');

fprintf('Program paused. Press enter to continue.\n');
pause;

fprintf('Compare CT and DT systems.\n');
% compare CT and DT
fprintf('norm(yd-yc) \n');
norm(yd-yc)
