% This script creates the coefficient matrices for the MiP
% simulink model as described in "mIpDerivations.pdf"
% mipParameters_rsw.m
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

% derived parameters from Numerical Renaissance (17.22)
% Bob's notes mIpDerivations.pdf
% These parameters go into the nonlinear Simulink MiP model

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

% Now set up the discrete-time controller

% Load the continuous-time and associated discrete-time linearized
% state-space models
% continuous-time linearized model [ALCmIP, BLCmIp, CLCmp, DLCmIp]
% discrete-time linearized model [ALDmIP, BLDmIp, CLDmp, DLDmIp]
load('mIpModel.mat')

% Load Neeraj's state estimate feedback controller parameters
% This defines his K and L matrices
load('MiP_control.mat')
Ts =1/200;
% Construct the digital controller matrices
% These appear in the controller present in the Simulink feedback path
KLDmIp=K; % Note that the minus sign is taken account in the feedback block
LLDmIp=L;
ALCBKLDmIp =ALDmIp-BLDmIp*KLDmIp-LLDmIp*CLDmIp;
