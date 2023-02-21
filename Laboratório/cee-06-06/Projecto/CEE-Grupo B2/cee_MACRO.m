%% *Inicialização*
clear all
close all
clc;

%% 1
load('fp_lin_matrices_fit3.mat'); %%Load Matrices A, B, C, D
eigenvalues_A=eig(A)
% pólo no spcd, sistema instável em malha aberta...
%% 2
Co=ctrb(A,B);
rank_a=rank(A)
rank_co=rank(Co)
%% 3
C=[0 0 1 0 0;0 0 0 0 0];
O=obsv(A,C)
rank_co=rank(O)
C=[1 0 0 0 0;0 0 1 0 0];
O=obsv(A,C)
rank_co=rank(O)
%% 4
[num, den]=ss2tf(A,B,C,D);
bode(num,den);
%% 5
load('fp_lin_matrices_fit3.mat'); %%Load Matrices A, B, C, D

%vector de condições iniciais, em gruas, e respectiva conversão para rad
x0_graus = [5.7296 0 7 0 0]; %[5.7296 0 0 0 0];
x0c=degtorad(x0_graus)';
%assumindo acesso a todos os estados
C=eye(5);
D=[0 0 0 0 0]';
T=2; % Time duration of the simulation
%% INSERIR COEFICIENTES

Qr = diag([10,0,1,0,0]); %Weight Matrix for x in the integral
Rr = 1; %Weight for the input variable
K = lqr(A, B, Qr, Rr); %Calculate feedback gain
[Pr,Lr,Kr]=care(A,B,Qr,Rr); %redundante, obtenção da matriz P, pólos, ganhos...

%% AGREGADOS OS DADOS após 2º lab

x0_graus = [5.7296 0 3 0 0];%ALTERADO
x0=degtorad(x0_graus)';
C=eye(5); %matriz identidade 5x5
D=[0 0 0 0 0]';
T=2; % Time duration of the simulation

Qr = diag([1/degtorad(15)^2,0,1/degtorad(3)^2,0,0]); %Weight Matrix for x in the integral
Rr = 1/5^2; %Weight for the input variable
K = lqr(A, B, Qr, Rr); %Calculate feedback gain
[Pr,VP,Kr]=care(A,B,Qr,Rr); %redundante...


%% 7
T=2;
C=[1 0 0 0 0;0 0 1 0 0];

G = 1*eye(size(A)); %Gain of the process noise
Qe = 1*eye(size(A))*10; %Variance of process errors - POTENCIÓMETRO?
Re = 1*eye(2); %Variance of measurement errors - ENCODER?

L = lqe(A, G, C, Qe, Re); %Calculate estimator gains

% 8
% Ac=A-B*K-L*C;
% C=[1 0 0 0 0;0 0 1 0 0];
% Bc=L;
% Cc=-K;
% Dc=[0 0];
% D=[0;0];

%% 2 casos melhores da 2ª aula lab
x0c_graus = [5.7296 0 0 0 0];
x0c=deg2rad(x0c_graus)';
T=5

Qr = diag([1/(1.75^2),0,1/(0.035^2),0,0]); %Weight Matrix for x in the integral
Rr = 1/36; %Weight for the input variable
K = lqr(A, B, Qr, Rr); %Calculate feedback gain
% Simulate controller