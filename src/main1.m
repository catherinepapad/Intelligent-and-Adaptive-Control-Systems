%% Intelligent & Adaptive Automatic Control Systems
% Papadopoulou Aikaterini - 10009 - aapapadop@ece.auth.gr


%% Setup & Input Data
close all
clear
clc


%% System & Model Parameters
% System
C = 1;
M = 1;
G = 10;

A = [0 1 ; -G/M -C/M];
B = [0 ; 1/M];
H = [0 ; -G/M];

f = @(x) sin(x);        % non linear part of the system

% Model
A_ref = [0 1 ; -1 -1.4];
B_ref = [0 ; 1];


%% Control Parameters
lambda0 = 0.001;
p0 = 0.001;
gamma = 500;
g = [gamma gamma gamma gamma];

Gamma = diag(g);


%% Reference Signal
% u = @(t) double(t>=0);                            % unit step (rename from u to r to use as input)
% r = @(t) u(t-2)-u(t-8);                           % pulse (requires unit step as u)
% r = @(t)  4 * cos(t) ;                            % cosine
r = @(t) 3 + 4 * cos(t) + 5 * sin(3*t);             % sum of sin and cos


%% Disturbances
amp = 500;                                                            % amplitude                                                                  
h = @(t) double(t>=0);                                                 % unit step
d = @(t) amp*(h(t-7) - h(t-12));                                       % pulse (requires unit step as h)


%% Simulation of the implemented control schema
tspan = 0:0.001:25;
x_init = zeros(14, 1);

mode = 0;               % Select mode: 0 for no disturbances, 1 for disturbances 

odefun = @(t, x)derivatives1(t,x,r,A,B,H,f,A_ref,B_ref,d,lambda0,p0,Gamma,mode);
[t, x] = ode23(odefun, tspan, x_init);


%% Define output variables
x_1_ref = x(:,3);
x_2_ref = x(:,4);
x_1 = x(:,1);
x_2 = x(:,2);
error = x_1 - x_1_ref;


%% Plots
figure
plot(t,x_1)
hold on
plot(t,x_1_ref)
xlabel('Time [s]','interpreter','latex','FontSize',13)
legend('$y$','$y_{m}$','interpreter','latex')
title({'System and Model Output';['$\lambda_0$ = ',num2str(lambda0),' , $p_0$ = ' ...
    ,num2str(p0),' , $\Gamma$ = diag\{$\gamma$\} , $\gamma$ = ',num2str(gamma)]},'Interpreter', 'latex','FontSize',15)


figure
plot(t,error)
xlabel('Time [s]','interpreter','latex','FontSize',13)
title({'Plot of output tracking error $e = y - y_{m}$';['$\lambda_0$ = ',num2str(lambda0),' , $p_0$ = ' ...
    ,num2str(p0),' , $\Gamma$ = diag\{$\gamma$\} , $\gamma$ = ',num2str(gamma)]},'Interpreter', 'latex','FontSize',15)