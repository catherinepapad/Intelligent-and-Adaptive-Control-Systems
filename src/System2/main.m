%% Intelligent & Adaptive Automatic Control Systems
% Papadopoulou Aikaterini - 10009 - aapapadop@ece.auth.gr


%% Setup
close all
clear
clc


%% System & Model Parameters
% System
C = 1;
M = 1;
G = 10;

A = [0 1 ; 0 -C/M];
B = [0 ; 1/M];
H = [0 ; -G/M];

f = @(x) sin(x);        % non linear part of the system

% Model
A_ref = [0 1 ; -1 -1.4];
B_ref = [0 ; 1];


%% Control Parameters
sign_l = 1;

gamma1 = 100;
gamma2 = 100;
gamma3 = 100;

a = 300;
Q = a*eye(2);

P = lyap(A_ref',Q);


%% Reference Signal
% u = @(t) double(t>=0);                            % unit step (rename from u to r to use as input)
% r = @(t) u(t-2)-u(t-8);                           % pulse (requires unit step function as u)
% r = @(t)  4 * cos(t) ;                            % cosine
r = @(t) 3 + 4 * cos(t) + 5 * sin(3*t);             % sum of sin and cos


%% Disturbances
amp = 100;                                                           % amplitude
h = @(t) double(t>=0);                                               % unit step
d = @(t) amp*(h(t-7) - h(t-12));                                     % pulse (requires unit step as h)


%% Simulation of the implemented control schema
tspan = 0:0.001:25;
x_init = zeros(8, 1);

mode = 0;               % Select mode: 0 for no disturbances, 1 for disturbances

odefun = @(t, x)derivatives(t,x,r,A,B,H,f,A_ref,B_ref,d,sign_l,gamma1,gamma2,gamma3,P,mode);
[t, x] = ode23(odefun, tspan, x_init);


%% Define output variables
x_1 = x(:,1);
x_2 = x(:,2);
x_1_ref = x(:,3);
x_2_ref = x(:,4);

error_x1 = x_1 - x_1_ref;
error_x2 = x_2 - x_2_ref;


%% Plots
figure
subplot(2,1,1)
plot(t,x_1)
hold on
plot(t,x_1_ref)
xlabel('Time [s]','interpreter','latex','FontSize',13)
legend('$x_1$','$x_{m,1}$','interpreter','latex')
title('Plot of state variable $x_1$','Interpreter', 'latex','FontSize',15)
subplot(2,1,2)
plot(t,x_2)
hold on
plot(t,x_2_ref)
xlabel('Time [s]','interpreter','latex','FontSize',13)
legend('$x_2$','$x_{m,2}$','interpreter','latex')
title('Plot of state variable $x_2$','Interpreter', 'latex','FontSize',15)
sgtitle({'System and Model State Variables';['$\gamma_1$ = ',num2str(gamma1),' , $\gamma_2$ = ' ...
    ,num2str(gamma2),' , $\gamma_3$ = ',num2str(gamma3), ' , $Q$ = ',num2str(a),'I']},'Interpreter', 'latex','FontSize',20)


figure
subplot(2,1,1)
plot(t,error_x1)
xlabel('Time [s]','interpreter','latex','FontSize',13)
title('Plot of error $e_1 = x_1 - x_{m,1}$','Interpreter', 'latex','FontSize',15)
subplot(2,1,2)
plot(t,error_x2)
xlabel('Time [s]','interpreter','latex','FontSize',13)
title('Plot of error $e_2 = x_2 - x_{m,2}$','Interpreter', 'latex','FontSize',15)
sgtitle({'Tracking Error';['$\gamma_1$ = ',num2str(gamma1),' , $\gamma_2$ = ' ...
    ,num2str(gamma2),' , $\gamma_3$ = ',num2str(gamma3), ' , $Q$ = ',num2str(a),'I']},'Interpreter', 'latex','FontSize',20)
