function sys_out = derivatives1(t,var,r,A,B,H,f,A_ref,B_ref,d,lambda0,p0,Gamma,mode)
disp(t)

% Define variables from stack
x_1 = var(1);
x_2 = var(2);
x_1_ref = var(3);
x_2_ref = var(4);
omega_1 = var(5);
omega_2 = var(6);
phi = [var(7) ; var(8) ; var(9) ; var(10)];
theta = [var(11) ; var(12) ; var(13) ; var(14)];

% Calculate state of system and model
x = [x_1 ; x_2];
y = x_1;

x_ref = [x_1_ref ; x_2_ref];
y_m = x_1_ref;

% Error
e = y - y_m;

% omega vector
omega = [omega_1 ; omega_2 ; y ; r(t)];

% Calculate derivates part 1
theta_dot = -Gamma * e * phi;
phi_dot = -p0 * phi + omega;

% Calculate control input
u = theta' * omega + theta_dot' * phi;

% Calculate derivates part 2
omega_1_dot = -lambda0 * omega_1 + u;
omega_2_dot = -lambda0 * omega_2 + y;
omega_dot = [omega_1_dot ; omega_2_dot];

if mode == 0
    x_dot = A*x + B*u + H*f(x_1);
else 
    x_dot = A*x + B*(u+d(t)) + H*f(x_1);
end
% x_dot = A*x + B*u + H*f(x_1);

x_dot_ref = A_ref*x_ref + B_ref*r(t);

sys_out = [x_dot ; x_dot_ref ; omega_dot ; phi_dot ; theta_dot];
end