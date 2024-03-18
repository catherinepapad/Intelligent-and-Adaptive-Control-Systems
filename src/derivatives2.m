function sys_out = derivatives2(t,var,r,A,B,H,f,A_ref,B_ref,d,sign_l,gamma1,gamma2,gamma3,P,mode)
disp(t)

% Define variables from stack
x_1 = var(1);
x_2 = var(2);
x_1_ref = var(3);
x_2_ref = var(4);
K = [var(5) var(6)];
L = var(7);
M = var(8);

% Calculate state of system and model
x = [x_1 ; x_2];

x_ref = [x_1_ref ; x_2_ref];


% error
e = x - x_ref;

% Calculate control input
u = -K*x - L*r(t) - M*f(x_1);

% Calculate derivates 
if mode == 0
    x_dot = A*x + B*u + H*f(x_1);
else 
    x_dot = A*x + B*(u+d(t)) + H*f(x_1);
end
% x_dot = A*x + B*u + H*f(x_1);

x_dot_ref = A_ref*x_ref + B_ref*r(t);

K_dot = gamma1 * B_ref' * P * e * x' * sign_l;
L_dot = gamma2 * B_ref' * P * e * r(t) * sign_l;
M_dot = gamma3 * B_ref' * P * e * f(x_1) * sign_l;

sys_out = [x_dot ; x_dot_ref ; K_dot' ; L_dot ; M_dot];
end