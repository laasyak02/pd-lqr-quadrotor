function [F, M, trpy, drpy] = lqr_controller(qd, t, qn, params, trajhandle)
% CONTROLLER quadrotor controller
% The current states are:
% qd{qn}.pos, qd{qn}.vel, qd{qn}.euler = [roll;pitch;yaw], qd{qn}.omega
% The desired states are:
% qd{qn}.pos_des, qd{qn}.vel_des, qd{qn}.acc_des, qd{qn}.yaw_des, qd{qn}.yawdot_des
% Using these current and desired states, you have to compute the desired controls

% =================== Your code goes here ===================
persistent gd;
persistent icnt;
 if isempty(gd)
     gd = zeros(0,3);
     icnt = 0;
 end
 icnt = icnt + 1;

%% Parameter Initialization
if ~isempty(t)
desired_state = trajhandle(t, qn);
end
%disp(desired_state)
F = 0;
M = [0;0;0];

m = params.mass;
g = params.grav;
I = params.I;

%Current state
r = qd{qn}.pos;
E = qd{qn}.euler;
r_dot = qd{qn}.vel;
w = qd{qn}.omega;

X_current = [r; E; r_dot; w];

%Desired state
r_des = desired_state.pos;
r_dot_des = desired_state.vel;
E3_des = desired_state.yaw;

r_acc_des = desired_state.acc;
E1_des = (1/g)*(r_acc_des(1)*sin(E3_des)-r_acc_des(2)*cos(E3_des));
E2_des = (1/g)*(r_acc_des(1)*cos(E3_des)+r_acc_des(2)*sin(E3_des));
E_des = [E1_des; E2_des; E3_des];

w3_des = desired_state.yawdot;
w_des = [0; 0; w3_des];

% epsilon = 1e-6;

X_des = [r_des; E_des; r_dot_des; w_des];
u_des = [m*g; M];

% X_des = epsilon * ones(12, 1);
% u_des = epsilon * ones(4, 1);

%disp(X)
% disp(X_des)
% disp(u_des)

p = w_des(1);
q = w_des(2);
r = w_des(3);

phi = E_des(1);
theta = E_des(2);
psi = E_des(3);

% syms rx ry rz phi theta psi1 r_dot_x r_dot_y r_dot_z p q r u1 u2x u2y u2z
% 
% X = [rx; ry; rz; phi; theta; psi1; r_dot_x; r_dot_y; r_dot_z; p; q; r];
% u = [u1; u2x; u2y; u2z];
% C = [cos(theta), 0, -cos(phi)*sin(theta); 
%     0, 1, sin(phi);
%     sin(theta), 0, cos(phi)*cos(theta)];
% C_inv = inv(C);
% disp(C_inv);
% w_omega = [p; q; r];
% % E_dot = C_inv*w_omega;
% E_dot = C \ [p; q; r];
% R = [cos(psi1)*cos(theta)-sin(phi)*sin(psi1)*sin(theta), -cos(phi)*sin(psi1), cos(psi1)*sin(theta)+cos(theta)*sin(phi)*sin(psi1);
%     cos(theta)*sin(psi1)+cos(psi1)*sin(phi)*sin(theta), cos(phi)*cos(psi1), sin(psi1)*sin(theta)-cos(psi1)*cos(theta)*sin(phi);
%     -cos(phi)*sin(theta), sin(phi), cos(phi)*cos(theta)];
% 
% r_dot_dot = [0; 0; -g]+(R/m)*[0; 0; u1];
% w_dot = inv(I)*([u2x; u2y; u2z] - (cross(w_omega, I*w_omega)));
% f = [r_dot_x; r_dot_y; r_dot_z; E_dot(1); E_dot(2); E_dot(3); r_dot_dot(1); r_dot_dot(2); r_dot_dot(3); w_dot(1); w_dot(2); w_dot(3)];
% 
% A_param = jacobian(f, X);
% disp(A_param)
% B_param = jacobian(f, u);
% % disp(B_param);
% X_values = {rx, ry, rz, phi, theta, psi1, r_dot_x, r_dot_y, r_dot_z, p, q, r};
% % X_subs = subs(X, X_values', num2cell(X_des));
% u_values = {u1, u2x, u2y, u2z};
% % u_subs = subs(u, u_values', num2cell(u_des));
% A = subs(A_param, [X_values, u_values]', [num2cell(X_des); num2cell(u_des)]);
% B = subs(B_param, [X_values, u_values]', [num2cell(X_des); num2cell(u_des)]);
% % disp(A);
% % disp(B);
% A=double(A);
% B=double(B);
% % disp(A);
% % disp(B);

A = zeros(12);
% disp(A);
A(1,7) = 1;
A(2,8) = 1;
A(3,9) = 1;
A(4,5) = -p*sin(theta)+r*cos(theta);
A(4,10) = cos(theta);
A(4,12) = sin(theta);
A(5,4) =  (1 / (cos(theta)^2))*(p*sin(theta)-r*cos(theta));
A(5,5) = tan(phi)*(p*cos(theta)+r*sin(theta));
A(5,10) = sin(theta)*tan(phi);
A(5,11) = 1;
A(5,12) = -cos(theta)*tan(phi);
A(6,4) = tan(phi)*(1 / (cos(phi)))*(r*cos(theta)-p*sin(theta));
A(6,5) = (1 / (cos(phi)))*(-p*cos(theta)-r*sin(theta));
A(6,10) = -sin(theta)/cos(phi);
A(6,12) = cos(theta)/cos(phi);
A(7,4) = g*sin(psi);
A(7,5) = g*cos(psi);
A(8,4) = -g*cos(psi);
A(8,5) = g*sin(psi);

B = zeros(12,4);
B(9,1) = 1/m;
B(10,2) = 1/I(1,1);
B(11,3) = 1/I(2,2);
B(12,4) = 1/I(3,3);

disp(A);
disp(B);

Q = 100 * eye(12);
R = 1 * eye(4);
[K S] = lqr(A, B, Q, R);
% disp(K);
u = K*X_current - u_des;
% disp(u);
% u = -u;
% disp(u);
F = u(1);
M = [u(2); u(3); u(4)];

% F = 0.3446;
% M = [0.1194;-0.0198;-0.0072];
% M = 1.0e-04*M;
% 
disp(X_current);
disp(X_des);
disp(F);
disp(M);
%Output trpy and drpy as in hardware
trpy = [0, 0, 0, 0];
drpy = [0, 0,       0,         0];

end
