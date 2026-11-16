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

F = 0;
M = [0;0;0];

m = params.mass;
g = params.grav;
I = params.I;

%Current state
r_val = qd{qn}.pos;
E = qd{qn}.euler;
r_dot = qd{qn}.vel;
w = qd{qn}.omega;

X_current = [r_val; E; r_dot; w];

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


X_des = [r_des; E_des; r_dot_des; w_des];
u_des = [m*g; M];


p_val = w_des(1);
q_val = w_des(2);
r_val = w_des(3);

phi_val = E_des(1);
theta_val = E_des(2);
psi_val = E_des(3);

%Defining A
A = zeros(12);
A(1,7) = 1;
A(2,8) = 1;
A(3,9) = 1;
A(4,5) = -p_val*sin(theta_val)+r_val*cos(theta_val);
A(4,10) = cos(theta_val);
A(4,12) = sin(theta_val);
A(5,4) =  (sec(phi_val)^2)*(p_val*sin(theta_val)-r_val*cos(theta_val));
A(5,5) = tan(phi_val)*(p_val*cos(theta_val)+r_val*sin(theta_val));
A(5,10) = sin(theta_val)*tan(phi_val);
A(5,11) = 1;
A(5,12) = -cos(theta_val)*tan(phi_val);
A(6,4) = tan(phi_val)*sec(phi_val)*(r_val*cos(theta_val)-p_val*sin(theta_val));
A(6,5) = sec(phi_val)*(-p_val*cos(theta_val)-r_val*sin(theta_val));
A(6,10) = -sin(theta_val)/cos(phi_val);
A(6,12) = cos(theta_val)/cos(phi_val);
A(7,4) = g*sin(psi_val);
A(7,5) = g*cos(psi_val);
A(8,4) = -g*cos(psi_val);
A(8,5) = g*sin(psi_val);

%Defining B
B = zeros(12,4);
B(9,1) = 1/m;
B(10,2) = 1/I(1,1);
B(11,3) = 1/I(2,2);
B(12,4) = 1/I(3,3);

%Defining Q
Q = diag([10, 10, 10, 1, 1, 10, 10, 10, 10, 0.1, 0.1, 0.1]);

%Defining R
R = diag([0.1, 1, 1, 1]);

%Obtaining K
[K S] = lqr(A, B, Q, R);

%Calculating u
u = K*(X_des-X_current) + u_des;

F = u(1);
M = [u(2); u(3); u(4)];

% disp(F);
% disp(M);
%Output trpy and drpy as in hardware
trpy = [0, 0, 0, 0];
drpy = [0, 0,       0,         0];

end
