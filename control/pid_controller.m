function [F, M, trpy, drpy] = pid_controller(qd, t, qn, params)
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
 
% =================== Your code starts here ===================
%% Parameter Initialization
r = qd{qn}.pos;
r_dot = qd{qn}.vel;
euler = qd{qn}.euler;
r_T = qd{qn}.pos_des;
r_dot_T = qd{qn}.vel_des;
r_acc_T = qd{qn}.acc_des;
yaw_des = qd{qn}.yaw_des;
r_des = qd{qn}.yawdot_des;
omega = qd{qn}.omega;
m = params.mass;
g = params.grav;
I = params.I;

%Position Control
kp = [11;11;12]; 
kd = [10.4;10.4;10]; 

%Hover
ep = r_T - r;
ev = r_dot_T - r_dot;

r_acc_des = r_acc_T + kd.*ev + kp.*ep;

u1 = m*g + m*r_acc_des(3);
F = u1;

%Attitude Control
roll_des = (1/g)*(r_acc_des(1)*sin(yaw_des)-r_acc_des(2)*cos(yaw_des));
pitch_des = (1/g)*(r_acc_des(1)*cos(yaw_des)+r_acc_des(2)*sin(yaw_des));

euler_des = [roll_des;pitch_des;yaw_des];
omega_des = [0;0;r_des];

kpa = [74;74;20];
kda = [58;58;15]; 
euler_error = euler_des - euler;

omega_error = omega_des - omega;

u2 = I*(kpa.*euler_error + kda.*omega_error);
M = u2;

% =================== Your code ends here ===================

%Output trpy and drpy as in hardware
trpy = [0, 0, 0, 0];
drpy = [0, 0, 0, 0];

end
