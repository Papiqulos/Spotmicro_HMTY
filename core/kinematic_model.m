clc;
clear;
close all;

syms theta1 theta2 theta3
syms a alpha d theta

%% Direct Kine% DH parameters: [a, alpha, d, theta]
r_legs = [0  pi/2 0 theta1;
          11 0    5 theta2;
          13 0    0 theta3];

l_legs = [0  pi/2 0 theta1;
          11 0   -5 theta2;
          13 0    0 theta3];

% Frame-to-frame DH transform
T_i = [cos(theta) -sin(theta)*cos(alpha)  sin(theta)*sin(alpha) a*cos(theta);
       sin(theta)  cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta);
       0           sin(alpha)             cos(alpha)            d;
       0           0                      0                     1];

T01    = subs(T_i, [a alpha d theta], [0  pi/2  0 theta1]);
T12_r  = subs(T_i, [a alpha d theta], [11 0     5 theta2]);
T12_l  = subs(T_i, [a alpha d theta], [11 0    -5 theta2]);
T23    = subs(T_i, [a alpha d theta], [13 0     0 theta3]);

% Base frame reorientation: end-effector x-axis points forward
Tbaseframe = [0 0 1 0;
              0 1 0 0;
             -1 0 0 0;
              0 0 0 1];

% Accumulated transforms
T_01    = Tbaseframe * T01;
T_012_r = Tbaseframe * T01 * T12_r;
T_012_l = Tbaseframe * T01 * T12_l;

% Full chain to end-effector
T_r = T_012_r * T23;
T_l = T_012_l * T23;

% Neutral position check
T_neutral_r = subs(T_r, [theta1 theta2 theta3], [0 0 0]);
T_neutral_l = subs(T_l, [theta1 theta2 theta3], [0 0 0]);

%% Differential Kinematics — Geometric Jacobian

% Joint rotation axes (z-column of accumulated transform up to that frame)
% z_i is the rotation axis of joint i+1 expressed in the base frame
z0   = Tbaseframe(1:3, 3);    % axis of joint 1 — z of base frame
z1   = T_01(1:3, 3);          % axis of joint 2 — z of frame 1
z2_r = T_012_r(1:3, 3);       % axis of joint 3 — z of frame 2 (right)
z2_l = T_012_l(1:3, 3);       % axis of joint 3 — z of frame 2 (left)

% Joint origins (4th column of accumulated transform = position in base frame)
p0   = Tbaseframe(1:3, 4);    % origin of joint 1 = [0;0;0]
p1   = T_01(1:3, 4);          % origin of joint 2
p2_r = T_012_r(1:3, 4);       % origin of joint 3 (right)
p2_l = T_012_l(1:3, 4);       % origin of joint 3 (left)

% End-effector positions
pe_r = T_r(1:3, 4);
pe_l = T_l(1:3, 4);

% Geometric Jacobian: J = [z_{i-1} x (pe - p_{i-1}); z_{i-1}]
J_r = [cross(z0, (pe_r - p0))   cross(z1, (pe_r - p1))   cross(z2_r, (pe_r - p2_r));
       z0                        z1                        z2_r];

J_l = [cross(z0, (pe_l - p0))   cross(z1, (pe_l - p1))   cross(z2_l, (pe_l - p2_l));
       z0                        z1                        z2_l];

J_r = simplify(J_r);
J_l = simplify(J_l);

% Position-only submatrix (3×3) for singularity analysis
Jv_r = J_r(1:3, :);
Jv_l = J_l(1:3, :);

det_r = simplify(det(Jv_r));
det_l = simplify(det(Jv_l));

disp('det(Jv_r) ='); disp(det_r);
disp('det(Jv_l) ='); disp(det_l);

