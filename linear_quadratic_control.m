g   = 9.81;
R_BALL_PX = 50;
s   = R_BALL_PX/0.02;        % [px/m]
c   = -(3/5)*g*s;            % px/s^2 per rad, hollow shell, pixel units
Ts  = 1/20;                  % nominal frame period [s]

A  = [0 1 0; 0 0 0; 1 0 0];
B  = [0; c; 0];
C = eye(3);
D = 0;
sysd = c2d(ss(A, B, C, D), Ts);

% Bryson's rule
x_max  = 50;                 % px
v_max  = 100;                % px/s
xi_max = 200;               % px*s
u_max  = deg2rad(1);         % rad

Q = diag([1/x_max^2, 1/v_max^2, 1/xi_max^2]);
R = 1/u_max^2;

K = dlqr(sysd.A, sysd.B, Q, R);
fprintf('%.6e\n', K)