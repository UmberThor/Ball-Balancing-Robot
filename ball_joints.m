clear;
close all;
clc;

% CONSTANTS
R_b = 58.19;
R_p = 84.00;
L1 = 80;
L2 = 80;

% COORDINATE OF POINTS ON BASE
base_c = [0, 0, 0];

M1 = [R_b * cos(pi); R_b * sin(pi); 0];
M2 = [R_b * cos(5*pi/3); R_b * sin(5*pi/3); 0];
M3 = [R_b * cos(pi/3); R_b * sin(pi/3); 0];


% HEIGHT AND NORMAL VECTOR
h = 100;
% n = [0.25, 0.25, 1];
n = [0, 0, 1];
n = n/norm(n);

% BALL JOINT COORDINATES
B1 = [
    -n(3)*R_p/sqrt(n(1)^2 + n(3)^2); 
    0;
    h + n(1)*R_p/sqrt(n(1)^2 + n(3)^2)
    ];

B2 = [
    n(3)*R_p/sqrt(4*n(3)^2 + (n(2)*sqrt(3) - n(1))^2);
    - sqrt(3) * n(3) * R_p / sqrt(4*n(3)^2 + (n(2)*sqrt(3) - n(1))^2);
    h + R_p * (n(2) * sqrt(3) - n(1)) / sqrt(4*n(3)^2 + (n(2)*sqrt(3) - n(1))^2);
    ];

B3 = [
    n(3)*R_p/sqrt(4*n(3)^2 + (n(2)*sqrt(3) + n(1))^2);
    sqrt(3) * n(3) * R_p / sqrt(4*n(3)^2 + (n(2)*sqrt(3) + n(1))^2);
    h - R_p * (n(2) * sqrt(3) + n(1)) / sqrt(4*n(3)^2 + (n(2)*sqrt(3) + n(1))^2);
    ];

% REACHABILITY
assert(norm(B1-M1)<L1+L2 & norm(B1-M1)>abs(L1-L2));
assert(norm(B2-M2)<L1+L2 & norm(B2-M2)>abs(L1-L2));
assert(norm(B3-M3)<L1+L2 & norm(B3-M3)>abs(L1-L2));


% PLOT
figure;
xlim([-2, 2]);
ylim([-2, 2]);
zlim([-1, 4]);
axis equal;

% Elements on base
scatter3(base_c(1), base_c(2), base_c(3), 'k', 'filled'); hold on;
scatter3(M1(1), M1(2), M1(3), 'r', 'filled'); hold on;
scatter3(M2(1), M2(2), M2(3), 'g', 'filled'); hold on;
scatter3(M3(1), M3(2), M3(3), 'b', 'filled'); hold on;
plotCircle3D(base_c, [0, 0, 1], R_b, 'k-'); hold on;

% Planes

% B1 plane
% [x z] = meshgrid(-2:2:0, -1:5:4);
% y = z * 0;
% s = surf(x,y,z); hold on;
% s.FaceColor = 'r';
% s.FaceAlpha = 0.25;

% B2 plane
% [x z] = meshgrid(0:2:2, -1:5:4);
% y = -sqrt(3) * x;
% s = surf(x,y,z); hold on;
% s.FaceColor = 'g';
% s.FaceAlpha = 0.25;

% B3 plane
% [x z] = meshgrid(0:2:2, -1:5:4);
% y = sqrt(3) * x;
% s = surf(x,y,z); hold on;
% s.FaceColor = 'b';
% s.FaceAlpha = 0.25;

% Plane plane
% [x y] = meshgrid(-2:4:2, -2:4:2);
% z = 1/(n(3)) * (h * n(3) - n(1) * x - n(2) * y);
% s = surf(x,y,z); hold on;
% s.FaceColor = 'y';
% s.FaceAlpha = 0.25;


% Ball joints
scatter3(0, 0, h, 'y', 'filled'); hold on;
plotCircle3D([0, 0, h], n, R_p, 'k-'); hold on;
quiver3(0, 0, h, 10*n(1), 10*n(2), 10*n(3), 'k');
scatter3(B1(1), B1(2), B1(3), 'r', 'filled'); hold on;
scatter3(B2(1), B2(2), B2(3), 'g', 'filled'); hold on;
scatter3(B3(1), B3(2), B3(3), 'b', 'filled'); hold on;

% Pin joints

% 1
A = (L1^2 - L2^2 + (M1(1)^2 - B1(1)^2 + M1(2)^2 - B1(2)^2 + M1(3)^2 - B1(3)^2))/(2*(M1(3) - B1(3)));
B = (B1(1) - M1(1))/(M1(3) - B1(3));
C = (B1(2) - M1(2))/(M1(3) - B1(3));
E = 0;
P1x = ((2*B1(1)+2*E*B1(2)-2*(B+C*E)*(A-B1(3))) - sqrt((-2*B1(1)-2*E*B1(2)+2*(B+C*E)*(A-B1(3)))^2 - 4*(1+E^2+(B+C*E)^2)*(B1(1)^2+B1(2)^2+(A-B1(3))^2-L1^2)))/(2 * (1 + E^2 + (B+C*E)^2));
P1 = [P1x; E * P1x; A + (B+C*E) * P1x];

% 2
A = (L1^2 - L2^2 + (M2(1)^2 - B2(1)^2 + M2(2)^2 - B2(2)^2 + M2(3)^2 - B2(3)^2))/(2*(M2(3) - B2(3)));
B = (B2(1) - M2(1))/(M2(3) - B2(3));
C = (B2(2) - M2(2))/(M2(3) - B2(3));
E = -sqrt(3);

P2x = ((2*B2(1)+2*E*B2(2)-2*(B+C*E)*(A-B2(3))) + sqrt((-2*B2(1)-2*E*B2(2)+2*(B+C*E)*(A-B2(3)))^2 - 4*(1+E^2+(B+C*E)^2)*(B2(1)^2+B2(2)^2+(A-B2(3))^2-L1^2)))/(2 * (1 + E^2 + (B+C*E)^2));
P2 = [P2x; E * P2x; A + (B+C*E) * P2x];

% 3
A = (L1^2 - L2^2 + (M3(1)^2 - B3(1)^2 + M3(2)^2 - B3(2)^2 + M3(3)^2 - B3(3)^2))/(2*(M3(3) - B3(3)));
B = (B3(1) - M3(1))/(M3(3) - B3(3));
C = (B3(2) - M3(2))/(M3(3) - B3(3));
E = sqrt(3);

P3x = ((2*B3(1)+2*E*B3(2)-2*(B+C*E)*(A-B3(3))) + sqrt((-2*B3(1)-2*E*B3(2)+2*(B+C*E)*(A-B3(3)))^2 - 4*(1+E^2+(B+C*E)^2)*(B3(1)^2+B3(2)^2+(A-B3(3))^2-L1^2)))/(2 * (1 + E^2 + (B+C*E)^2));
P3 = [P3x; E * P3x; A + (B+C*E) * P3x];

scatter3(P1(1), P1(2), P1(3), 'r', 'filled'); hold on;
scatter3(P2(1), P2(2), P2(3), 'g', 'filled'); hold on;
scatter3(P3(1), P3(2), P3(3), 'b', 'filled'); hold on;
plot3([P1(1), B1(1)], [P1(2), B1(2)], [P1(3), B1(3)], 'r');
plot3([P1(1), M1(1)], [P1(2), M1(2)], [P1(3), M1(3)], 'r');
plot3([P2(1), B2(1)], [P2(2), B2(2)], [P2(3), B2(3)], 'g');
plot3([P2(1), M2(1)], [P2(2), M2(2)], [P2(3), M2(3)], 'g');
plot3([P3(1), B3(1)], [P3(2), B3(2)], [P3(3), B3(3)], 'b');
plot3([P3(1), M3(1)], [P3(2), M3(2)], [P3(3), M3(3)], 'b');

axis equal;

% ANGLES
q1 = atan(sqrt((P1(2)-M1(2))^2 + (P1(1)-M1(1))^2)/abs(P1(3)-M1(3)))
q2 = atan(sqrt((P2(2)-M2(2))^2 + (P2(1)-M2(1))^2)/abs(P2(3)-M2(3)))
q3 = atan(sqrt((P3(2)-M3(2))^2 + (P3(1)-M3(1))^2)/abs(P3(3)-M3(3)))