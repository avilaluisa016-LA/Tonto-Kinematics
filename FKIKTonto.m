%{ 4-DOF Tonto
%   For this project I will be using denavit-Hartenberg Parameters to 
%   derive my forward and inverse kinematic equations. }%
clc, clear;

%% Deriving Denavit-Hartenberg Parameters
% Variables:
h = 1.625; % inches
L1 = 5.75;
L2 = 5.25;
L3 = 1.5;
syms theta1 theta2 theta3 theta4

% DH parameters
% i = (a, alpha, d, theta)
f1 = [0, pi/2, 0, theta1];
f2 = [h, 0, 0, theta2]; 
f3 = [L1, 0, 0, theta3]; 
f4 = [L2, 0, 0, theta4];
f5 = [L3, 0, 0, 0];

% Compute the transformation matrices for each link
T1 = dh(f1(1), f1(2), f1(3), f1(4));
T2 = dh(f2(1), f2(2), f2(3), f2(4));
T3 = dh(f3(1), f3(2), f3(3), f3(4));
T4 = dh(f4(1), f4(2), f4(3), f4(4));
T5 = dh(f5(1), f5(2), f5(3), f5(4));

T05 = simplify(T1 * T2 * T3 * T4 * T5); % My forward kinematic matrix

fprintf("DH Parameters\n")
fprintf("~~~~~~~~~~~~~\n")
disp(T05)

fprintf("End Effector's Coordinates\n")
fprintf("~~~~~~~~~~~~~\n")
xPos = T05(1,4);
yPos = T05(2,4);
zPos = T05(3,4);

% Equation for Inverse Kinematics
fprintf("x = %s\n", char(xPos));
fprintf("y = %s\n", char(yPos));
fprintf("z = %s\n", char(zPos));
fprintf("~~~~~~~~~~~~~~/n")


%% Varifying DH Parameters
reach = h + L1 + L2 + L3; % 14.125
xyReachLimit = reach;
zReachLimit = reach;

% Numeric Visualization
while true
    angle1 = input('Enter theta1 (rad): ');
    angle2 = input('Enter theta2 (rad): ');
    angle3 = input('Enter theta3 (rad): ');
    angle4 = input('Enter theta4 (rad): ');

    N01 = dh(0,  pi/2, 0, angle1); % T changed to N to avoid errors
    N12 = dh(h,  0,    0, angle2);
    N23 = dh(L1, 0,    0, angle3);
    N34 = dh(L2, 0,    0, angle4);
    N45 = dh(L3, 0,    0, 0);

    N02 = N01*N12;
    N03 = N02*N23;
    N04 = N03*N34;
    N05 = N04*N45;

    nxPos = N05(1,4);
    nyPos = N05(2,4);
    nzPos = N05(3,4);

    origins = [[0;0;0], N01(1:3,4), N02(1:3,4), N03(1:3,4), N04(1:3,4),...
        N05(1:3,4)];

    plot3(origins(1,:), origins(2,:), origins(3,:), '-o', 'LineWidth',...
        2, 'MarkerSize', 8);
    grid on; axis equal;
    xlabel('X'); ylabel('Y'); zlabel('Z');
    xlim([-xyReachLimit xyReachLimit]);
    ylim([-xyReachLimit xyReachLimit]);
    zlim([-zReachLimit zReachLimit]);
    drawnow;

    fprintf('End-effector position: x=%.3f, y=%.3f, z=%.3f\n', nxPos,...
        nyPos, nzPos);

    again = input('Try another pose? (y/n): ', 's');
    if ~strcmpi(again, 'y')
        break;
    end
end
%% DH transform function
function T = dh(a, alpha, d, theta)
    T = [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), ...
         a*cos(theta);
         sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), ...
         a*sin(theta);
         0,            sin(alpha),             cos(alpha),            d;
         0,            0,                      0,                     1];
end