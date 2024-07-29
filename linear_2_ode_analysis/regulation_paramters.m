%% System parameters
% a0 = -4;
% a1 = 4;
% b0 = 2;
% b1 = 1;
clc; clear;
a0 = -10;
a1 = 10;
b0 = 1;
b1 = 1;
% T = 0.025;
T = 0.025;
% T = 0.25;
gamma = 1;

A = [0 1; -a0 -a1];
B = [b1; b0];
x0 = [10;5];

if (det(ctrb(A,B)) ~= 0)
    disp("(A,B) - controllable");
    disp(ctrb(A,B));
    disp(det(ctrb(A,B)));
else
    disp("(A,B) - not controllable");
end

%% optimization parameters
Q = eye(2);
R = 1;

[Kopt, Popt] = lqr(A,B,Q,R);
disp("optimal P");
disp(Popt);
disp("optimal K");
disp(Kopt)


%% adaptation parameters
A0 = (A-B*Kopt);
Pa = lyap(A0',0.01*eye(2));
disp("Pa for adaptive control")
disp(Pa)
Pa = Popt;
disp("A0: ")
disp(A0)

%% initial policy
Ko = [1 1];
disp("admissible policy eigenvalues")
disp(eig(A-B*Ko));

