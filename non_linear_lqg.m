clear variables;
clc;

% Define values of system variables
% variable  unit
M = 1000;   % kg
m1 = 100;   % kg
m2 = 100;   % kg
l1 = 20;    % m
l2 = 10;    % m
g = 9.8;    % m/s^2

% Define simulation parameters
tspan = 0:0.1:100;

% Define systen matrices
a = [0 1 0 0 0 0;
     0 0 -g*m1/M 0 -g*m2/M 0;
     0 0 0 1 0 0;
     0 0 -(M*g + m1*g)/(M*l1) 0 -m2*g/(M*l1) 0;
     0 0 0 0 0 1;
     0 0 -m1*g/(M*l2) 0 -(M*g + m2*g)/(M*l2) 0];
b = transpose([0 1/M 0 1/(l1*M) 0 1/(l2*M)]);

% Define output vector matrix
c1 = [1 0 0 0 0 0;
      0 0 0 0 0 0;
      0 0 0 0 0 0];

% Get gain matrices using pole placement method
P = [-10 -11 -12 -13 -14 -15];
L1 = place(a', c1', P)';

% Define LQG controller matrices
A = a-L1*c1;
B = [b L1];
C = eye(6);
D = 0*[b L1];

final_response = simset('solver','ode45','SrcWorkspace','Base');
sim('non_linear_LQG',tspan,final_response);