function dQ = linear(t,Q,U)
% Define values of system variables
% variable  unit
M = 1000;   % kg
m1 = 100;   % kg
m2 = 100;   % kg
l1 = 20;    % m
l2 = 10;    % m
g = 9.8;    % m/s^2

A = [0 1 0 0 0 0; 0 0 -m1*g/M 0 -m2*g/M 0; 0 0 0 1 0 0; 0 0 -((M*g)+(m1*g))/(M*l1) 0 -g*m2/(M*l1) 0; 0 0 0 0 0 1; 0 0 -m1*g/(M*l2) 0 -((M*g)+(m2*g))/(M*l2) 0];
B = [0; 1/M; 0; 1/(l1*M); 0; 1/(l2*M)];
% Get system output
dQ = A*Q + B*U;

