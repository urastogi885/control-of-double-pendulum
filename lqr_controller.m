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

% Define system matrices
a = [0 1 0 0 0 0;
     0 0 -g*m1/M 0 -g*m2/M 0;
     0 0 0 1 0 0;
     0 0 -(M*g + m1*g)/(M*l1) 0 -m2*g/(M*l1) 0;
     0 0 0 0 0 1;
     0 0 -m1*g/(M*l2) 0 -(M*g + m2*g)/(M*l2) 0];

b = transpose([0 1/M 0 1/(l1*M) 0 1/(l2*M)]);
% Considering output vector to be: x(t), theta1(t), theta2(t)
c = [1 0 0 0 0 0;
     0 0 1 0 0 0;
     0 0 0 0 1 0];
% We only have one input parameter: Force, F, along positive x-direction
d = transpose([1 0 0]);

% State weights matrix
% These weights were found using iterating over various values
q = [5 0 0 0 0 0;
     0 0 0 0 0 0;
     0 0 5000 0 0 0;
     0 0 0 0 0 0;
     0 0 0 0 5000 0;
     0 0 0 0 0 0];
r = 0.001;

% Get LQR parameters
[k, s, p] = lqr(a, b, q, r);

% Verify stability using lyapunov indirect method
% All poles should lie on the left-half plane for the system to be
% controllable
poles_a = eig(a);
a_e = a-b*k;
poles_a_e = eig(a_e);

% Define linear system model for LQR controller
system = ss(a-b*k, b, c, d);
tspan = 0:0.1:100;
initial = [5 0 deg2rad(20) 0 deg2rad(37) 0];
% Get system response for linear model 
[t,q1] = ode45(@(t,q)linear(t,q,-k*q),tspan,initial);
figure(1);
hold on
plot(t,q1(:,1),'r')
plot(t,q1(:,3),'g')
plot(t,q1(:,5),'b')
ylabel('x(t), theta1(t), theta2(t)')
xlabel('time')
title('Linear system response')
legend('x(t)','theta1(t)','theta2(t)')

% Get system response for non-linear model
[t,q2] = ode45(@(t,q)non_linear(t,q,-k*q),tspan,initial);
figure(2);
hold on
plot(t,q2(:,1),'r')
plot(t,q2(:,3),'g')
plot(t,q2(:,5),'b')
ylabel('x(t), theta1(t), theta2(t)')
xlabel('time')
title('Non-Linear system response')
legend('x(t)','theta1(t)','theta2(t)')