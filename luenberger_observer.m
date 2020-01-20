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

% State weights matrix
q = [5 0 0 0 0 0;
     0 0 0 0 0 0;
     0 0 5000 0 0 0;
     0 0 0 0 0 0;
     0 0 0 0 5000 0;
     0 0 0 0 0 0];
r = 0.001;

% Define output vector matrices
c1 = [1 0 0 0 0 0;
      0 0 0 0 0 0;
      0 0 0 0 0 0];
c3 = [1 0 0 0 0 0;
      0 0 0 0 0 0;
      0 0 0 0 1 0];
c4 = [1 0 0 0 0 0;
      0 0 1 0 0 0;
      0 0 0 0 1 0];

% Define noisy vector,b and step input at t = 50s for the observer
b_noise = randn(6, 1);
u = 0*tspan;
u(50:end) = 1;

% Get gain matrices using pole placement method
P = [-10 -11 -12 -13 -14 -15];
L1 = place(a', c1', P)';
L3 = place(a', c3', P)';
L4 = place(a', c4', P)';

% Obtain output of the linear system with output vector: x(t)
sys1 = ss(a, b+b_noise, c1,zeros(3,1));
y1_est = lsim(sys1, u, tspan);
% Obtain the estimate using the observer
sys_est_1 = ss(a-L1*c1, [b L1(:, 1)], c1, zeros(3,2));
[x1_est, t] = lsim(sys_est_1, [u; y1_est(:, 1)'], tspan);
% Draw both output and estimate for linear system
figure();
hold on
plot(t, y1_est(:,1), 'r', 'Linewidth', 2)
plot(t, x1_est(:,1), 'k--', 'Linewidth', 2)
ylabel('x(t)')
xlabel('time in s')
legend('Output of x(t)','Estimation of x(t)')
title('Linear System Response for x(t)')
% Draw both output and estimate for original system using simulink designed model
non_linear = simset('solver', 'ode45', 'SrcWorkspace', 'Base');
non_linear_output_1=sim('non_linear_observer', tspan, non_linear, c1);
figure();
hold on
plot(non_linear_output_1, output(:,1), 'r', 'Linewidth', 2)
plot(non_linear_output_1, states(:,1), 'k--', 'Linewidth', 2)
ylabel('x(t)')
xlabel('time in s')
legend('Output of x(t)','Estimation of x(t)')
title('Original System Response for x(t)')
hold off

% Obtain output of the linear system with output vector: x(t), theta1(t)
sys3 = ss(a, b+b_noise, c3, zeros(3,1));
y3_est = lsim(sys3, u, tspan);
% Obtain the estimate using the observer
sys_est_3 = ss(a-L3*c3, [b L3(:, [1 3])], c3, zeros(3,3));
[x3_est, t] = lsim(sys_est_3, [u; y3_est(:, [1 3])'], tspan);
% Draw both output and estimate for linear system
figure();
hold on
plot(t, y3_est(:,1), 'r', 'Linewidth', 2)
plot(t, x3_est(:,1), 'k--', 'Linewidth', 2)
plot(t, y3_est(:,2), 'g', 'Linewidth', 2)
plot(t, x3_est(:,2), 'k--', 'Linewidth', 2)
ylabel('x(t), theta1(t)')
xlabel('time in s')
legend('Output of x(t)', 'Estimation of x(t)', 'Output of theta2(t)', 'Estimation of theta2(t)')
title('Linear System Response for x(t), theta2(t)')
% Draw both output and estimate for original system using simulink designed model
non_linear_output_3=sim('non_linear_observer', tspan, non_linear, c3);
figure();
hold on
plot(non_linear_output_3, output(:,1), 'r', 'Linewidth', 2)
plot(non_linear_output_3, states(:,1), 'k--', 'Linewidth', 2)
plot(non_linear_output_3, output(:,2), 'g', 'Linewidth', 2)
plot(non_linear_output_3, states(:,2), 'k--', 'Linewidth', 2)
ylabel('x(t), theta1(t)')
xlabel('time in s')
legend('Output of x(t)','Estimation of x(t)','Output of theta1(t)','Estimation of theta1(t)')
title('Original System Response for x(t), theta1(t)')
hold off

% Obtain output of the linear system with output vector: x(t), theta1(t),
% theta2(t)
sys4 = ss(a,b+b_noise, c4, zeros(3,1));
y4_est = lsim(sys1, u, tspan);
% Obtain the estimate using the observer
sys_est_4 = ss(a-L4*c4,[b L4],c4,zeros(3,4));
[x4_est,t] = lsim(sys_est_4, [u; y4_est'], tspan);
% Draw both output and estimate for linear system
figure();
hold on
plot(t, y4_est(:,1), 'r', 'Linewidth', 2)
plot(t, x4_est(:,1), 'k--', 'Linewidth', 2)
plot(t, y4_est(:,2), 'g', 'Linewidth', 2)
plot(t, x4_est(:,2), 'k--', 'Linewidth', 2)
plot(t, y4_est(:,3), 'b', 'Linewidth', 2)
plot(t, x4_est(:,3), 'k--', 'Linewidth', 2)
ylabel('x(t), theta1(t), theta2(t)')
xlabel('time')
legend('Output of x(t)','Estimation of x(t)','Output of theta1(t)','Estimation of theta1(t)','Output of theta2(t)','Estimation of theta2(t)')
title('Linear System Response for x(t), theta1(t), theta2(t)')
% Draw both output and estimate for original system using simulink designed model
non_linear_output_4=sim('non_linear_observer', tspan, non_linear, c4);
figure();
hold on
plot(non_linear_output_4, output(:,1), 'r', 'Linewidth', 2)
plot(non_linear_output_4, states(:,1), 'k--', 'Linewidth', 2)
plot(non_linear_output_4, output(:,2), 'g', 'Linewidth', 2)
plot(non_linear_output_4, states(:,2), 'k--', 'Linewidth', 2)
plot(non_linear_output_4, output(:,3), 'b', 'Linewidth', 2)
plot(non_linear_output_4, states(:,3), 'k--', 'Linewidth', 2)
ylabel('x(t), theta1(t), theta2(t)')
xlabel('time in s')
legend('Output of x(t)','Estimation of x(t)','Output of theta1(t)','Estimation of theta1(t)','Output of theta2(t)','Estimation of theta2(t)')
title('Original System Response for x(t), theta1(t), theta2(t)')
hold off