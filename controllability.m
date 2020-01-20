clear variables;
clc;

% Define symbolic variables
syms m1 m2 M g l1 l2 real

% Define system matrices
a = [0 1 0 0 0 0;
     0 0 -g*m1/M 0 -g*m2/M 0;
     0 0 0 1 0 0;
     0 0 -(M*g + m1*g)/(M*l1) 0 -m2*g/(M*l1) 0;
     0 0 0 0 0 1;
     0 0 -m1*g/(M*l2) 0 -(M*g + m2*g)/(M*l2) 0];

b = transpose([0 1/M 0 1/(l1*M) 0 1/(l2*M)]);
% Define controllabilty matrix using system matrices
controllability_matrix = [b a*b (a^2)*b (a^3)*b (a^4)*b (a^5)*b];
% Controllabilty matrix should be full rank, that is, 6.
rank = rank(controllability_matrix)
% To find the coniditions at which the system is controllable 
det1 = simplify(det(controllability_matrix))
det2 = det(controllability_matrix)