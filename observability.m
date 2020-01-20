clear variables;
clc;
% Define symbolic variables
syms m1 m2 M g l1 l2 x t1 t2 dx dt1 dt2 real
% Define system matrices
aT = transpose([0 1 0 0 0 0;
                0 0 -g*m1/M 0 -g*m2/M 0;
                0 0 0 1 0 0;
                0 0 -(M*g + m1*g)/(M*l1) 0 -m2*g/(M*l1) 0;
                0 0 0 0 0 1;
                0 0 -m1*g/(M*l2) 0 -(M*g + m2*g)/(M*l2) 0]);
b = transpose([0 1/M 0 1/(l1*M) 0 1/(l2*M)]);
% Weight matrices based on the output vector
% Output vectors: x(t)
c1T = transpose([1 0 0 0 0 0;
                 0 0 0 0 0 0;
                 0 0 0 0 0 0]);
% Output vectors: theta1(t), theta2(t)
c2T = transpose([0 0 0 0 0 0;
                 0 0 1 0 0 0;
                 0 0 0 0 1 0]);
% Output vectors: x(t), theta2(t)
c3T = transpose([1 0 0 0 0 0;
                 0 0 0 0 0 0;
                 0 0 0 0 1 0]);
% Output vectors: x(t), theta1(t), theta2(t)
c4T = transpose([1 0 0 0 0 0;
                 0 0 1 0 0 0;
                 0 0 0 0 1 0]);
% Find rank of the observability for each of the set of output vectors
% Running will directly give out the ranks in the command window
rank1 = rank([c1T aT*c1T (aT^2)*c1T (aT^3)*c1T (aT^4)*c1T (aT^5)*c1T])
rank2 = rank([c2T aT*c2T (aT^2)*c2T (aT^3)*c2T (aT^4)*c2T (aT^5)*c2T])
rank3 = rank([c3T aT*c3T (aT^3)*c3T (aT^3)*c3T (aT^4)*c3T (aT^5)*c3T])
rank4 = rank([c4T aT*c4T (aT^4)*c4T (aT^3)*c4T (aT^4)*c4T (aT^5)*c4T])
