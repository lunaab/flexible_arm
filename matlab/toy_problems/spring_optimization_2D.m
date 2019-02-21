
% World information
g = 0; %-9.81;

% Spring Information
L1 = 2;
k1 = 10;
b1 = 0; %1;
anchor1 = [-1 0];

L2 = 2;
k2 = 10;
b2 = 0; %1;
anchor2 = [1 0];

% Mass information
m = 1;

% Objective function
f = @(pos)(-1*m*g*pos(2)) ...
    + 0.5*(k1)*(sqrt((pos(2)-anchor1(2))^2 + (pos(1)-anchor1(1))^2)-L1)^2 ...
    + 0.5*(k2)*(sqrt((pos(2)-anchor2(2))^2 + (pos(1)-anchor2(1))^2)-L2)^2 ...
    + 0.5*(b1)*(atan((pos(1)-anchor1(1))/(pos(2)-anchor1(2))))^2 ...
    + 0.5*(b2)*(atan((anchor1(1)-pos(1))/(anchor1(2)-pos(2))))^2;


x0 = [0 0];
x = simulannealbnd(f,x0);
%x = fminsearch(f, x0, options);
