% World Information
g = -9.81;

% Spring Information
ks = [1; 1; 1];
Ls = 5*[1; 1; 1]; %(2*sqrt(3)/3)
bs = 0*[1; 1; 1];
anchors = [0 2*sqrt(3)/3 0; -1 -sqrt(3)/3 0; 1 -sqrt(3)/3 0];

% Mass Information
m = 1;

f = @(pos)spring_PE3(pos, ks, Ls, bs, anchors, m, g);

x0 = [0 0 0];
x = fminsearch(f, x0);

X = [anchors(1,1) anchors(2,1) anchors(3,1), anchors(1,1)];
Y = [anchors(1,2) anchors(2,2) anchors(3,2), anchors(1,2)];
Z = [anchors(1,3) anchors(2,3) anchors(3,3), anchors(1,3)];

plot3(X,Y,Z, '-r');
hold on
plot3([anchors(1,1) x(1)], [anchors(1,2) x(2)],[anchors(1,3) x(3)], '-g');
plot3([anchors(2,1) x(1)], [anchors(2,2) x(2)],[anchors(2,3) x(3)], '-g');
plot3([anchors(3,1) x(1)], [anchors(3,2) x(2)],[anchors(3,3) x(3)], '-g');