% World Information
g = 0;

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