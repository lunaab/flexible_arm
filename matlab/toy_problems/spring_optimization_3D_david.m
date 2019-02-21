% World Information
g = 0;

% Simualtion Variables
x_spac = 0.035;
k_passive = 6000;
k_active = 8000;
l_passive = 0.035;
l_active = 0.9*l_passive;

% Spring Information
ks = [k_active; k_active; k_active];
Ls = [0.0381404*0.9; 0.0454664*0.9; 0.0381404*0.9]; %(2*sqrt(3)/3)
bs = 0*[1; 1; 1];
anchors = [0 2 0.04; 0 2+x_spac 0.04; 0 2+x_spac/2 0.0647487];

% Mass Information
m = 0.052;

f = @(pos)spring_PE3_david(pos, ks, Ls, bs, anchors, m, g);

%x0 = [0.024415711154404,2.010418304886409,0.061763535296554];
x0 =[ -0.0143385   1.99747912  0.04083488 ];
x = fminsearch(f, x0);

X = [anchors(1,1) anchors(2,1) anchors(3,1), anchors(1,1)];
Y = [anchors(1,2) anchors(2,2) anchors(3,2), anchors(1,2)];
Z = [anchors(1,3) anchors(2,3) anchors(3,3), anchors(1,3)];

plot3(X,Y,Z, '-r');
hold on
plot3([anchors(1,1) x(1)], [anchors(1,2) x(2)],[anchors(1,3) x(3)], '-g');
plot3([anchors(2,1) x(1)], [anchors(2,2) x(2)],[anchors(2,3) x(3)], '-g');
plot3([anchors(3,1) x(1)], [anchors(3,2) x(2)],[anchors(3,3) x(3)], '-g');
