function out = spring_PE3(pos, ks, Ls, bs, anchors, m, g)

% Some nice vectors
ez = [0; 0; 1];

% Gravitational Potential Energy (PE)
grav = -1*m*g*pos(2);

% Spring PE caused by streching
spring1_l = 0.5*ks(1)*( sqrt( (pos(1) - anchors(1,1))^2 + (pos(2) - anchors(1,2))^2 + (pos(3) - anchors(1,3))^2 ) - Ls(1) )^2;

spring2_l = 0.5*ks(2)*( sqrt( (pos(1) - anchors(2,1))^2 + (pos(2) - anchors(2,2))^2 + (pos(3) - anchors(2,3))^2 ) - Ls(2) )^2;

spring3_l = 0.5*ks(3)*( sqrt( (pos(1) - anchors(3,1))^2 + (pos(2) - anchors(3,2))^2 + (pos(3) - anchors(3,3))^2 ) - Ls(3) )^2;


% Determining the bend angle for each spring
A1 = [pos(1) - anchors(1,1); pos(2) - anchors(1,2); pos(3) - anchors(1,3)];
A1 = A1/norm(A1);
if (isnan(A1))
    alpha1 = 0;
else 
    alpha1 = acos(dot(A1,ez));
end

A2 = [pos(1) - anchors(2,1); pos(2) - anchors(2,2); pos(3) - anchors(2,3)];
A2 = A2/norm(A2);
if (isnan(A2))
    alpha2 = 0;
else 
    alpha2 = acos(dot(A2,ez));
end

A3 = [pos(1) - anchors(3,1); pos(2) - anchors(3,2); pos(3) - anchors(3,3)];
A3 = A3/norm(A3);
if (isnan(A3))
    alpha3 = 0;
else 
    alpha3 = acos(dot(A3,ez));
end

% Spring PE caused by bending
spring1_b = 0.5*bs(1)*alpha1^2;

spring2_b = 0.5*bs(2)*alpha2^2;

spring3_b = 0.5*bs(3)*alpha3^2;

% Total Potential Energy
out = grav + spring1_l + spring2_l + spring3_l + spring1_b + spring2_b + spring3_b;

end