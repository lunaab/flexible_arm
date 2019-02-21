% Counters
count = 1;

% World Information
g = 0;

% Simualtion Variables
x_spac = 0.035;
k_passive = 6000;
k_active = 8000;
l_passive = 0.035;
l_active = 0.9*l_passive;



% Constant Spring Information
bs = 0*[1; 1; 1];
anchors = [0 2 0.04; 0 2+x_spac 0.04; 0 2+x_spac/2 0.0647487];
anchors = anchors + 0;
num_springs = 3;
Ls = zeros(1,num_springs);
ks = zeros(1,num_springs);

% Mass Information
m = 0.052;

% Equilibrium Storage
eq_final = zeros(3, 2^num_springs);

% Looping Through Combinations
for s1 = 0:1
    if s1 == 0
        ks(1,1) = k_passive;
        Ls(1,1) = l_passive;
    else
        ks(1,1) = k_active;
        Ls(1,1) = l_active;
    end
    for s2 = 0:1
        if s2 == 0
            ks(1,2) = k_passive;
            Ls(1,2) = l_passive;
        else
            ks(1,2) = k_active;
            Ls(1,2) = l_active;
        end
        for s3 = 0:1
            if s1 == 0
                ks(1,3) = k_passive;
                Ls(1,3) = l_passive;
            else
                ks(1,3) = k_active;
                Ls(1,3) = l_active;
            end
            % Set the optimization function
            f = @(pos)spring_PE3_david(pos, ks, Ls, bs, anchors, m, g);

            % Solve the optimization and store result
            x0 = [1, 1, 1];
            x = fminsearch(f, x0);
            eq_final(:, count)=x;
            count = count + 1;
        end
    end
end

eq_final;

S = max_vol_approx(eq_final, 3);

% s = svds(eq_final);
% [U, S, V] = svd(eq_final);
% 
% [x, y, z] = ellipsoid(0,0,0,s(1,1),s(2,1),s(3,1),30);
% figure
% surf(x, y, z)
% axis equal



