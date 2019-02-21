function S=max_vol_approx(A, k)
    [l_A, w_A] = size(A);
    A_shadow = A;
    
    S = [];
    [l_S, w_S] = size(S);
    
    while w_S < k
        v = -1 * realmax * ones(l_A, 1);
        v_norm = -1 * realmax;
    
        for j = 1:w_A
            if norm(A_shadow(:, j)) > v_norm
                v = A(:, j);
                v_shadow = A_shadow(:, j);
                v_norm = norm(A_shadow(:, j));
            end
        end
    
        for j = 1:w_A
            a = A_shadow(:, j);
            proj = a * (dot(a, v_shadow)/norm(a)^2);
            A_shadow(:, j) = a - proj;
        end
        
        S = [S v];
        [l_S, w_S] = size(S);
    end

end