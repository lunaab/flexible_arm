function PE=determinePE( free_node_info, fixed_node_info, connections, spring_info)
    
    % connections: adjacency graph weighted by spring k
    % node_info: [x y z m] 1 row per node
    % spring_info: [node1 node2 L0 k] for each spring

    % World Constants
    g = 0;
    
    function k=getSpringK(i,j)
        k=connections(i,j);
    end

    function L0=getSpringL0(i,j)
        L0 = 0;
        for iter = 1:length(spring_info)
            match1 = spring_info(iter,1:2) == [i j];
            match2 = spring_info(iter,1:2) == [j i];
            if ((match1(1,1) && match1(1,2)) || (match2(1,1) && match2(1,2)))
                L0 = spring_info(iter,3);
                break
            end
        end
    end

    function out=visited(i,j)
        out = false;
        [l_s,w_s] = size(visited_springs);
        for iter = 1:l_s
            match1 = visited_springs(iter,:) == [i j];
            match2 = visited_springs(iter,:) == [j i];
            out = ((match1(1,1) && match1(1,2)) || (match2(1,1) && match2(1,2)));
            if out
                break
            end
        end
    end


    totalPE = 0;

    [l,w] = size(connections);
    node_info = [fixed_node_info; free_node_info]; %IMPORTANT
    
    visited_springs = [];
    for i = 1:l
        if (i > length(fixed_node_info))
            totalPE = totalPE + -node_info(i,4)*node_info(i,3)*g;
        end
        for j = 1:w
            if (connections(i,j) > 0 && ~visited(i,j))
                dist = norm(node_info(j,1:3) - node_info(i,1:3));
                k = getSpringK(i,j);
                L0 = getSpringL0(i,j);
                totalPE = totalPE + 0.5*k*(dist-L0)^2;
                visited_springs = [visited_springs; i j];
            end
        end
    end
    
    PE = totalPE;
end