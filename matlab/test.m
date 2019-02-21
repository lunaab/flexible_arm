% connections: adjacency graph weighted by spring k
    % node_info: [x y z m fixed(1=YES)] 1 row per node
    % spring_info: [node1 node2 L0 k] for each spring

connections = (ones(4,4) - eye(4))*8000;
           
fixed_node_info = [0 2 0.04 0.052;
                   0 2.035 0.04 0.052;
                   0 2.0175 0.0647487 0.052];

               
free_node_position = [0 0 0];
free_node_mass = 0.052;
free_node_info = [free_node_position free_node_mass];
         
spring_info = [1 2 1 8000;
               1 3 1 8000;
               2 3 1 8000;
               1 4 0.0381404*0.9 8000;
               2 4 0.0454664*0.9 8000;
               3 4 0.0381404*0.9 8000];
           
options = optimset('OutputFcn',@outfun);
f = @(pos)determinePE([pos free_node_mass], fixed_node_info, connections, spring_info);
[x,fval,exitflag,output] = fminsearch(f, free_node_position, options);