function best_particle=binaryPSwarmM(rows, columns, feval,  iterations, population)

    function swarm = genSwarm(rows, columns, population)
        swarm = round(rand(rows, columns, population));
    end


    % Constants
    ITER_DEFAULT = 1000;
    POP_DEFAULT = 100;

    % Default number of iterations
    if nargin < 4
        iterations = ITER_DEFAULT;
        population = POP_DEFAULT;
    elseif nargin < 5
        population = POP_DEFAULT;
    end
    
    current_swarm = genSwarm(rows, columns, population);
    
    
end