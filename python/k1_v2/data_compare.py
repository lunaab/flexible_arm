import numpy as np


for i in range(0, 36):

    # open files
    luna_file = open('k1_v2_luna/t_' + str(i) + '.txt')
    jinda_file = open('k1_v2_jinda/t_' + str(i) + '.txt')
    
    # read off header
    luna_file.readline()
    jinda_file.readline()
    
    # read off garbage
    jinda_file.readline()
    jinda_file.readline()
    jinda_file.readline()
    jinda_file.readline()
    
    luna_pos = np.zeros((13, 3))
    jinda_pos = np.copy(luna_pos)
    for j in range(0, 13):
        luna = luna_file.readline()
        luna = luna.rstrip()
        luna = luna.split(',')
        jinda = jinda_file.readline()
        jinda = jinda.rstrip()
        jinda = jinda.split('\t')
        for k in range (0, 3):
            luna_pos[j, k] = float(luna[k])
            jinda_pos[j, k] = float(jinda[k])
            
    diff = luna_pos - jinda_pos
    np.savetxt('diff/t_' + str(i) + '.txt', diff)
            
        
        
        
    
