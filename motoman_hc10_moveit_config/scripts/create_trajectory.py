#! /usr/bin/env python
import numpy as np
import csv

def moveJ(qi,qf,tf,Te):
# This function calculate the different angular values
# INPUT: initial articulations values, final articulations values, total time, step time
# OUTPUT: matrix 6x20
    l1=np.array(([1,0,0,0]))
    l2=np.array(([0,1,0,0]))
    l3=np.array(([1,tf,tf**2,tf**3]))
    l4=np.array(([0,1,2*tf,3*(tf**2)]))

    A = np.array(l1)
    A = np.vstack((A,l2))
    A = np.vstack((A,l3))
    A = np.vstack((A,l4))
    A = np.linalg.inv(A)

    for j in range (round(tf/Te)) : 
        for i in range (6) :
            Q = np.array(([qi[i]],[0],[qf[i]],[0]))
            C = np.matmul(A,Q)

            a = C[0] + C[1]*Te*j + C[2]*((Te*j)**2) + C[3]*((Te*j)**3)
            if (i == 0) :  
                b = np.array(a)
            else :
                b = np.append(b,a)
        if (j == 0):
            q = np.array(b)
        else :
            q = np.vstack((q,b))
    return q


if __name__=="__main__":

    # Compute the trajectory
    qi=[0.4419472813606262, 0.13577520847320557, 1.0137178897857666, -0.7563529014587402, -0.2653222978115082, 0.03390097618103027]
    qf = np.copy(qi)
    qf[0] = qf[0]+0.34
    q = moveJ(qi,qf,5,0.1)
    print (q)

    #TODO: get trajectory file path from user input
    traj_file_path = '../trajectories/trajectory.csv'
    # Write the trajectory
    with open(traj_file_path,'w') as csv_file:
        csv_writer = csv.writer(csv_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
        for i in range(len(q)):
            csv_writer.writerow(q[i])

    print("Trajectory written to file.")
