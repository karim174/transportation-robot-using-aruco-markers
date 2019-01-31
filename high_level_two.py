#module
import math
import numpy as np
def inverse( speeds_v_w , length , radius):
    #calcuating right and left wheel speeds for motor given the desired V and W
    right_wheel = ((speeds_v_w[0] + length*speeds_v_w[1])/radius)*60/(2*3.14)
    left_wheel = ((speeds_v_w[0] - length*speeds_v_w[1])/radius)*60/(2*3.14)
    wheels_speeds = np.array([left_wheel , right_wheel])
    wheels_speeds[0]=max(min(wheels_speeds[0],125),0)
    wheels_speeds[1]=max(min(wheels_speeds[1],125),0)
    return wheels_speeds

def high_level_control(reference, pose, goal,offset, kp, ka, kb):

    #goal must be at least 15 cms to percieve marker this is mainly done for reaching products
    #also their is an offset between camera and motor .. literally hahaa
    #subtracting  this offset from rau makes goal closer to robot by this distance in direction
    #of rau making robot cg at this virtual goal and camera at original goal
    #so actually goal point would be achieved and reached by camera rather than CG
    #goal should be specified to be reached by the camera

    #stability is achieved  and robust at kp > 0 and kb <0 and ka+5/3kb - 2/pi * kp > 0
    #kp = 0.3 ka=8 kb=-1.5
    #pose is in form [x,z,theta] and it's a numpy array reference to robot
    #data is given in cms
    #control is in polar coordinates
    #rau = distance to goal #alpha angle between rau and Z-axis of robot
    # beta is angle between rau and orientation of Z-axis of goal

    if reference == 'product':
        z=pose[1]-(15+offset)*np.absolute(np.cos(pose[2]))+offset
        if pose[2] > 0:
            x=pose[0]-(15+offset)*np.absolute(np.sin(pose[2]))
        else:
            x=pose[0]+(15+offset)*np.absolute(np.sin(pose[2]))
        rau = math.sqrt((z) **2 + (x) **2)
        alpha = -1*np.arctan2(x,z)
        if pose[2] > 0:
            beta = 3.14-pose[2]-alpha
        else:
            beta = 3.14+pose[2]-alpha
        print (x,z)
    elif reference == 'global' :

        #from reference from robot to marker
        rot_Matrix = np.array([[np.cos(pose(2)),-1*np.sin(pose(2)),0],[np.sin(pose(2)),np.cos(pose(2)),0],[0,0,1]])
        pose_R = -1*np.dot(rot_Matrix,pose)

        #calculating goal relative to robot reference to marker
        goal_R_T = goal - pose_R
        goal_R_R =np.dot(rot_Matrix.T,goal_R_T)

        z=goal_R_R[1]+(15+offset)*np.cos(goal_R_R[2])+offset
        x=goal_R_R[0]+(15+offset)*np.sin(goal_R_R[2])
        rau = math.sqrt((z) **2 + (x) **2)
        alpha = numpy.arctan2(x,z)
        beta = goal_R_R[2]-alpha
        print (alpha,beta)
    #safety condition stop within 5 cms of goal
    if rau < 15 :
        return (0,0)
    else:
        return (kp*rau, ka*alpha-kb*beta)
