import numpy as np
from math import cos,sin,atan2,sqrt


# modify angle range
def modify_angle(angle):
    if angle > np.pi:
        angle -= 2 * np.pi
    elif angle < -np.pi:
        angle += 2 * np.pi
    if angle > np.pi:
        angle -= 2 * np.pi
    elif angle < -np.pi:
        angle += 2 * np.pi
    return angle
# rotation matrix
def pose(a, b, c):
    r11 = cos(b)*cos(c)
    r12 = -cos(b)*sin(c)
    r13 = sin(b)
    r21 = cos(a)*sin(c)+cos(c)*sin(a)*sin(b)
    r22 = cos(a)*cos(c)-sin(a)*sin(b)*sin(c)
    r23 = -cos(b)*sin(a)
    r31 = sin(a)*sin(c)-cos(a)*cos(c)*sin(b)
    r32 = cos(c)*sin(a)+cos(a)*sin(b)*sin(c)
    r33 = cos(a)*cos(b)
    # print(r11, r12, r13, '\n')
    # print(r21, r22, r23, '\n')
    # print(r31, r32, r33, '\n')
    return r11,r12,r13,r21,r22,r23,r31,r32,r33


# inverse kinematics
def inverse(x,y,z,a,b,c,flag1,flag2,flag3):
    r11,r12,r13,r21,r22,r23,r31,r32,r33 = pose(a,b,c)
    p1,p2,p3 = x,y,z
    temp1 = sqrt(pow(p2-0.0855*r23,2)+pow(p1-0.0855*r13,2)-0.023*0.023)   #used in u1
    u1 = atan2(p2-0.0855*r23, p1-0.0855*r13) - atan2(0.023, flag1*temp1)
    c1 = cos(u1)
    s1 = sin(u1)
    temp2 = sqrt(pow(r22*c1-r12*s1,2)+pow(r21*c1-r11*s1,2))             #used in u5
    u5 = atan2(r23*c1-r13*s1, flag2*temp2)
    c5 = cos(u5)
    s5 = sin(u5)
    u6 = atan2((r12*s1-r22*c1)/c5, (r21*c1-r11*s1)/c5)
    u234 = atan2(-r33/c5, (r13*c1+r23*s1)/c5)
    c234 = cos(u234)
    s234 = sin(u234)
    # print(u1, u5, u6, u234)

    t1 = p1*c1 + p2*s1 - 0.0855*c5*c234 - 0.077*s234
    t2 = p3-0.23 + 0.0855*c5*s234 -0.077*c234
    c3 = (pow(t1,2)+pow(t2,2)-0.185*0.185-0.17*0.17)/(2*0.17*0.185)
    if c3 > 1 or c3 < -1:
        return 0,0,0,0,0,0,False
    s3 = flag3*sqrt(1-c3*c3)

    u3 = atan2(s3, c3)
    u2 = atan2((0.185+0.17*c3)*t1-0.17*s3*t2, (0.185+0.17*c3)*t2+0.17*s3*t1)
    u4 = u234 - u2 - u3

    u1 = modify_angle(u1)
    u2 = modify_angle(u2)
    u3 = modify_angle(u3)
    u4 = modify_angle(u4)
    u5 = modify_angle(u5)
    u6 = modify_angle(u6)

    joint_angle = [u1, u2, u3, u4, u5, u6]
    # joint angle limit
    joint_limit = np.array([200, 90, 120, 150, 150, 180]) / 180 * np.pi
    for i in range(6):
        if abs(joint_angle[i]) > joint_limit[i]:
            return 0,0,0,0,0,0,False

    return u1,u2,u3,u4,u5,u6,True
    

# get legal solution
def legal_result(pose):
    x,y,z,a,b,c = pose[0],pose[1],pose[2],pose[3],pose[4],pose[5]
    flag = [-1,1]
    for i in range(2):
        flag1 = flag[i]
        for j in range(2):
            flag2 = flag[j]
            for k in range(2):
                flag3 = flag[k]
                u1,u2,u3,u4,u5,u6,legal = inverse(x, y, z, a, b, c, flag1, flag2, flag3)
                if legal:
                    return np.array([u1, u2, u3, u4, u5, u6])