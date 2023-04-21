from Robot.Robot import Robot
import numpy as np
import until
import time
import sys
# Write the path to ik.py and until.py here
# For example, the IK folder is in your Documents folder
sys.path.append(f'D:/tool/robotics')

import ik
import until
import getpass
import numpy as np



'''
function: generate a point in specfic proportion to the two-point connection
parameter:
    q0 -- start point, type of 1-D array,
    q1 -- end point, type of 1-D array,
    theta -- the proportion to divide the line, type of float between [0,1]
return:
    q -- generated point, type of 1-D array,
'''
def convex_combination(q0,q1,theta):

    q = q0*(1-theta) + q1*theta
    return q


'''
function: generate several equal sets of points on a two-point line
parameter:
    q0 -- start point, type of 1-D array
    q1 -- end point type of 1-D array
    n -- the number of points to return, type of even int > 0
return:
    qs -- lists of generated points, type of (n,) list with element of type (d,) array, 
          where d is the dimension of q0
'''
def linear_imputation(q0,q1,n:'int>0'):
    thetas = np.linspace(0.0,1.0,n)
    qs = []
    for theta in thetas:
        q = convex_combination(q0,q1,theta)
        qs.append(q)
    return qs


'''
function: get the coefficient matrix of 5-degree polynomial fit curve of n equal
          length segments on a line
parameter:
    q_start -- start point in joint space, type of 1-D array
    q_end -- end point in joint space, type of 1-D array
    v_imp -- velocity imputation, type of (n,) list with element of 1-D array
    a_imp -- accelaration imputation, type of (n,) list with element of 1-D array
    time_seg -- time length of every segment
    n -- number of segments, type of odd int > 0
return:
    k -- list of coefficient matrix, type of (n,) list with element of matrix
'''
def get_all_coef(q_start, q_end, time_seg, n):
    points = linear_imputation(q_start, q_end, n+1)
    a_imp = [np.zeros(6)] * (n+1)
    v_imp = [np.zeros(6)]
    # v_imp[0] = v_imp[-1] = 0
    # v_imp[i] = average velocity between two segments
    for i in range(n-1):
        v_imp.append((points[i+2]-points[i])/2/time_seg)
    v_imp.append(np.zeros(6))
    k = []
    for i in range(n):
        coef_i = until.myPlanning(startPosition=points[i], endPosition=points[i+1], 
                                  startVel=v_imp[i], endVel=v_imp[i+1], 
                                  startAcc=a_imp[i], endAcc=a_imp[i+1], 
                                  time=time_seg)
        k.append(coef_i)
    return k    


start = np.array([0, 0, 0, 0, 0, 0])
# box_1 = np.array([-13.0957, 63.1055, 82.6172, -55.1953, 6.3281, 86.3086])
# box_2 = np.array([5.5371, 64.9512, 75.2344, -48.3398, 5.8887, 86.3086])
box_1 = np.array([-5.00976562 ,  61.34765625,  74.61914062, -54.6015625,  0.08789062,   0.61523438 ])
box_2 = np.array([  15.3046875 ,   64.49023438,  69.9609375, -54.41601562,  -0.69140625 ,  -0.52734375 ])
position_A = np.array([0.37, -0.09, 0.135, np.pi, 0, 0])
position_B = np.array([0.288, -0.288, 0.175, np.pi, 0, 0])
qA = ik.legal_result(position_A) * 180 / np.pi
qB = ik.legal_result(position_B) * 180 / np.pi
# end = np.array([-88.1543, 62.4902, 70.7520, -48.6035, 8.0859, 85.7812])
end_1 = np.array([ -90.30859375 ,   62.3359375,  69.2578125, -50.328125,   3.3046875 ,  3.58789062 ])
end_2 = np.array([ -90.30859375 ,   52.0359375,  73.2578125, -45.328125,   3.3046875 ,  3.58789062 ])

mid_1 = np.array([ -5.44921875  ,  50.00976562,  71.98242188, -43.41796875,  0.43945312,   -0.26367188])
mid_2 = np.array([-64.16015625 ,  42.71484375,  54.84375, -15.38085938,  0.08789062,  -0.08789062])
mid_3 = np.array([  20.546875  ,  48.42773438,  65.91796875, -28.03710938,  0.3515625,   -0])

print('qB:');print(qB)
# print('end:');print(end)

interval_start2box1 = 2
interval_box12A = 2
interval_A2B_seg = 0.1
interval_B2end = 5
interval_end2box2 = 5
interval_box22A = 2
interval_end2start = 2
interval_B2end2 = 5

n = 19

t_start2box1 = interval_start2box1 + 2 # wait for 5s to suck
t_box12A = t_start2box1 + interval_box12A
t_A2B = t_box12A + n * interval_A2B_seg
t_B2end = t_A2B + interval_B2end + 2    # wait for 5s to unsuck
t_end2box2 = t_B2end + interval_end2box2 + 2    # wait for 5s to suck
t_box22A = t_end2box2 + interval_box22A
t_A2B_again = t_box22A + n * interval_A2B_seg
t_B2end_again = t_A2B_again + interval_B2end2 + 2    # wait for 5s to unsuck
t_end2start = t_B2end_again + interval_end2start
t_suspend = t_end2start + 2 #wait for 2s and then suspend


#temp v
v_mid = 0.1


coef_start2box1 = until.quinticCurvePlanning(start, box_1, interval_start2box1)
coef_box12A = until.quinticCurvePlanning2(box_1,mid_1, qA,v_mid, interval_box12A/2,interval_box12A)
coefs_A2B = get_all_coef(qA, qB, interval_A2B_seg, n)
# coef_B2end1 = until.quinticCurvePlanning(qB,end_1, interval_B2end)
coef_B2end1 = until.quinticCurvePlanning2(qB,mid_2,end_1, v_mid,interval_B2end/2,interval_B2end)
coef_end12box2 = until.quinticCurvePlanning2(end_1, mid_3,box_2, v_mid,interval_end2box2/2,interval_end2box2)
coef_box22A = until.quinticCurvePlanning2(box_2, mid_3,qA,v_mid, interval_box22A/2,interval_box22A)
coef_end22start = until.quinticCurvePlanning(end_2, start, interval_end2start)

coef_B2end2 = until.quinticCurvePlanning2(qB,mid_2,end_2,v_mid, interval_B2end/2,interval_B2end)


def traj_example():
    # 根据具体的串口进行更改，波特率不需要更改
    # com: Windows: "COM6" Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"
    r = Robot(com='COM5', baud=250000) 
    # 连接到真实机器人
    r.connect()
    # 控制周期（推荐周期，最好不要改）
    T = 0.02
    # 初始化时间
    t = 0

    #开始控制机械臂运动
    # 使用该函数可以使机械臂回到零位
    r.go_home()
    # 开始控制机器人
    # 原点到A点
    while(1):
        time_start = time.time()

        if t >= t_suspend:
            pass    
        # move from start to box1
        if t < t_start2box1:
            # call the trajactory planning funcion
            if t <= interval_start2box1:
                q = until.quinticCurveExcute2(coef_start2box1, t)
            else:
                q = until.quinticCurveExcute2(coef_start2box1, interval_start2box1)
            state = False # vacumm gripper is off
        
        # move from box1 to A
        elif t < t_box12A:
            q = until.quinticCurveExcute2(coef_box12A, t-t_start2box1)
            state = False

        # move from A to B
        elif t < t_A2B:
            # q = until.quinticCurveExcute2(k_A2B, t-t_box12A)
            state = False
            # first figure out which the segment curve the robot is implementing
            seg_index = int((t-t_box12A)/interval_A2B_seg)
            if seg_index < n:
                q = until.quinticCurveExcute2(coefs_A2B[seg_index],
                                        t-t_box12A-seg_index*interval_A2B_seg)
            else:
                q = qB
        
        # move from B to end1
        elif t < t_B2end:
            if t-t_A2B < interval_B2end:
                q = until.quinticCurveExcute2(coef_B2end1, t-t_A2B)
            else:
                q = until.quinticCurveExcute2(coef_B2end1, interval_B2end)
            state = False
        
        # move from end to box2:
        elif t < t_end2box2:
            if t-t_B2end < interval_end2box2:
                q = until.quinticCurveExcute2(coef_end12box2, t-t_B2end)
            else:
                q = until.quinticCurveExcute2(coef_end12box2, interval_end2box2)
            state = False
        
        # move from box2 to A
        elif t < t_box22A:
            q = until.quinticCurveExcute2(coef_box22A, t-t_end2box2)
            state = False

        # move from A to B again
        elif t < t_A2B_again:
            # q = until.quinticCurveExcute2(k_A2B, t-t_box12A)
            state = False
            # first figure out which the segment curve the robot is implementing
            seg_index = int((t-t_box22A)/interval_A2B_seg)
            if seg_index < n:
                q = until.quinticCurveExcute2(coefs_A2B[seg_index],
                                        t-t_box22A-seg_index*interval_A2B_seg)
            else:
                q = qB
        
        # move from B to end again
        elif t < t_B2end_again:
            if t-t_A2B_again < interval_B2end:
                q = until.quinticCurveExcute2(coef_B2end2, t-t_A2B_again)
            else:
                q = until.quinticCurveExcute2(coef_B2end2, interval_B2end)
            state = False
        
        # move from end to start:
        elif t < t_end2start:
            q = until.quinticCurveExcute2(coef_end22start, t-t_B2end_again)
            state = False
        
        else:
            q = until.quinticCurveExcute2(coef_end22start, interval_end2start)
            state = False


        # # 重新开始一次循环
        # if t >= 30:
        #     print('Control Finished')
        #     break
        # # 通过时间与规划目标关节位置的关系，得到挡墙时刻下期望机械臂关节到达的位置
        # if t < 5:
        #     q = until.quinticCurveExcute2(k_start2box1,)
        # elif t >= 6 and t <11:
        #     q = until.quinticCurveExcute2(k_box12A,t-6)
        # elif t > 12 and t < 22:
        #     q = until.quinticCurveExcute2(k_A2B,t-12)
        # elif t > 23 and t < 28:
        #     q = until.quinticCurveExcute2(k_B2end,t-23)
        # 控制机械臂运动，syncMove输入格式为6*1的np.array，单位为度，代表的含义是当前周期下机械臂关节的位置
        # 注意速度约束
        r.syncMove(np.reshape(q, (6, 1)))
        # 更新时间
        t = t + T
        # 定时器操作
        time_end = time.time()
        spend_time = time_end - time_start
        if spend_time < T:
            time.sleep(T - spend_time)
        else:
            print("timeout!")


if __name__ == '__main__': 
    traj_example()
  
