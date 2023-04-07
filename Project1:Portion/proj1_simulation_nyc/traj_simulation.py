#python
import sys
# Write the path to ik.py and until.py here
# For example, the IK folder is in your Documents folder
sys.path.append(f'D:/tool/robotics')

import ik
import until
import getpass
import numpy as np


####################################
### You Can Write Your Code Here ###                   

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


####################################

def sysCall_init():
    # initialization the simulation
    doSomeInit()    # must have           

    #---------------------------------------------------------------------------
    
    """ this program shows a 3-postion picking task
    step1: the robot stats to run from the rest position (q0)
    step2: the robot moves to box1 and wait for 5s
    step3: the robot moves to A
    step4: the robot moves to B
    step5: the robot moves to end and wait for 5s
    step6: the robot moves to box2 and wait for 5s
    step7: the robot moves to A
    step8: the robot moves to B
    step9: the robot moves to  end and wait for 5s
    step10: the robot moves to start and wait for 2s
    step11: suspend simulation
    
    parameter:
    interval_*2*: time interval of every curve
    n: the number of segments of A-B curve, odd int > 0
    t_*2*: the time when the robot reach the end of curve and about to move like next curve
    coef_*2*: the coefficient of 5-degree fitting curve
    interval_seg: time interval of every small segment on A-B curve
    coefs_A2B: list of the coefficient of 5-degree fitting curve of every small segment
    """
    global coef_start2box1, coef_box12A, coefs_A2B, coef_B2end, coef_end2box2, coef_box22A, coef_end2start
    global interval_start2box1, interval_box12A, interval_A2B_seg, interval_B2end, interval_end2box2
    global interval_box22A, interval_end2start
    global t_start2box1, t_box12A, t_A2B, t_B2end, t_end2box2, t_box22A, t_suspend
    global t_A2B_again, t_B2end_again, t_end2start
    global n, qB
    start = np.array([0, 0, 0, 0, 0, 0])
    box_1 = np.array([-13.0957, 63.1055, 82.6172, -55.1953, 6.3281, 86.3086])/180*np.pi
    box_2 = np.array([5.5371, 64.9512, 75.2344, -48.3398, 5.8887, 86.3086])/180*np.pi
    position_A = np.array([0.37, -0.09, 0.115, np.pi, 0, 0])
    position_B = np.array([0.288, -0.288, 0.155, np.pi, 0, 0])
    qA = ik.legal_result(position_A)
    qB = ik.legal_result(position_B)
    end = np.array([-88.1543, 62.4902, 70.7520, -48.6035, 8.0859, 85.7812])/180*np.pi
    
    print('qB:');print(qB)
    print('end:');print(end)
    print('joint limits:');print(Joint_limits)
    
    interval_start2box1 = 2
    interval_box12A = 5
    interval_A2B_seg = 0.5
    interval_B2end = 5
    interval_end2box2 = 5
    interval_box22A = 5
    interval_end2start = 2

    n = 19
    
    t_start2box1 = interval_start2box1 + 5 # wait for 5s to suck
    t_box12A = t_start2box1 + interval_box12A
    t_A2B = t_box12A + n * interval_A2B_seg
    t_B2end = t_A2B + interval_B2end + 5    # wait for 5s to unsuck
    t_end2box2 = t_B2end + interval_end2box2 + 5    # wait for 5s to suck
    t_box22A = t_end2box2 + interval_box22A
    t_A2B_again = t_box22A + n * interval_A2B_seg
    t_B2end_again = t_A2B_again + interval_B2end + 5    # wait for 5s to unsuck
    t_end2start = t_B2end_again + interval_end2start
    t_suspend = t_end2start + 2 #wait for 2s and then suspend

    coef_start2box1 = until.quinticCurvePlanning(start, box_1, interval_start2box1)
    coef_box12A = until.quinticCurvePlanning(box_1, qA, interval_box12A)
    coefs_A2B = get_all_coef(qA, qB, interval_A2B_seg, n)
    coef_B2end = until.quinticCurvePlanning(qB,end, interval_B2end)
    coef_end2box2 = until.quinticCurvePlanning(end, box_2, interval_end2box2)
    coef_box22A = until.quinticCurvePlanning(box_2, qA, interval_box22A)
    coef_end2start = until.quinticCurvePlanning(end, start, interval_end2start)

    #--------------------------------------------------------------------------
    
def sysCall_actuation():
    # put your actuation code in this function   
    
    # get absolute time, t
    t = sim.getSimulationTime()
    
    # if t>15s, pause the simulation
    if t >= t_suspend:
        sim.pauseSimulation()    
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
    
    # move from B to end
    elif t < t_B2end:
        if t-t_A2B < interval_B2end:
            q = until.quinticCurveExcute2(coef_B2end, t-t_A2B)
        else:
            q = until.quinticCurveExcute2(coef_B2end, interval_B2end)
        state = False
    
    # move from end to box2:
    elif t < t_end2box2:
        if t-t_B2end < interval_end2box2:
            q = until.quinticCurveExcute2(coef_end2box2, t-t_B2end)
        else:
            q = until.quinticCurveExcute2(coef_end2box2, interval_end2box2)
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
            q = until.quinticCurveExcute2(coef_B2end, t-t_A2B_again)
        else:
            q = until.quinticCurveExcute2(coef_B2end, interval_B2end)
        state = False
    
    # move from end to start:
    elif t < t_end2start:
        q = until.quinticCurveExcute2(coef_end2start, t-t_B2end_again)
        state = False
    
    else:
        q = until.quinticCurveExcute2(coef_end2start, interval_end2start)
        state = False
    
    # check if the joint velocities beyond limitations.
    # if they do, the simulation will stops and report errors.
    runState = move(q, state)

    if not runState:
        sim.pauseSimulation()
        


####################################################
### You Don't Have to Change the following Codes ###
####################################################

def doSomeInit():
    global Joint_limits, Vel_limits, Acc_limits
    Joint_limits = np.array([[-200, -90, -120, -150, -150, -180],
                            [200, 90, 120, 150, 150, 180]]).transpose()/180*np.pi
    Vel_limits = np.array([100, 100, 100, 100, 100, 100])/180*np.pi
    Acc_limits = np.array([500, 500, 500, 500, 500, 500])/180*np.pi
    
    global lastPos, lastVel, sensorVel
    lastPos = np.zeros(6)
    lastVel = np.zeros(6)
    sensorVel = np.zeros(6)
    
    global robotHandle, suctionHandle, jointHandles
    robotHandle = sim.getObject('.')
    suctionHandle = sim.getObject('./SuctionCup')
    jointHandles = []
    for i in range(6):
        jointHandles.append(sim.getObject('./Joint' + str(i+1)))
    sim.writeCustomDataBlock(suctionHandle, 'activity', 'off')
    sim.writeCustomDataBlock(robotHandle, 'error', '0')
    
    global dataPos, dataVel, dataAcc, graphPos, graphVel, graphAcc
    dataPos = []
    dataVel = []
    dataAcc = []
    graphPos = sim.getObject('./DataPos')
    graphVel = sim.getObject('./DataVel')
    graphAcc = sim.getObject('./DataAcc')
    color = [[1, 0, 0], [0, 1, 0], [0, 0, 1], [1, 1, 0], [1, 0, 1], [0, 1, 1]]
    for i in range(6):
        dataPos.append(sim.addGraphStream(graphPos, 'Joint'+str(i+1), 'deg', 0, color[i]))
        dataVel.append(sim.addGraphStream(graphVel, 'Joint'+str(i+1), 'deg/s', 0, color[i]))
        dataAcc.append(sim.addGraphStream(graphAcc, 'Joint'+str(i+1), 'deg/s2', 0, color[i]))

def sysCall_sensing():
    # put your sensing code here
    if sim.readCustomDataBlock(robotHandle,'error') == '1':
        return
    global sensorVel
    for i in range(6):
        pos = sim.getJointPosition(jointHandles[i])
        if i == 0:
            if pos < -160/180*np.pi:
                pos += 2*np.pi
        vel = sim.getJointVelocity(jointHandles[i])
        acc = (vel - sensorVel[i])/sim.getSimulationTimeStep()
        if pos < Joint_limits[i, 0] or pos > Joint_limits[i, 1]:
            print("Error: Joint" + str(i+1) + " Position Out of Range!")
            sim.writeCustomDataBlock(robotHandle, 'error', '1')
            return
        
        if abs(vel) > Vel_limits[i]:
            print("Error: Joint" + str(i+1) + " Velocity Out of Range!")
            sim.writeCustomDataBlock(robotHandle, 'error', '1')
            return
        
        if abs(acc) > Acc_limits[i]:
            print("Error: Joint" + str(i+1) + " Acceleration Out of Range!")
            sim.writeCustomDataBlock(robotHandle, 'error', '1')
            return
        
        sim.setGraphStreamValue(graphPos,dataPos[i], pos*180/np.pi)
        sim.setGraphStreamValue(graphVel,dataVel[i], vel*180/np.pi)
        sim.setGraphStreamValue(graphAcc,dataAcc[i], acc*180/np.pi)
        sensorVel[i] = vel

def sysCall_cleanup():
    # do some clean-up here
    sim.writeCustomDataBlock(suctionHandle, 'activity', 'off')
    sim.writeCustomDataBlock(robotHandle, 'error', '0')


def move(q, state):
    if sim.readCustomDataBlock(robotHandle,'error') == '1':
        return
    global lastPos, lastVel
    for i in range(6):
        if q[i] < Joint_limits[i, 0] or q[i] > Joint_limits[i, 1]:
            print('t=');print(sim.getSimulationTime())
            print("q["+str(i+1)+"]=");print(q[i])
            print("move(): Joint" + str(i+1) + " Position Out of Range!")
            return False
        if abs(q[i] - lastPos[i])/sim.getSimulationTimeStep() > Vel_limits[i]:
            print("move(): Joint" + str(i+1) + " Velocity Out of Range!")
            return False
        if abs(lastVel[i] - (q[i] - lastPos[i]))/sim.getSimulationTimeStep() > Acc_limits[i]:
            print("move(): Joint" + str(i+1) + " Acceleration Out of Range!")
            return False
            
    lastPos = q
    lastVel = q - lastPos
    
    for i in range(6):
        sim.setJointTargetPosition(jointHandles[i], q[i])
        
    if state:
        sim.writeCustomDataBlock(suctionHandle, 'activity', 'on')
    else:
        sim.writeCustomDataBlock(suctionHandle, 'activity', 'off')
    
    return True
    