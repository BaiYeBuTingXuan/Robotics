from Robot.Robot import Robot
import numpy as np
import until
import time
import math


def traj_example():
    # 根据具体的串口进行更改，波特率不需要更改
    # com: Windows: "COM6" Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"
    r = Robot(com='COM6', baud=250000) 
    # 连接到真实机器人
    r.connect()
    # 控制周期（推荐周期，最好不要改）
    T = 0.02
    # 初始化时间
    t = 0
    # 对应三点的关节值
    qA_1 =  np.array([-13.09570312, 63.10546875 , 82.6171875, -55.1953125 ,6.328125 , 86.30859375])#/math.pi*180  //物块一的关节角
    qA_2 =  np.array([ 5.53710938, 64.95117188, 75.234375, -48.33984375 , 5.88867188 , 86.30859375])#/math.pi*180  //物块二的关节角
    # qB =  np.array([-0.08845569, 0.11414149, 1.54840006, -0.5902079, 0.04097188,-0.07763637]).T/math.pi*180
    qB =  np.array([ -85.95703125,  60.55664062, 65.0390625, -33.13476562 ,  8.0859375 ,  85.51757812] ).T#/math.pi*180  //中间点（忽略）
    qB_1 =  np.array([ 5.53710938, 64.95117188, 75.234375, -48.33984375 , 5.88867188 , 86.30859375] ).T#/math.pi*180
    qC =  np.array([-86.8359375  , 59.32617188,  91.23046875,-79.01367188  , 9.4921875   ,  95.00976562]).T#/math.pi*180 
    end = np.array([-86.8359375  , 59.32617188,  91.23046875,-79.01367188  , 9.4921875   ,  95.00976562])#//放置第一次物块的关节角
    end = np.array([-86.30859375  , 46.14257812  , 94.83398438 ,-69.16992188  ,   8.70117188  , 94.5703125   ])#//放置第二次物块的关节角
    # 规划代码可以写这里，提前规划好曲线，这里只是匀速规划曲线（效果肯定是不行的）
    # 规划的从零位到A点的时间，以2秒为例
    tOA_1 = 2
    # 规划曲线为匀速曲线,仅仅用于从机械臂的零位到A_1点
    v1 = (qA_1-0)/tOA_1
    # 在A_1点停止的时间
    tA_1 = 5
    # A_1到B点时间
    tA_1B = 2
    # A_1到C点时间
    tA_1C = 3
    # 过B点速度,单位是（度/秒）
    midVel = 0
    # 规划A点到C点经过B点的曲线，quinticCurvePlanning2见源代码
    k_1 = until.quinticCurvePlanning2(qA_1,qB,qC,midVel,tA_1B,tA_1C)
    # 规划完成
    # 在C点停留的时间
    tC = 5

    # C到B的时间
    tCB = 2
    # C到A_2的时间
    tCA_2 = 3
    # 过B点速度,单位是（度/秒）
    midVel = 0
    # 规划A点到C点经过B点的曲线，quinticCurvePlanning2见源代码
    k_2 = until.quinticCurvePlanning2(qC,qB_1,qA_2,midVel,tCB,tCA_2)
    # 规划完成

    # 在A_2停留的时间
    tA_2 = 5

    # A_1到B点时间
    tA_2B = 2
    # A_1到C点时间
    tA_2C = 3
    # 过B点速度,单位是（度/秒）
    midVel = 0
    # 规划A点到C点经过B点的曲线，quinticCurvePlanning2见源代码
    k_3 = until.quinticCurvePlanning2(qA_2,qB,qC,midVel,tA_2B,tA_2C)
    # 规划完成

    #开始控制机械臂运动
    # 使用该函数可以使机械臂回到零位
    r.go_home()
    # 开始控制机器人
    # 原点到A点
    while(1):
        start = time.time()
        # 重新开始一次循环
        if t >= tOA_1 + tA_1 + tA_1C + tC + tCA_2 + tA_2 + tA_2C:
            print('Control Finished')
            break
        # 通过时间与规划目标关节位置的关系，得到挡墙时刻下期望机械臂关节到达的位置
        if t < tOA_1:
            q = v1*t
        elif t < tOA_1 + tA_1:
            q=q
        elif t >= tOA_1 + tA_1 and t <tOA_1 + tA_1 + tA_1C:
            q = until.quinticCurveExcute2(k_1,t-tOA_1-tA_1)
        # 控制机械臂运动，syncMove输入格式为6*1的np.array，单位为度，代表的含义是当前周期下机械臂关节的位置
        elif t <tOA_1 + tA_1 + tA_1C + tC:
            q=q
        elif t >= tOA_1 + tA_1 + tC and t <tOA_1 + tA_1 + tA_1C + tC + tCA_2:
            q = until.quinticCurveExcute2(k_2,t-tOA_1-tA_1-tA_1C-tC)
        elif t <tOA_1 + tA_1 + tA_1C + tC + tCA_2 + tA_2:
            q=q
        elif t >= tOA_1 + tA_1 + tC + tCA_2 + tA_2 and t <tOA_1 + tA_1 + tA_1C + tC + tCA_2 + tA_2 + tA_2C:
            q = until.quinticCurveExcute2(k_3,t-tOA_1-tA_1-tA_1C-tC-tCA_2-tA_2)
        
        # 注意速度约束
        r.syncMove(np.reshape(q, (6, 1)))
        # 更新时间
        t = t + T
        # 定时器操作
        end = time.time()
        spend_time = end - start
        if spend_time < T:
            time.sleep(T - spend_time)
        else:
            print("timeout!")


if __name__ == '__main__': 
    traj_example()
  