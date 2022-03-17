from cmath import pi
import rospy
from time import sleep
import numpy as np
from Translate import T
import math
#UR机械臂
from rtde_control import RTDEControlInterface as RTDEControl
import rtde_receive
#from UR_robot import UR_robot as UR

import roslib
#from torch import true_divide; roslib.load_manifest('robotiq_2f_gripper_control')
#from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output  as outputMsg


def move_endtcp(pose,axis,offset):
    #从当前点pose->T_pose

    #增量判断
    #平移判断
    pose_move=[0.0]*6
    if axis[0]==1:
        pose_move[0]=offset[0]
    if axis[1]==1:
        pose_move[1]=offset[1]
    if axis[2]==1:
        pose_move[2]=offset[2]
    #旋转沿TCP旋转
    if axis[3]==1:
        pose_move[3]=offset[3]/180.0*math.pi
    if axis[4]==1:
        pose_move[4]=offset[4]/180.0*math.pi
    if axis[5]==1:
        pose_move[5]=offset[5]/180.0*math.pi
    #得到增量的pose相对于末端tcp的坐标
    #转换为相对于末端T矩阵
    #T_move为理想位姿相对于末端坐标系的坐标
    #T_pose为末端坐标系相对于基座坐标系的
    T_move=T.rot2T(pose_move)
    T_pose=T.rot2T(pose)
    #矩阵乘法需要用np.dot()
    T_target=np.dot(T_pose,T_move)
    #从T_target到pose_target
    pose_target=T.T2rot(T_target)
    return pose_target

def robotiq(ACT,GTO,PR,ATR,SP,FR):
    pub = rospy.Publisher('Robotiq2FGripperRobotOutput', outputMsg.Robotiq2FGripper_robot_output)
    command = outputMsg.Robotiq2FGripper_robot_output()
    for  i in range(2):
        command = outputMsg.Robotiq2FGripper_robot_output();
        command.rACT = ACT
        command.rGTO = GTO
        command.rPR = PR
        command.rATR = ATR
        command.rSP  = SP
        command.rFR  = FR
        pub.publish(command)
        rospy.sleep(0.1)
    rospy.sleep(4)
def move_circle(pose,offset):

        TCP_offset=0.1     #移动TCP_z轴距离
        offset=offset*0.0175   #弧度转角度
        offset_sum=0       #初始角度累加
        F_world_link=T.rot2T(pose)     #目标相对于基座的T
        F_robot_pose=rtde_r.getActualTCPPose()   #获得当前位姿
        F_robot_pose=T.rot2T(F_robot_pose)    #当前位姿转换成T
        F_link_EndFector=np.dot(np.linalg.inv(F_world_link),F_robot_pose)  #goal2baselink*eelink2goal=eelink2baselink
        P,M=T.T2rot_2(F_link_EndFector)   #旋转平移矩阵

        radius=math.sqrt(math.pow(P[0],2)+math.pow(P[1],2))   #得R
        angle=math.atan2(P[1],P[0])                           #的rad

        for i in range(20):
            
            pose_now=rtde_r.getActualTCPPose()
            target=move_endtcp(pose_now,[0,0,1,0,0,0],[0,0,TCP_offset,0,0,0])  #沿tcp移动TCP_offset距离
            print("请输入a")
            input()
            rtde_c.moveL(target,0.02,0.01,True)
            sleep(0.1)

            while abs(rtde_r.getActualTCPForce()[2]) < 3:
                print("1")
                sleep(0.05)

            print("stopL start")
            rtde_c.stopL(0.5)
            print("stopL end")
            sleep(1)

            # pose_now=rtde_r.getActualTCPPose()
            # sleep(0.1)
            # pose_now_T=T.rot2T(pose_now)
            # speed = [0.5*pose_now_T[0,2], 0.5*pose_now_T[1,2],0.5*pose_now_T[2,2], 0, 0, 0]
            # print("请输入")
            # input()
            # rtde_c.moveUntilContact(speed)
            # print("stop")

            if abs(rtde_r.getActualTCPPose()[2]-pose_now[2])>=0.1:
                print("robotiq start")
                print("请输入")
                input()
                # robotiq(1, 1, 100, 0, 10, 1)
                print("robotiq open")
                rtde_c.moveL(pose_now,0.1,0.1)
                # robotiq(1, 1, 200, 0, 10, 1)

                #60度旋转
                offset=60/180*math.pi
                for j in range(5):
                    offset_sum=offset_sum+offset
                    tran=[radius*math.cos(angle+offset_sum),radius*math.sin(angle+offset*offset_sum),P[2]]
                    rot=[M[0],M[1],M[2]+offset_sum]
                    eelink2goal=[tran[0],tran[1],tran[2],rot[0],rot[1],rot[2]]
                    eelink2goal=T.rot2T(eelink2goal)
                    target=np.dot(F_world_link,eelink2goal)
                    target=T.T2rot(target)       #目标位姿

                    ret=rtde_c.moveL([target[0],target[1],target[2],target[3],target[4],target[5]],0.1,0.1)

                    pose_now=rtde_r.getTargetTCPPose()
                    target=move_endtcp(pose_now,[0,0,1,0,0,0],[0,0,TCP_offset,0,0,0])  #沿tcp移动TCP_offset距离
                    print("请输入")
                    input()
                    sleep(3)
                    rtde_c.moveL(target,0.005,0.005,True)
                    sleep(0.5)

                    print("robotiq start")
                    print("请输入")
                    input()
                    # robotiq(1, 1, 100, 0, 10, 1)
                    print("robotiq open")
                    rtde_c.moveL(pose_now,0.1,0.1)
                    # robotiq(1, 1, 200, 0, 10, 1)

                    print("move:",ret)
                    print(target)
                    sleep(0.5)
                return 
            else:
                rtde_c.moveL(pose_now,0.1,0.1)
                offset_sum=offset_sum+offset
                tran=[radius*math.cos(angle+offset_sum),radius*math.sin(angle+offset_sum),P[2]]
                rot=[M[0],M[1],M[2]+offset_sum]
                eelink2goal=[tran[0],tran[1],tran[2],rot[0],rot[1],rot[2]]
                eelink2goal=T.rot2T(eelink2goal)
                target=np.dot(F_world_link,eelink2goal)
                target=T.T2rot(target)       #目标位姿

                ret=rtde_c.moveL(target,0.1,0.1)
                print("move:",ret)
                if ret==False:
                    offset_sum=offset_sum-offset
                sleep(0.5)
            sleep(1)

if __name__ == '__main__':

    # rospy.init_node('move')
    #ur连接
    rtde_r = rtde_receive.RTDEReceiveInterface("192.168.3.101")
    rtde_c = RTDEControl("192.168.3.101", RTDEControl.FLAG_USE_EXT_UR_CAP)
    
    #插试管
    # rot_pose=[0.4708,-0.4503,0.5796,1.611,2.680,0.020]
    rot_pose=[-0.3518,0.042,0.612,1.71,2.666,0.031]
    p1=195.71/180*pi
    p2=-72.39/180*pi
    p3=-86.84/180*pi
    p4=-97.81/180*pi
    p5=88.15/180*pi
    p6=-8.94/180*pi

    rtde_c.moveJ([p1,p2,p3,p4,p5,p6],0.5,0.2)
    sleep(1)
    move_circle(rot_pose,20)




