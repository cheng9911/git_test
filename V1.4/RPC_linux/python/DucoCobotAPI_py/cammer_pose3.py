#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import sys
from geometry_msgs.msg import PoseStamped, Pose, Transform
from std_msgs.msg import String
import threading
#from autolab_core import RigidTransform
from time import sleep
from copy import deepcopy
import numpy as np
from scipy.spatial.transform import Rotation as Ro
import socket
from rtde_control import RTDEControlInterface as RTDEControl
import rtde_contro
def ro_rt_T(a, b, c, d, e, f, g):
    r = Ro.from_quat([d, e, f, g])
    rotation = r.as_matrix()
    translation = np.array([a, b, c])
    one = np.array([0, 0, 0, 1])
    t = np.concatenate(
        [rotation.reshape(3, 3), translation.reshape(3, 1)], axis=1)
    T = np.concatenate([t.reshape(3, 4), one.reshape(1, 4)], axis=0)
    return T


def translate2siyuan(matrix):

    end_rotation = matrix[0:3, 0:3]
    end_translation = matrix[0:3, 3]
    end_translation[0] = end_translation[0]
    r = Ro.from_matrix(end_rotation)
    end_rotation = r.as_quat()
    qvec_nvm = np.array(end_rotation)
    return end_translation, qvec_nvm


def send_matrix(tr_matrix, ro_matrix):
    str_my = ""

    for x in tr_matrix:
        str_my += str('%.4f' % x) + " "
    for y in ro_matrix:
        str_my += str('%.4f' % y) + " "
    # str_my += "end"
    str_my.encode('utf-8')
    return str_my

def quart2rot(data):
    a=Ro.from_quat(data)
    rot=a.as_rotvec()
    return rot

def rot2quart(data):
    a=Ro.from_rotvec(data)
    quart=a.as_quat()
    return quart

def genCommand(ACT,GTO,PR,ATR,SP,FR):
    command = outputMsg.Robotiq2FGripper_robot_output();
    command.rACT = ACT
    command.rGTO = GTO
    command.rPR = PR
    command.rATR = ATR
    command.rSP  = SP
    command.rFR  = FR
    return command


# def thread_spin():
#     rospy.spin()

def callback(data):


    global flag
    global pose_flag
    global move_flag
    pose_flag.position.x=pose_flag.position.x+data.position.x
    pose_flag.position.y=pose_flag.position.y+data.position.y
    pose_flag.position.z=pose_flag.position.z+data.position.z
    pose_flag.orientation.x=pose_flag.orientation.x+data.orientation.x
    pose_flag.orientation.y=pose_flag.orientation.y+data.orientation.y
    pose_flag.orientation.z=pose_flag.orientation.z+data.orientation.z
    pose_flag.orientation.w=pose_flag.orientation.w+data.orientation.w
    flag=flag+1

    if(flag==90):
        #mark旋转平移矩阵
        
        print("flag=",flag)
        #data_t = ro_rt_T(round(data.position.x,3), round(data.position.y,3),round(data.position.z,3), round(data.orientation.x,3),round(data.orientation.y,), round(data.orientation.z,3),round(data.orientation.w,3))
        data_t = ro_rt_T(pose_flag.position.x/90, pose_flag.position.y/90,pose_flag.position.z/90, pose_flag.orientation.x/90,pose_flag.orientation.y/90, pose_flag.orientation.z/90,pose_flag.orientation.w/90)
        #print("mark2cammer=",data)
    
        #current_pose旋转平移矩阵
        get_pose = rtde_r.getActualTCPPose()
        current_quart=rot2quart([get_pose[3],get_pose[4],get_pose[5]])
        current_pose=[get_pose[0],get_pose[1],get_pose[2],current_quart[0],current_quart[1],current_quart[2],current_quart[3]]
        eelink2baselink=ro_rt_T(current_pose[0],current_pose[1],current_pose[2],current_pose[3],current_pose[4],current_pose[5],current_pose[6])
        #print("get_pose=",get_pose)

        #cammer2eelink旋转平移矩阵
        cammer2eelink = np.array([[-0.999924876769970,	0.00339757983469717,	-0.0117769804206380,	0.0310828426656648],
        [-0.00327451116284631,	-0.999939999032199,	-0.0104535119552724,	0.0873993578886651],
        [-0.0118117904318327,	-0.0104141627999327,	0.999876005722765,	0.0175452134201781],
        [0,	0,	0,	1]])

        #设置目标mark2cammer
        mark2cammer = ro_rt_T(0.06+0.08,0.016+0.065,0.4-0.0675,1,0,0,0)
        #mark2cammer = ro_rt_T(0.06+0.08,0.016+0.065,0.5,1,0,0,0)

        #目标eelink2baselink_2放入list
        eelink2baselink_2=np.dot(np.dot(np.dot(np.dot(eelink2baselink,cammer2eelink),data_t),np.linalg.inv(mark2cammer)),np.linalg.inv(cammer2eelink))
        translations, rotation_matrix = translate2siyuan(eelink2baselink_2)
    
        str = send_matrix(translations, rotation_matrix)
        list = []
        list = str.split(' ')
        for i in range(6):
            list[i]=float(list[i])

        rot=quart2rot([list[3],list[4],list[5],list[6]])
        print("send_command=",[list[0],list[1], list[2],rot[0], rot[1], rot[2]])

        #移动机械臂
        #rtde_c.moveJ_IK([round(list[0],3),round(list[1],3), round(list[2],3),round(rot[0],3), round(rot[1],3), round(rot[2],3)], 0.5, 0.3)
        rtde_c.moveJ_IK([list[0],list[1], list[2],rot[0], rot[1], rot[2]], 0.5, 0.3)
        
        #标志位
        flag=0
        pose_flag.position.x=0
        pose_flag.position.y=0
        pose_flag.position.z=0
        pose_flag.orientation.x=0
        pose_flag.orientation.y=0
        pose_flag.orientation.z=0
        pose_flag.orientation.w=0
        move_flag=move_flag+1
        print("move_flag=",move_flag)
        # if(move_flag==3):
        #     rospy.signal_shutdown()
    if(move_flag==5):
        #夹爪闭合
        pub = rospy.Publisher('Robotiq2FGripperRobotOutput', outputMsg.Robotiq2FGripper_robot_output)
        command = outputMsg.Robotiq2FGripper_robot_output()
        sleep(2)
        for  i in range(10):
            command = genCommand(1,1,200,0,255,1)
            pub.publish(command)
            rospy.sleep(0.1)
        #放试管盖
        get_current_pose = rtde_r.getActualTCPPose()
        get_current_pose[2]=get_current_pose[2]+0.05
        rtde_c.moveL(get_current_pose,0.5,0.3)
        get_current_pose[1]=get_current_pose[1]-0.20
        rtde_c.moveL(get_current_pose,0.5,0.3)
        get_current_pose[2]=get_current_pose[2]-0.168
        rtde_c.moveL(get_current_pose,0.5,0.3)

        for  i in range(10):
            command = genCommand(1,1,80,0,255,100)
            pub.publish(command)
            rospy.sleep(0.1)
        rospy.signal_shutdown(move_flag==5)

        


if __name__ == "__main__":

    # 初始化ROS节点
    rospy.init_node('marker_move')

    #robotiq active
    pub = rospy.Publisher('Robotiq2FGripperRobotOutput', outputMsg.Robotiq2FGripper_robot_output)
    command = outputMsg.Robotiq2FGripper_robot_output()
    for  i in range(2):
        command = genCommand(0,0,0,0,0,0)
        pub.publish(command)
        rospy.sleep(0.1)
    rospy.sleep(2)
    for  i in range(2):
        command = genCommand(1,1,50,0,150,80)
        pub.publish(command)
        rospy.sleep(0.1)
    rospy.loginfo("robotiq active")

    #连接UR
    rtde_r = rtde_receive.RTDEReceiveInterface("192.168.3.102")
    rtde_c = RTDEControl("192.168.3.102", RTDEControl.FLAG_USE_EXT_UR_CAP)


    flag=0   ##求平均值累加次数
    move_flag=0
    pose_flag=Pose()  #定位移动次数
    print("start callback")
    
    #到达目标并定位
    # while move_flag<3:
    #     rospy.Subscriber("aruco_single/cammer_pose", Pose, callback,queue_size=1)
    rospy.Subscriber("aruco_single/cammer_pose", Pose, callback,queue_size=1)
    rospy.spin()
    #add_thread = threading.Thread(target = thread_spin)




lib.load_manifest('robotiq_2f_gripper_control')
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output  as outputMsg

    
def ro_rt_T(a, b, c, d, e, f, g):
    r = Ro.from_quat([d, e, f, g])
    rotation = r.as_matrix()
    translation = np.array([a, b, c])
    one = np.array([0, 0, 0, 1])
    t = np.concatenate(
        [rotation.reshape(3, 3), translation.reshape(3, 1)], axis=1)
    T = np.concatenate([t.reshape(3, 4), one.reshape(1, 4)], axis=0)
    return T


def translate2siyuan(matrix):

    end_rotation = matrix[0:3, 0:3]
    end_translation = matrix[0:3, 3]
    end_translation[0] = end_translation[0]
    r = Ro.from_matrix(end_rotation)
    end_rotation = r.as_quat()
    qvec_nvm = np.array(end_rotation)
    return end_translation, qvec_nvm


def send_matrix(tr_matrix, ro_matrix):
    str_my = ""

    for x in tr_matrix:
        str_my += str('%.4f' % x) + " "
    for y in ro_matrix:
        str_my += str('%.4f' % y) + " "
    # str_my += "end"
    str_my.encode('utf-8')
    return str_my

def quart2rot(data):
    a=Ro.from_quat(data)
    rot=a.as_rotvec()
    return rot

def rot2quart(data):
    a=Ro.from_rotvec(data)
    quart=a.as_quat()
    return quart

def genCommand(ACT,GTO,PR,ATR,SP,FR):
    command = outputMsg.Robotiq2FGripper_robot_output();
    command.rACT = ACT
    command.rGTO = GTO
    command.rPR = PR
    command.rATR = ATR
    command.rSP  = SP
    command.rFR  = FR
    return command


# def thread_spin():
#     rospy.spin()

def callback(data):


    global flag
    global pose_flag
    global move_flag
    pose_flag.position.x=pose_flag.position.x+data.position.x
    pose_flag.position.y=pose_flag.position.y+data.position.y
    pose_flag.position.z=pose_flag.position.z+data.position.z
    pose_flag.orientation.x=pose_flag.orientation.x+data.orientation.x
    pose_flag.orientation.y=pose_flag.orientation.y+data.orientation.y
    pose_flag.orientation.z=pose_flag.orientation.z+data.orientation.z
    pose_flag.orientation.w=pose_flag.orientation.w+data.orientation.w
    flag=flag+1

    if(flag==90):
        #mark旋转平移矩阵
        
        print("flag=",flag)
        #data_t = ro_rt_T(round(data.position.x,3), round(data.position.y,3),round(data.position.z,3), round(data.orientation.x,3),round(data.orientation.y,), round(data.orientation.z,3),round(data.orientation.w,3))
        data_t = ro_rt_T(pose_flag.position.x/90, pose_flag.position.y/90,pose_flag.position.z/90, pose_flag.orientation.x/90,pose_flag.orientation.y/90, pose_flag.orientation.z/90,pose_flag.orientation.w/90)
        #print("mark2cammer=",data)
    
        #current_pose旋转平移矩阵
        get_pose = rtde_r.getActualTCPPose()
        current_quart=rot2quart([get_pose[3],get_pose[4],get_pose[5]])
        current_pose=[get_pose[0],get_pose[1],get_pose[2],current_quart[0],current_quart[1],current_quart[2],current_quart[3]]
        eelink2baselink=ro_rt_T(current_pose[0],current_pose[1],current_pose[2],current_pose[3],current_pose[4],current_pose[5],current_pose[6])
        #print("get_pose=",get_pose)

        #cammer2eelink旋转平移矩阵
        cammer2eelink = np.array([[-0.999924876769970,	0.00339757983469717,	-0.0117769804206380,	0.0310828426656648],
        [-0.00327451116284631,	-0.999939999032199,	-0.0104535119552724,	0.0873993578886651],
        [-0.0118117904318327,	-0.0104141627999327,	0.999876005722765,	0.0175452134201781],
        [0,	0,	0,	1]])

        #设置目标mark2cammer
        mark2cammer = ro_rt_T(0.06+0.08,0.016+0.065,0.4-0.0675,1,0,0,0)
        #mark2cammer = ro_rt_T(0.06+0.08,0.016+0.065,0.5,1,0,0,0)

        #目标eelink2baselink_2放入list
        eelink2baselink_2=np.dot(np.dot(np.dot(np.dot(eelink2baselink,cammer2eelink),data_t),np.linalg.inv(mark2cammer)),np.linalg.inv(cammer2eelink))
        translations, rotation_matrix = translate2siyuan(eelink2baselink_2)
    
        str = send_matrix(translations, rotation_matrix)
        list = []
        list = str.split(' ')
        for i in range(6):
            list[i]=float(list[i])

        rot=quart2rot([list[3],list[4],list[5],list[6]])
        print("send_command=",[list[0],list[1], list[2],rot[0], rot[1], rot[2]])

        #移动机械臂
        #rtde_c.moveJ_IK([round(list[0],3),round(list[1],3), round(list[2],3),round(rot[0],3), round(rot[1],3), round(rot[2],3)], 0.5, 0.3)
        rtde_c.moveJ_IK([list[0],list[1], list[2],rot[0], rot[1], rot[2]], 0.5, 0.3)
        
        #标志位
        flag=0
        pose_flag.position.x=0
        pose_flag.position.y=0
        pose_flag.position.z=0
        pose_flag.orientation.x=0
        pose_flag.orientation.y=0
        pose_flag.orientation.z=0
        pose_flag.orientation.w=0
        move_flag=move_flag+1
        print("move_flag=",move_flag)
        # if(move_flag==3):
        #     rospy.signal_shutdown()
    if(move_flag==5):
        #夹爪闭合
        pub = rospy.Publisher('Robotiq2FGripperRobotOutput', outputMsg.Robotiq2FGripper_robot_output)
        command = outputMsg.Robotiq2FGripper_robot_output()
        sleep(2)
        for  i in range(10):
            command = genCommand(1,1,200,0,255,1)
            pub.publish(command)
            rospy.sleep(0.1)
        #放试管盖
        get_current_pose = rtde_r.getActualTCPPose()
        get_current_pose[2]=get_current_pose[2]+0.05
        rtde_c.moveL(get_current_pose,0.5,0.3)
        get_current_pose[1]=get_current_pose[1]-0.20
        rtde_c.moveL(get_current_pose,0.5,0.3)
        get_current_pose[2]=get_current_pose[2]-0.168
        rtde_c.moveL(get_current_pose,0.5,0.3)

        for  i in range(10):
            command = genCommand(1,1,80,0,255,100)
            pub.publish(command)
            rospy.sleep(0.1)
        rospy.signal_shutdown(move_flag==5)

        


if __name__ == "__main__":

    # 初始化ROS节点
    rospy.init_node('marker_move')

    #robotiq active
    pub = rospy.Publisher('Robotiq2FGripperRobotOutput', outputMsg.Robotiq2FGripper_robot_output)
    command = outputMsg.Robotiq2FGripper_robot_output()
    for  i in range(2):
        command = genCommand(0,0,0,0,0,0)
        pub.publish(command)
        rospy.sleep(0.1)
    rospy.sleep(2)
    for  i in range(2):
        command = genCommand(1,1,50,0,150,80)
        pub.publish(command)
        rospy.sleep(0.1)
    rospy.loginfo("robotiq active")

    #连接UR
    rtde_r = rtde_receive.RTDEReceiveInterface("192.168.3.102")
    rtde_c = RTDEControl("192.168.3.102", RTDEControl.FLAG_USE_EXT_UR_CAP)


    flag=0   ##求平均值累加次数
    move_flag=0
    pose_flag=Pose()  #定位移动次数
    print("start callback")
    
    #到达目标并定位
    # while move_flag<3:
    #     rospy.Subscriber("aruco_single/cammer_pose", Pose, callback,queue_size=1)
    rospy.Subscriber("aruco_single/cammer_pose", Pose, callback,queue_size=1)
    rospy.spin()
    #add_thread = threading.Thread(target = thread_spin)




