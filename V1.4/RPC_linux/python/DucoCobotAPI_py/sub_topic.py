#! /usr/bin/env python
import sys
import glob
import time
#import _thread
import threading
from DucoCobot import DucoCobot
sys.path.append('gen_py')
sys.path.append('lib')
from thrift import Thrift
from thrift.transport import TSocket
from thrift.transport import TTransport
from thrift.protocol import TBinaryProtocol
from gen_py.robot.ttypes import StateRobot, StateProgram, OperationMode,TaskState,Op
import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray
def thread_fun():
    duco_cobot = DucoCobot(ip,7003)
    # Connect!
    duco_cobot.open()
    rospy.init_node("talker")
    pub = rospy.Publisher("joint_position",Float64MultiArray,queue_size=1)
    rate=rospy.Rate(10)
    while not rospy.is_shutdown():
        tcp_pose=duco_cobot.get_actual_joints_position()
        msg=Float64MultiArray(data=tcp_pose)
        pub.publish(msg)
        rate.sleep()
        print("pos: ", msg)
        time.sleep(1)

def doMsg1():
    #测试后期删除
    ip='192.168.1.10'
    duco_cobot = DucoCobot(ip,7003)
    duco_cobot.open()
    duco_cobot.power_on(True)
    duco_cobot.enable(True)
    ###
    array=np.array([0,0,0,0,0,0])
    rospy.init_node("listener_p")
    #rospy.loginfo("I heard:%s",msg.data)
    i=0
    flag=1
    while flag:
        #rospy.sleep(2)
        dat=rospy.wait_for_message("chatter",Float64MultiArray)
        for j in range(6):
            array[j]=dat.data[j]
        print(array)#接收利用数据
        ret=duco_cobot.movej(array,10,20,0,True)
        print("movej ",ret)
        ret=duco_cobot.movej([0.2,0,0,0,0,0],10,20,0,True)
    
        print("movej ",ret)
        i=i+1
        if i==2:
            flag=0
    
if __name__ == "__main__":
    #2.初始化 ROS 节点:命名(唯一)
    
    #3.实例化 订阅者 对象
    #sub = rospy.Subscriber("chatter",String,doMsg,queue_size=10)
    #4.处理订阅的消息(回调函数)
    #5.设置循环调用回调函数
    ip='192.168.1.10'
    duco_cobot = DucoCobot(ip,7003)
    duco_cobot.open()
    duco_cobot.power_on(True)
    duco_cobot.enable(True)
    #thd_A = threading.Thread(target=thread_fun)
    #thd_A.start()
    doMsg1()
    print("接收信息结束")