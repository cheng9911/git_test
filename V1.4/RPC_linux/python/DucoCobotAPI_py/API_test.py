#!/usr/bin/env python
import sys
import numpy as np
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

ip='192.168.1.10'

def main():
    
    
    duco_cobot = DucoCobot(ip,7003)

    #op = Op()

    # Connect!
    duco_cobot.open()
    duco_cobot.power_on(True)
    duco_cobot.enable(True)
    #duco_cobot.set_tool_data("api_tool",[0,0,0,0,0,0],[0,0,0,0])
    #duco_cobot.set_wobj("api_wobj",[0,0,0,0,0,0])
    #movej-----True 代表阻塞运行，返回值代表当前任务结束时的状态，False代表非阻塞运行，返回值代表任务的id
    # ret=duco_cobot.movej([0,0,-1.5708,0,1.5708,0],10,20,0,True)
    # print("movej ",ret)
    # movej2区别于movej的地方为其关节速度的单位为rad/s而不是百分比了
    # ret=duco_cobot.movej2([0,0,0,0,0,0],0.1,0.2,0,True)
    # print("movej ",ret)
    #运动movej_pose
    # ret=duco_cobot.movej_pose([-0.6909,0.2996,0.79521,0,0,0],10,10,0,[],"","",True)
    pose=np.array([-0.690911,0.299594,0.5,0,0,0])
    pose1=pose.copy()
    pose2=pose.copy()
    pose3=pose.copy()
    pose4=pose.copy()
    pose1[2]=pose[2]+0.05
    pose2[2]=pose1[2]+0.05
    pose3[2]=pose2[2]+0.05
    pose4[2]=pose3[2]+0.05
    ret=duco_cobot.spline([pose,pose1,pose2,pose3,pose4],0.1,0.1,"","",True)
    # ret=duco_cobot.movel([-0.690911,0.299594,0.6,0,0,0],0.1,0.1,0,[],"","",True)
    #ret=duco_cobot.movel([0.491,0.7274,0.286755,1.0533,-0.20245,-2.8770000],1,1,0,[],"","",True,op)
    #print("movel ",ret)
    
    #while 1:
    #    ret = 0
    #    state=duco_cobot.get_noneblock_taskstate(ret)
    #    print(state)
    #    time.sleep(1)
       
    # Close!
    duco_cobot.close()


if __name__ == '__main__':
    try:
        main()
    except Thrift.TException as tx:
        print('%s' % tx.message)
