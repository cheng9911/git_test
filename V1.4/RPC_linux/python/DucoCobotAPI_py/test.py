#! /usr/bin/env python
from email.policy import default
from pip import main
from sub1 import doMsg
import sub_topic 
from DucoCobot import DucoCobot
import numpy as np
def oo():
    print("error")
if __name__ == "__main__":
    #里面不能加括号504:sub_topic.doMsg1,
    # switch={502:oo,#测试
    #         503:doMsg,
    #         504:sub_topic.doMsg1
    #         }
    # choice=np.array([502,503,504])
    # for i in range(2):
    #     switch.get(choice[i],default)()
    pose=np.array([-0.690911,0.299594,0.5,0,0,0])
    pose1=pose.copy()
    pose2=pose.copy()
    pose3=pose.copy()
    pose4=pose.copy()
    pose1[2]=pose[2]+0.05
    pose2[2]=pose1[2]+0.05
    pose3[2]=pose2[2]+0.05
    pose4[2]=pose3[2]+0.05

    print(pose)    
    print(pose1)
    print(pose2)
    print(pose3)
    print(pose4)
    
    print("接收信息结束")