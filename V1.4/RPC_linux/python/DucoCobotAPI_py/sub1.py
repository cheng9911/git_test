#! /usr/bin/env python
"""
    需求: 实现基本的话题通信，一方发布数据，一方接收数据，
         实现的关键点:
         1.发送方
         2.接收方
         3.数据(此处为普通文本)


    消息订阅方:
        订阅话题并打印接收到的消息

    实现流程:
        1.导包 
        2.初始化 ROS 节点:命名(唯一)
        3.实例化 订阅者 对象
        4.处理订阅的消息(回调函数)
        5.设置循环调用回调函数



"""
#1.导包 
import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray

def doMsg():
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
            
        print(array)#接受利用数据
        i=i+1
        if i==10:
            flag=0
    
if __name__ == "__main__":
    #2.初始化 ROS 节点:命名(唯一)
    
    #3.实例化 订阅者 对象
    #sub = rospy.Subscriber("chatter",String,doMsg,queue_size=10)
    #4.处理订阅的消息(回调函数)
    #5.设置循环调用回调函数
    doMsg()
    print("接收信息结束")