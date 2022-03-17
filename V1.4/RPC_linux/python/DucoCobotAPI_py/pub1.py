#! /usr/bin/env python
"""
    需求: 实现基本的话题通信，一方发布数据，一方接收数据，
         实现的关键点:
         1.发送方
         2.接收方
         3.数据(此处为普通文本)

         PS: 二者需要设置相同的话题


    消息发布方:
        循环发布信息:HelloWorld 后缀数字编号

    实现流程:
        1.导包 
        2.初始化 ROS 节点:命名(唯一)
        3.实例化 发布者 对象
        4.组织被发布的数据，并编写逻辑发布数据


"""
#1.导包 
#import string
import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray

if __name__ == "__main__":
    #2.初始化 ROS 节点:命名(唯一)
    rospy.init_node("talker_p")
    #3.实例化 发布者 对象
    pub = rospy.Publisher("chatter",Float64MultiArray,queue_size=1)
    #4.组织被发布的数据，并编写逻辑发布数据
     #创建 msg 对象
    msg_front = "hello 你好"
    count = 0  #计数器 
    # 设置循环频率
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():

        #拼接字符串
        array=np.array([0,0,0,0,0,0])

        msg=Float64MultiArray(data=array)
        

        pub.publish(msg)
        rate.sleep()
        rospy.loginfo(msg)
        count += 1
