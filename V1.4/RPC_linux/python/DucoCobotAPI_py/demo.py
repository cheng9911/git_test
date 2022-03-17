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

ip='192.168.1.10'
def thread_fun():
    duco_cobot = DucoCobot(ip,7003)
    # Connect!
    duco_cobot.open()
    while 1:
        tcp_pose=duco_cobot.get_actual_joints_position()
        print("pos: ", tcp_pose)
        time.sleep(1)

def main():
    #_thread.start_new_thread(thread_fun,())
    thd_A = threading.Thread(target=thread_fun)
    thd_A.start()
    duco_cobot = DucoCobot(ip,7003)

    #op = Op()

    # Connect!
    duco_cobot.open()
    duco_cobot.power_on(True)
    duco_cobot.enable(True)
    #duco_cobot.set_tool_data("api_tool",[0,0,0,0,0,0],[0,0,0,0])
    #duco_cobot.set_wobj("api_wobj",[0,0,0,0,0,0])
    ret=duco_cobot.movej([0,0,-1.5708,0,1.5708,0],10,20,0,True)
    print("movej ",ret)
    ret=duco_cobot.movej([0,0,0,0,0,0],10,20,0,True)
    print("movej ",ret)
    #ret=duco_cobot.movel([0.491,0.7274,0.286755,1.0533,-0.20245,-2.8770000],1,1,0,[],"","",True,op)
    #print("movel ",ret)
    ret=duco_cobot.servoj([0,0,-1.5708,0,1.5708,0,0],0.5,5)
    print(ret)
    #while 1:
    #    ret = 0
    #    state=duco_cobot.get_noneblock_taskstate(ret)
    #    print(state)
    #    time.sleep(1)
       
    # Close!
    #duco_cobot.close()


if __name__ == '__main__':
    try:
        main()
    except Thrift.TException as tx:
        print('%s' % tx.message)
