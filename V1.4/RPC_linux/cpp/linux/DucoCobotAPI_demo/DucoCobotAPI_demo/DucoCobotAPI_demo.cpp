#include <thread>
#include <iostream>

#ifdef _WIN32
#include <windows.h>
#else
#include<unistd.h>
#endif
#include "DucoCobot.h"

using namespace std;

std::string ip = "192.168.12.96";

struct RealTimeData rt_data;


void thread_fun()
{
    DucoCobot duco_cobot(ip, 7003);
    try
    {
        duco_cobot.open();
        while (1)
        {
            std::vector<double> joints;
            duco_cobot.get_actual_joints_position(joints);
//            std::cout <<"current joinsts pos is:";
//            for (unsigned int i = 0; i<joints.size(); i++)
//            {
//                std::cout <<joints[i] << " ";
//            }
//            std::cout << endl;
#ifdef _WIN32
            Sleep(1000);
#else
            usleep(1000);
#endif
        }
    }
    catch (...)
    {
        std::cout << "thread error!" << std::endl;
        duco_cobot.close();
    }
}

RealTimeData get_data()
{
    for (int i = 0; i < 10; i++) {
        rt_data.data[i] = 0.0001;
    }
    rt_data.mode = 0;

    return rt_data;
}


int main(int argc, char *argv[])
{
    OP op;
    op.trig_io_1 = 5;
    op.trig_io_2 = 0;
    op.trig_dist_1 = 0;
    op.trig_dist_2 = 0;
    op.trig_time_1 = 3;
    op.trig_time_2 = 0;
    op.trig_value_1 = true;
    op.trig_value_2 = false;
    op.time_or_dist_1 = 1;
    op.time_or_dist_2 = 0;

    DucoCobot duco_cobot(ip, 7003);
    std::thread thread_state(thread_fun);
    try
    {
        int result = duco_cobot.open();
        cout << "open: " << result << endl;
        int nResult = duco_cobot.power_on(true);
        cout << "power_on: " << result << endl;
//        nResult = duco_cobot.enable(true);

//        vector<double> tcp_offset;
//        duco_cobot.get_tcp_offset(tcp_offset);
//        cout << tcp_offset[0] << " " << tcp_offset[1] << " " << tcp_offset[2] << " " << tcp_offset[3] << " " << tcp_offset[4] << " " << tcp_offset[5] << endl;
//        vector<double> wobj_offset(6, 0.1);
//        cout << "set_wobj_offset: " << duco_cobot.set_wobj_offset(wobj_offset)<< endl;
//        duco_cobot.collision_detect(0);

//        nResult = duco_cobot.start_realtime_mode();
//        cout << "start_realtime_mode: " << nResult << endl;
//        nResult = duco_cobot.start_realtime_thread(get_data);

//        getchar();

//        nResult = duco_cobot.end_realtime_mode();
//        cout << "end_realtime_mode: " << nResult << endl;

        std::vector<double> joints;
        joints.push_back(0);
        joints.push_back(0);
        joints.push_back(-1.5708);
        joints.push_back(0);
        joints.push_back(1.5708);
        joints.push_back(0);

        nResult = duco_cobot.enable_speed_optimization(false);
        std::cout << "enable_speed_optimization: " << nResult << std::endl;
        nResult = duco_cobot.movej(joints, 30, 90, 0, true,op);
        std::cout << "movej: " << nResult << std::endl;
        nResult = duco_cobot.disable_speed_optimization(false);
        std::cout << "disable_speed_optimization: " << nResult << std::endl;
//        joints.push_back(0);
//        nResult = duco_cobot.servoj(joints,0.5,5,200,20);
//        std::cout << "servoj: " << nResult << std::endl;
        std::vector<double> pose;
        pose.push_back(-0.492000);
        pose.push_back(0.140500);
        pose.push_back(0.568415);
        pose.push_back(-3.141592);
        pose.push_back(0);
        pose.push_back(1.570796);

//        nResult = duco_cobot.movel(pose, 1, 1, 0, vector<double>(), "", "", true, op, 0);
//        std::cout << "movel ret: " << nResult << std::endl;
        vector<double> ps1 = {-0.480348,0.262296,0.422482,3.141589,0.000004,1.570796};
        vector<double> ps2 = {-0.480348,-0.022912,0.348405,3.141589,0.000004,1.570796};
        vector<double> near_ = {0.344151,0.042322,-1.882206,0.269085,1.570801,0.344151};
//        std::cout << "move_circle ret: " << duco_cobot.move_circle(ps1, ps2, 0.1, 1, 0, 0, near_, "", "", true, op);
        std::cout << endl;
        vector<double> tmp;
//        duco_cobot.get_actual_joints_torque(tmp);
//        cout << tmp.size() << endl;
//        cout << tmp[0] << " " << tmp[1] << " " << tmp[2] << " " << tmp[3] << " " << tmp[4] << " " << tmp[5] << endl;
//        tmp.clear();
//        duco_cobot.get_target_joints_torque(tmp);
//        cout << tmp.size() << endl;
//        cout << tmp[0] << " " << tmp[1] << " " << tmp[2] << " " << tmp[3] << " " << tmp[4] << " " << tmp[5] << endl;
//        tmp.clear();
//        tmp.push_back(5);
//        tmp.push_back(0.3);
//        tmp.push_back(0.3);
//        tmp.push_back(0.3);
//        cout << "set_load_data: " << duco_cobot.set_load_data(tmp) << endl;
//        tmp.clear();
//        sleep(1);
//        duco_cobot.get_tool_load(tmp);
//        cout << "load: " << tmp[0] << " " << tmp[1] << " " << tmp[2] << " " << tmp[3] << endl;
//        tmp.clear();


        getchar();
    }
    catch (...)
    {
        std::cout << "main error!" << std::endl;
//        duco_cobot.close();
    }
}

