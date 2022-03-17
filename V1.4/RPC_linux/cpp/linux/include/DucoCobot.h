#ifndef _DUCOCOBOTRPC
#define _DUCOCOBOTRPC 
#include <string>
#include <vector>
#include <memory>
#include <thread>

namespace apache {
	namespace thrift {
		namespace transport {
			class TSocket;
			class TTransport;
		}
	}
}
namespace apache {
	namespace thrift {
		namespace protocol {
			class TProtocol;
		}
	}
}
class RPCRobotClient;

/**
 * @brief

enum StateRobot {
    SR_Start = 0,
    SR_Initialize = 1,
    SR_Logout = 2,
    SR_Login = 3,
    SR_PowerOff = 4,
    SR_Disable = 5,
    SR_Enable = 6
};
 *
 */

/**
 * @brief
 *
enum StateProgram {
    SP_Stopped = 0,
    SP_Stopping = 1,
    SP_Running = 2,
    SP_Paused = 3,
    SP_Pausing = 4
};
 *
 */

/**
 * @brief
enum OperationMode {
    kManual = 0,
    kAuto = 1,
    kRemote = 2
};

 */

/**
 * @brief
enum TaskState {
    ST_Idle = 0,
    ST_Running = 1,
    ST_Paused = 2,
    ST_Stopped = 3,
    ST_Finished = 4,
    ST_Interrupt = 5,
    ST_Error = 6,
    ST_Illegal = 7,
    ST_ParameterMismatch = 8
};

 */

/**
 * @brief
enum SafetyState {
    SS_INIT = 0,
    SS_WAIT = 2,
    SS_CONFIG = 3,
    SS_POWER_OFF = 4,
    SS_RUN = 5,
    SS_RECOVERY = 6,
    SS_STOP2 = 7,
    SS_STOP1 = 8,
    SS_STOP0 = 9,
    SS_MODEL = 10,
    SS_REDUCE = 12,
    SS_BOOT = 13,
    SS_FAIL = 14,
    SS_UPDATE = 99
};
 */

struct RealTimeData
{
    float data[10];
    uint32_t mode;
};


struct OP{
    char time_or_dist_1;
    char trig_io_1;
    bool trig_value_1;
    double trig_time_1;
    double trig_dist_1;
    char time_or_dist_2;
    char trig_io_2;
    bool trig_value_2;
    double trig_time_2;
    double trig_dist_2;
};


typedef RealTimeData (*callbackfun)();

class DucoCobot
{
public:

	DucoCobot(std::string ip, unsigned int port);

    int32_t open();

    int32_t close();

    int32_t power_on(const bool block) ;

    int32_t power_off(const bool block) ;

    int32_t enable(const bool block) ;

    int32_t disable(const bool block) ;

    int32_t shutdown(const bool block) ;

    int32_t stop(const bool block) ;

    int32_t pause(const bool block) ;

    int32_t resume(const bool block) ;

    int32_t run_program(const std::string& name, const bool block) ;

    int32_t set_tool_data(const std::string& name, const std::vector<double> & tool_offset, const std::vector<double> & payload, const std::vector<double>& inertia_tensor) ;

    void get_tool_load(std::vector<double> & _return) ;

    void get_tcp_offset(std::vector<double> & _return) ;

    int32_t set_wobj(const std::string& name, const std::vector<double> & wobj) ;

    void get_wobj(std::vector<double> & _return) ;

    int32_t set_wobj_offset(const std::vector<double> & wobj) ;

    void cal_fkine(std::vector<double> & _return, const std::vector<double> & joints_position, const std::vector<double> & tool, const std::vector<double> & wobj) ;

    void cal_ikine(std::vector<double> & _return, const std::vector<double> & p, const std::vector<double> & q_near, const std::vector<double> & tool, const std::vector<double> & wobj) ;

    int32_t set_standard_digital_out(const int16_t num, const bool value, const bool block) ;

    int32_t set_tool_digital_out(const int16_t num, const bool value, const bool block) ;

    bool get_standard_digital_in(const int16_t num) ;

    bool get_tool_digital_in(const int16_t num) ;

    bool get_config_digital_in(const int16_t num) ;

    double get_standard_analog_voltage_in(const int16_t num) ;

    double get_tool_analog_voltage_in(const int16_t num) ;

    double get_standard_analog_current_in(const int16_t num) ;

    bool get_function_digital_in(const int16_t num) ;

    bool get_function_digital_out(const int16_t num) ;

    void read_raw_data_485(std::vector<int8_t> & _return, const int32_t len) ;

    void read_raw_data_485_ht(std::vector<int8_t> & _return, const std::vector<int8_t> & head, const std::vector<int8_t> & tail) ;

    void read_raw_data_485_h(std::vector<int8_t> & _return, const std::vector<int8_t> & head, const int32_t len) ;

    bool write_raw_data_485(const std::vector<int8_t> & data) ;

    bool write_raw_data_485_h(const std::vector<int8_t> & data, const std::vector<int8_t> & head) ;

    bool write_raw_data_485_ht(const std::vector<int8_t> & data, const std::vector<int8_t> & head, const std::vector<int8_t> & tail) ;

    void tool_read_raw_data_485(std::vector<int8_t> & _return, const int32_t len) ;

    void tool_read_raw_data_485_h(std::vector<int8_t> & _return, const std::vector<int8_t> & head, const int32_t len) ;

    void tool_read_raw_data_485_ht(std::vector<int8_t> & _return, const std::vector<int8_t> & head, const std::vector<int8_t> & tail) ;

    bool tool_write_raw_data_485(const std::vector<int8_t> & data) ;

    bool tool_write_raw_data_485_h(const std::vector<int8_t> & data, const std::vector<int8_t> & head) ;

    bool tool_write_raw_data_485_ht(const std::vector<int8_t> & data, const std::vector<int8_t> & head, const std::vector<int8_t> & tail) ;

    void read_raw_data_can(std::vector<int8_t> & _return) ;

    bool write_raw_data_can(const int32_t id, const std::vector<int8_t> & data) ;

    bool read_bool_reg(const int16_t num) ;

    int32_t read_word_reg(const int16_t num) ;

    double read_float_reg(const int16_t num) ;

    int32_t write_bool_reg(const int16_t num, const bool value) ;

    int32_t write_word_reg(const int16_t num, const int32_t value) ;

    int32_t write_float_reg(const int16_t num, const double value) ;

    int32_t movej(const std::vector<double> & joints_list, const double v, const double a, const double r, const bool block, const OP &op = op_) ;

    int32_t movej2(const std::vector<double> & joints_list, const double v, const double a, const double r, const bool block, const OP &op = op_) ;

    int32_t movej_pose(const std::vector<double> & p, const double v, const double a, const double r, const std::vector<double> & q_near, const std::string& tool, const std::string& wobj, const bool block, const OP &op = op_) ;

    int32_t movej_pose2(const std::vector<double> & p, const double v, const double a, const double r, const std::vector<double> & q_near, const std::string& tool, const std::string& wobj, const bool block, const OP &op = op_) ;

    int32_t movel(const std::vector<double> & p, const double v, const double a, const double r, const std::vector<double> & q_near, const std::string& tool, const std::string& wobj, const bool block, const OP &op = op_) ;

    int32_t movec(const std::vector<double> & p1, const std::vector<double> & p2, const double v, const double a, const double r, const bool mode, const std::vector<double> & q_near, const std::string& tool, const std::string& wobj, const bool block, const OP &op = op_) ;

    int32_t move_circle(const std::vector<double> & p1, const std::vector<double> & p2, const double v, const double a, const double r, const bool mode, const std::vector<double> & q_near, const std::string& tool, const std::string& wobj, const bool block, const OP &op = op_) ;

    int32_t tcp_move(const std::vector<double> & pose_offset, const double v, const double a, const double r, const bool block, const OP &op = op_) ;

    int32_t tcp_move_2p(const std::vector<double> & p1, const std::vector<double> & p2, const double v, const double a, const double r, const bool block, const OP &op = op_) ;

    int32_t spline(const std::vector<std::vector<double> > & pose_list, const double v, const double a, const std::string& tool, const std::string& wobj, const bool block, const OP &op = op_) ;

    int32_t speedj(const std::vector<double> & joints_list, const double a, const int32_t time, const bool block) ;

    int32_t speedl(const std::vector<double> & pose_list, const double a, const int32_t time, const bool block) ;

    int32_t speed_stop(const bool block) ;

    int32_t servoj(const std::vector<double> & joints_list, const double v, const double a, const double kp = 200, const double kd = 25) ;

    int32_t servoj_pose(const std::vector<double> & pose_list, const double v, const double a, const std::vector<double> & q_near, const std::string& tool, const std::string& wobj, const double kp = 200, const double kd = 25) ;

    int32_t teach_mode(const bool block) ;

    int32_t end_teach_mode(const bool block) ;

    int32_t modbus_add_signal(const std::string& ip, const int32_t slave_number, const int32_t signal_address, const int32_t signal_type, const std::string& signal_name) ;

    int32_t modbus_delete_signal(const std::string& signal_name) ;

    int32_t modbus_read(const std::string& signal_name) ;

    int32_t modbus_write(const std::string& signal_name, const int32_t value) ;

    void modbus_set_frequency(const std::string& signal_name, const int32_t frequence) ;

    void get_last_error(std::vector<std::string> & _return) ;

    int32_t get_noneblock_taskstate(const int32_t id) ;

    void log_info(const std::string& message) ;

    void log_error(const std::string& message) ;

    int32_t simulation(const bool sim, const bool block) ;

    int32_t speed(const double val) ;

    void get_robot_state(std::vector<int8_t> & _return) ;

    void get_flange_pose(std::vector<double> & _return) ;

    void get_flange_speed(std::vector<double> & _return) ;

    void get_flange_acceleration(std::vector<double> & _return) ;

    void get_tcp_pose(std::vector<double> & _return) ;

    void get_tcp_speed(std::vector<double> & _return) ;

    void get_tcp_acceleration(std::vector<double> & _return) ;

    void get_tcp_force(std::vector<double> & _return) ;

    void get_actual_joints_position(std::vector<double> & _return) ;

    void get_target_joints_position(std::vector<double> & _return) ;

    void get_actual_joints_speed(std::vector<double> & _return) ;

    void get_target_joints_speed(std::vector<double> & _return) ;

    void get_actual_joints_acceleration(std::vector<double> & _return) ;

    void get_target_joints_acceleration(std::vector<double> & _return) ;

    void get_actual_joints_torque(std::vector<double> & _return) ;

    void get_target_joints_torque(std::vector<double> & _return) ;

    int32_t stop_record_track() ;

    int32_t start_record_track(const std::string& name) ;

    int32_t collision_detect(const int32_t value) ;

    int32_t replay(const std::string& name, const int32_t value) ;

    int32_t set_load_data(const std::vector<double> & value) ;

    int32_t fc_start() ;

    int32_t fc_stop() ;

    int32_t fc_config(const std::vector<bool> & direction, const std::vector<double> & ref_ft, const std::vector<double> & damp, const std::vector<double> & max_vel, const std::vector<double> & number_list, const std::string& toolname, const std::string& wobjname, const int32_t value) ;

    int32_t fc_move() ;

    int32_t fc_guard_act(const std::vector<bool> & direction, const std::vector<double> & ref_ft, const std::string& toolname, const std::string& wobjname, const int32_t type) ;

    int32_t fc_guard_deact() ;

    int32_t fc_force_set_value(const std::vector<bool> & direction, const std::vector<double> & ref_ft) ;

    int32_t fc_wait_pos(const std::vector<double> & middle, const std::vector<double> & range, const bool absolute, const int32_t duration, const int32_t timeout) ;

    int32_t fc_wait_vel(const std::vector<double> & middle, const std::vector<double> & range, const bool absolute, const int32_t duration, const int32_t timeout) ;

    int32_t fc_wait_ft(const std::vector<double> & middle, const std::vector<double> & range, const bool absolute, const int32_t duration, const int32_t timeout) ;

    int32_t fc_wait_logic(const std::vector<int32_t> & value) ;

    void fc_get_ft(std::vector<double> & _return) ;

    bool fc_mode_is_active() ;

    int32_t start_realtime_mode();

    int32_t end_realtime_mode();

    int32_t start_realtime_thread(callbackfun callback);

    void end_realtime_thread();

    int32_t enable_speed_optimization(const bool block);

    int32_t disable_speed_optimization(const bool block);

private:
    std::shared_ptr<apache::thrift::transport::TSocket> socket_;
	std::shared_ptr<apache::thrift::transport::TTransport> transport;
	std::shared_ptr<apache::thrift::protocol::TProtocol> protocol;
	RPCRobotClient *robot;
    static struct OP op_;
};

#endif
