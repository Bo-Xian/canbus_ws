#ifndef CANBUS_DRIVER_H_
#define CANBUS_DRIVER_H_

// c++ library
#include <string>
#include <vector>
#include <queue>
#include <thread>
#include <iostream>
#include <cstring>
#include <cerrno>
#include <bitset> // show data in binary
// canbus
#include <PCANBasic.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <unistd.h>

// ros library
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>


namespace canbus_driver {

typedef struct {
    // SOF ID RTR Control Data CRC ACK EOF
    uint16_t can_id;
    uint8_t can_dlc;
    uint8_t msg_type;
    uint8_t can_data[8] = {0};
} CanBusCmd;

namespace DeviceID {
    // send to STM Board
    constexpr uint16_t Stm0x777     = 0x0777;

    // recive from STM Board
    constexpr uint16_t Stm0x778     = 0x0778;

    // send to Curtis
    constexpr uint16_t Curtis0x203  = 0x0203;
    constexpr uint16_t Curtis0x303  = 0x0303;

    // recive from Curtis
    constexpr uint16_t Curtis0x183  = 0x0183;
    constexpr uint16_t Curtis0x283  = 0x0283;
    constexpr uint16_t Curtis0x383  = 0x0383;

};

class CanBusDriver{
public:
    CanBusDriver(ros::NodeHandle *nh);
    ~CanBusDriver();

private:

    bool initCanBus(TPCANHandle handler, uint16_t baud_rate);

    bool initPublisher(ros::NodeHandle *nh);
    void pubErrorMessages(std::string errorMsg);
    void pubWheelVelocity(float velocity);
    void pubWheelAngle(float angle);
    void pubMotorCurrent(float motorCurrent);
    void pubMotorDriverTemperature(float motorDriverTemperature);

    bool initSubscriber(ros::NodeHandle *nh);
    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);
    void forkControlSpeedCallback(const std_msgs::Float32::ConstPtr& msg);
    void wheelAccelerationRatioCallback(const std_msgs::Float32::ConstPtr& msg);
    void wheelDecelerationRatioCallback(const std_msgs::Float32::ConstPtr& msg);
    void wheelAngleOffsetCallback(const std_msgs::Float32::ConstPtr& msg);
    void pubForkHight(float hight);

    void addMsgToQueueThread();
    void transmitMsgThread();
    void receiveMsgThread();

    void handleMsg183(unsigned char length, unsigned char data[8]);
    void handleMsg283(unsigned char length, unsigned char data[8]);
    void handleMsg383(unsigned char length, unsigned char data[8]);
    void handleMsg778(unsigned char length, unsigned char data[8]);

    TPCANHandle pcan_handle_;

    uint16_t can_baud_rate_;

    std::queue<CanBusCmd> msg_queue_;

    std::mutex msg_queue_mutex_;
    std::mutex update_curtis_mutex_;

    ros::Publisher wheel_velocity_pub_;
    ros::Publisher wheel_angle_pub_;
    ros::Publisher error_message_pub_;
    ros::Publisher motor_current_pub_;
    ros::Publisher motor_driver_temperature_pub_;
    ros::Publisher fork_hight_in_cm_pub_;

    ros::Subscriber wheel_cmd_vel_sub_;
    ros::Subscriber fork_control_speed_sub_;
    ros::Subscriber wheel_acceleration_sub_;
    ros::Subscriber wheel_deceleration_sub_;

    uint8_t msg_data_203_[8];
    uint8_t msg_data_303_[8];

    struct parameters{

        int add_msg_rate_;

        short receive_wheel_angle_upper_limit_;
        short receive_wheel_angle_lower_limit_;
        short receive_wheel_speed_upper_limit_;
        short receive_wheel_speed_lower_limit_;
        short receive_motor_temperature_upper_limit_;
        short receive_motor_temperature_lower_limit_;
        short receive_moter_current_upper_limit_;

        short receive_fork_hight_upper_limit_;
        short receive_fork_hight_lower_limit_;

        float wheel_radius_;
        float wheel_reduction_ratio_;

    } params;

};

}; // end namespace canbus_driver

#endif // CANBUS_DRIVER_H_
