#ifndef WHEEL_CONTROLLER_H_
#define WHEEL_CONTROLLER_H_

// c++ library
#include <iostream>

// ros library
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>

// ros dynamic_reconfigure
#include <dynamic_reconfigure/server.h>
#include <wheel_controller/wheelControllerParamConfig.h>

// yaml
#include <fstream>
#include <ros/package.h>
#include <yaml-cpp/yaml.h>

using ParamConfig = wheel_controller::wheelControllerParamConfig;

namespace wheel_controller {

class WheelController{
public:
    WheelController(ros::NodeHandle *nh);
    ~WheelController();

private:

    bool initPublisher(ros::NodeHandle *nh);
    void pubWheelControlCmd(const geometry_msgs::Twist cmd);
    void pubEncoderCmd(const geometry_msgs::Twist cmd);

    bool initSubscriber(ros::NodeHandle *nh);
    void setCmdVelLocalPlannerCallback(const geometry_msgs::Twist::ConstPtr& msg);
    void getEncoderAngleCallback(const std_msgs::Float32::ConstPtr& msg);
    void getEncoderVelocityCallback(const std_msgs::Float32::ConstPtr& msg);

    bool initDynamicReconfigure();
    void dynamicReconfigureCallback(ParamConfig &config, uint32_t level);

    // Rostopic
    ros::Publisher wheel_cmd_pub_;
    ros::Subscriber cmd_vel_local_planner_sub_;

    ros::Publisher encoder_cmd_pub_;
    ros::Subscriber encoder_angle_sub_;
    ros::Subscriber encoder_velocity_sub_;

    // Dynamic reconfigure
    dynamic_reconfigure::Server<ParamConfig> cfg_server_;
    dynamic_reconfigure::Server<ParamConfig>::CallbackType cfg_Cb_;

    // YAML
    std::string yaml_path_;
    YAML::Node yaml_node_;

    struct parameters{

        float wheel_angle_offset_;
        float encoder_angle_;
        float encoder_velocity_;

    } params;

};

}; // end namespace wheel_controller

#endif // WHEEL_CONTROLLER_H_
