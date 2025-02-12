#ifndef WHEEL_CONTROLLER_H_
#define WHEEL_CONTROLLER_H_

// c++ library
#include <iostream>

// ros library
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

// ros dynamic_reconfigure
#include <dynamic_reconfigure/server.h>
#include <wheel_controller/wheelControllerParamConfig.h>

// yaml
#include <fstream>
#include <ros/package.h>
#include <yaml-cpp/yaml.h>


namespace wheel_controller {

class WheelController{
public:
    WheelController(ros::NodeHandle *nh);
    ~WheelController();

private:

    bool initPublisher(ros::NodeHandle *nh);
    void pubWheelControlCmd(const geometry_msgs::Twist cmd);

    bool initSubscriber(ros::NodeHandle *nh);
    void setCmdVelLocalPlannerCallback(const geometry_msgs::Twist::ConstPtr& msg);

    bool initDynamicReconfigure();
    void dynamicReconfigureCallback(wheel_controller::wheelControllerParamConfig &config, uint32_t level);

    ros::Publisher wheel_cmd_pub_;

    ros::Subscriber cmd_vel_local_planner_sub_;

    dynamic_reconfigure::Server<wheel_controller::wheelControllerParamConfig> cfg_server_;
    dynamic_reconfigure::Server<wheel_controller::wheelControllerParamConfig>::CallbackType cfg_Cb_;

    struct parameters{
        std::string yaml_path_;
        YAML::Node yaml_node_;

        float wheel_angle_offset_;
    } params;

};

}; // end namespace wheel_controller

#endif // WHEEL_CONTROLLER_H_
