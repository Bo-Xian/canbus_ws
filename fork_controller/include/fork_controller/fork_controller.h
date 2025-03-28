#ifndef FORK_CONTROLLER_H_
#define FORK_CONTROLLER_H_

// c++ library
#include <string>
#include <vector>
#include <queue>
#include <thread>
#include <iostream>

// ros library
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>

// ros dynamic_reconfigure
#include <dynamic_reconfigure/server.h>
#include <fork_controller/forkControllerParamConfig.h>

// yaml
#include <fstream>
#include <ros/package.h>
#include <yaml-cpp/yaml.h>

using ParamConfig = fork_controller::forkControllerParamConfig;

namespace fork_controller {

class ForkController{
public:
    ForkController(ros::NodeHandle *nh);
    ~ForkController();

private:

    bool initPublisher(ros::NodeHandle *nh);
    void pubForkControlSpeed(float speed);
    void pubForkHight(float hight);

    bool initSubscriber(ros::NodeHandle *nh);
    void getForkHightCallback(const std_msgs::Float32::ConstPtr& msg);
    void setForkHightCallback(const std_msgs::Float32::ConstPtr& msg);

    bool initDynamicReconfigure();
    void dynamicReconfigureCallback(ParamConfig &config, uint32_t level);

    void forkControlThread();

    float forkHightUpwardPid(float error);
    float forkHightDownwardPid(float error);

    // Rostopic
    ros::Publisher fork_control_pub_;
    ros::Publisher fork_hight_pub_;

    ros::Subscriber fork_hight_sub_;
    ros::Subscriber target_hight_sub_;

    // YAML
    std::string yaml_path_;
    YAML::Node yaml_node_;

    // Dynamic reconfigure
    dynamic_reconfigure::Server<ParamConfig> cfg_server_;
    dynamic_reconfigure::Server<ParamConfig>::CallbackType cfg_Cb_;

    // Thread
    std::atomic<bool> stop_thread_;
    std::thread fork_thread_;

    struct parameters{
        int ros_rate_;

        float fork_hight_upper_limit_;
        float fork_hight_lower_limit_;

        float upward_min_speed_;
        float downward_min_speed_;

        float current_hight_;
        float target_hight_;
        float total_travel_;

        float fork_hight_tolerance_;
        float fork_hight_offset_a_;  // y = ax + b
        float fork_hight_offset_b_;  // y = ax + b

        float upward_prev_error_;
        float upward_integral_;
        float upward_kp_;
        float upward_ki_;
        float upward_kd_;

        float downward_prev_error_;
        float downward_integral_;
        float downward_kp_;
        float downward_ki_;
        float downward_kd_;

    } params;

};

}; // end namespace fork_controller

#endif // FORK_CONTROLLER_H_
