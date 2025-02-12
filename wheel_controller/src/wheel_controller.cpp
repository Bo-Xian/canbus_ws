#define WHEEL_CONTROLLER_VERSION 1.0

#include <wheel_controller/wheel_controller.h>

namespace wheel_controller {

WheelController::WheelController(ros::NodeHandle *nh){
    // 初始化參數
    params.yaml_path_           = ros::package::getPath("wheel_controller") + "/cfg/wheel_controller.yaml";
    params.yaml_node_            = YAML::LoadFile(params.yaml_path_);

    params.wheel_angle_offset_  = 0;  // degree

    // 顯示版本
    ROS_INFO_STREAM("WHEEL_CONTROLLER_VERSION " << WHEEL_CONTROLLER_VERSION);

    // 初始化各項功能
    while(!initPublisher(nh)){
        ROS_ERROR_STREAM("Publisher initialization failed!");
        ros::Duration(1.0).sleep();
    }
    ROS_INFO_STREAM("Publisher initialized successfully!");

    while(!initSubscriber(nh)){
        ROS_ERROR_STREAM("Subscriber initialization failed!");
        ros::Duration(1.0).sleep();
    }
    ROS_INFO_STREAM("Subscriber initialized successfully!");

    while(!initDynamicReconfigure()){
        ROS_ERROR_STREAM("Subscriber initialization failed!");
        ros::Duration(1.0).sleep();
    }
    ROS_INFO_STREAM("Dynamic Reconfigure initialized successfully!");

    ROS_INFO_STREAM("Start wheel controller node...");
}

WheelController::~WheelController(){}


bool WheelController::initPublisher(ros::NodeHandle *nh) {
    wheel_cmd_pub_ = nh->advertise<geometry_msgs::Twist>  ("canbus/set_wheel_cmd_vel", 1000);
    return true;
}

void WheelController::pubWheelControlCmd(const geometry_msgs::Twist cmd){
    ROS_INFO_STREAM("Cmd Val published");
    wheel_cmd_pub_.publish(cmd);
}

bool WheelController::initSubscriber(ros::NodeHandle *nh){
    cmd_vel_local_planner_sub_ = nh->subscribe("wheel_controller/set_cmd_vel_local_planner", 10, &WheelController::setCmdVelLocalPlannerCallback, this);
    return true;
}

void WheelController::setCmdVelLocalPlannerCallback(const geometry_msgs::Twist::ConstPtr& msg){
    ROS_INFO_STREAM("Cmd Val Callback");
    geometry_msgs::Twist cmd;
    cmd.linear.x = msg->linear.x;
    cmd.angular.z = msg->angular.z + params.wheel_angle_offset_ / 180.0 * M_PI;
    pubWheelControlCmd(cmd);
}

bool WheelController::initDynamicReconfigure(){
    cfg_Cb_ = boost::bind(&WheelController::dynamicReconfigureCallback, this, _1, _2);
    cfg_server_.setCallback(cfg_Cb_);
    return true;
}

void WheelController::dynamicReconfigureCallback(wheel_controller::wheelControllerParamConfig &config, uint32_t level){
    params.yaml_node_["wheel_angle_offset_"] = params.wheel_angle_offset_ = config.wheel_angle_offset_;
    ROS_INFO_STREAM("wheel_angle_offset_ " << params.wheel_angle_offset_);

    // 將參數存回yaml檔
    ROS_INFO_STREAM("Save Params to YAML");
    std::ofstream fout(params.yaml_path_);
    fout << params.yaml_node_;
}


};  //end namespace wheel_controller

int main(int argc, char **argv){
    ros::init(argc, argv, "wheel_controller_node");
    ros::NodeHandle nh;
    wheel_controller::WheelController wheelcontroller(&nh);
    ros::spin();
    return 0;
}
