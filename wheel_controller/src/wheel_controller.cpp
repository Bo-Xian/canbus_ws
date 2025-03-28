#define WHEEL_CONTROLLER_VERSION 1.0

#include <wheel_controller/wheel_controller.h>

namespace wheel_controller {

WheelController::WheelController(ros::NodeHandle *nh):
    yaml_path_(ros::package::getPath("wheel_controller") + "/cfg/wheel_controller.yaml"),
    yaml_node_(YAML::LoadFile(yaml_path_))
{
    // 初始化參數
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
    wheel_cmd_pub_      = nh->advertise<geometry_msgs::Twist>  ("canbus_driver/set_wheel_cmd_vel", 1000);
    encoder_cmd_pub_    = nh->advertise<geometry_msgs::Twist>  ("wheel_controller/get_encoder_cmd_vel", 1000);
    return true;
}

void WheelController::pubWheelControlCmd(const geometry_msgs::Twist cmd){
    wheel_cmd_pub_.publish(cmd);
}

void WheelController::pubEncoderCmd(const geometry_msgs::Twist cmd){
    encoder_cmd_pub_.publish(cmd);
}

bool WheelController::initSubscriber(ros::NodeHandle *nh){
    cmd_vel_local_planner_sub_  = nh->subscribe("wheel_controller/set_cmd_vel_local_planner",   10, &WheelController::setCmdVelLocalPlannerCallback, this);
    encoder_angle_sub_          = nh->subscribe("canbus_driver/get_wheel_angle",                10, &WheelController::getEncoderAngleCallback, this);
    encoder_velocity_sub_       = nh->subscribe("canbus_driver/get_wheel_velocity",             10, &WheelController::getEncoderVelocityCallback, this);
    return true;
}

void WheelController::setCmdVelLocalPlannerCallback(const geometry_msgs::Twist::ConstPtr& msg){
    geometry_msgs::Twist cmd;
    cmd.linear.x = msg->linear.x;
    cmd.angular.z = msg->angular.z + params.wheel_angle_offset_ / 180.0 * M_PI;
    pubWheelControlCmd(cmd);
}

void WheelController::getEncoderAngleCallback(const std_msgs::Float32::ConstPtr& msg){
    params.encoder_angle_ = msg->data;
    geometry_msgs::Twist cmd;
    cmd.linear.x = params.encoder_velocity_;
    cmd.angular.z = params.encoder_angle_ - params.wheel_angle_offset_ / 180.0 * M_PI;
    pubEncoderCmd(cmd);
}

void WheelController::getEncoderVelocityCallback(const std_msgs::Float32::ConstPtr& msg){
    params.encoder_velocity_ = msg->data;
    geometry_msgs::Twist cmd;
    cmd.linear.x = params.encoder_velocity_;
    cmd.angular.z = params.encoder_angle_ - params.wheel_angle_offset_ / 180.0 * M_PI;
    pubEncoderCmd(cmd);
}

bool WheelController::initDynamicReconfigure(){
    cfg_Cb_ = boost::bind(&WheelController::dynamicReconfigureCallback, this, _1, _2);
    cfg_server_.setCallback(cfg_Cb_);
    return true;
}

void WheelController::dynamicReconfigureCallback(ParamConfig &config, uint32_t level){
    yaml_node_["wheel_angle_offset_"] = params.wheel_angle_offset_ = config.wheel_angle_offset_;
    ROS_INFO_STREAM("wheel_angle_offset_ " << params.wheel_angle_offset_);

    // 將參數存回yaml檔
    ROS_INFO_STREAM("Save Params to YAML");
    std::ofstream fout(yaml_path_);
    fout << yaml_node_;
}


};  //end namespace wheel_controller

int main(int argc, char **argv){
    ros::init(argc, argv, "wheel_controller_node");
    ros::NodeHandle nh;
    wheel_controller::WheelController wheelcontroller(&nh);
    ros::spin();
    return 0;
}
