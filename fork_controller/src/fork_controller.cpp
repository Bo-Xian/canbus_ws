#define FORK_CONTROLLER_VERSION 1.0

#include <fork_controller/fork_controller.h>

namespace fork_controller {

ForkController::ForkController(ros::NodeHandle *nh){
    // 初始化參數
    params.ros_rate_                =    50;  // hz

    params.yaml_path_ = ros::package::getPath("fork_controller") + "/cfg/fork_controller.yaml";
    params.yaml_node_ = YAML::LoadFile(params.yaml_path_);

    params.fork_hight_upper_limit_  =  3.95;  // meter
    params.fork_hight_lower_limit_  =  0.06;  // meter

    params.upward_min_speed_        =  20.0;  // percentage
    params.downward_min_speed_      =  20.0;  // percentage

    params.current_hight_           =  0.20;  // meter
    params.target_hight_            =  0.20;  // meter
    params.total_travel_            =  0.20;  // meter

    params.fork_hight_tolerance_    =  0.01;  // meter
    params.fork_hight_offset_a_     =   1.0;
    params.fork_hight_offset_b_     =   0.0;

    params.upward_prev_error_       =   0.0;
    params.upward_integral_         =   0.0;
    params.upward_kp_               = 500.0;
    params.upward_ki_               =   0.0;
    params.upward_kd_               =   0.0;

    params.downward_prev_error_     =   0.0;
    params.downward_integral_       =   0.0;
    params.downward_kp_             = 200.0;
    params.downward_ki_             =   0.0;
    params.downward_kd_             =   0.0;

    // 顯示版本
    ROS_INFO_STREAM("FORK_CONTROLLER_VERSION " << FORK_CONTROLLER_VERSION);

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

    // 啟動枒杈控制執行緒
    std::thread b(&ForkController::forkControlThread, this);
    b.detach();

    ROS_INFO_STREAM("Start fork controller node...");
}

ForkController::~ForkController(){}


bool ForkController::initPublisher(ros::NodeHandle *nh) {
    fork_control_pub_ = nh->advertise<std_msgs::Float32>  (       "canbus/set_fork_control_speed", 1000);
    fork_hight_pub_   = nh->advertise<std_msgs::Float32>  ("fork_controller/get_fork_hight_in_cm", 1000);
    return true;
}

void ForkController::pubForkControlSpeed(float speed){
    if(speed <= -100.0) speed = -100.0;
    if(100.0 <=  speed) speed =  100.0;

    if(0 < speed &&    speed < params.upward_min_speed_) speed =    params.upward_min_speed_;
    if(speed < 0 && -params.downward_min_speed_ < speed) speed = -params.downward_min_speed_;

    std_msgs::Float32 msg;
    msg.data = speed;
    fork_control_pub_.publish(msg);
}

void ForkController::pubForkHight(float hight){
    std_msgs::Float32 msg;
    msg.data = (hight - params.fork_hight_offset_b_) / params.fork_hight_offset_a_;
    fork_hight_pub_.publish(msg);
}

bool ForkController::initSubscriber(ros::NodeHandle *nh){
    fork_hight_sub_     = nh->subscribe("canbus/get_fork_hight_in_cm", 10, &ForkController::getForkHightCallback, this);
    target_hight_sub_   = nh->subscribe("fork_controller/set_fork_hight_in_cm", 10, &ForkController::setForkHightCallback, this);
    return true;
}

void ForkController::getForkHightCallback(const std_msgs::Float32::ConstPtr& msg){
    params.current_hight_ = msg->data / 100.0;
}

void ForkController::setForkHightCallback(const std_msgs::Float32::ConstPtr& msg){
    float hight = msg->data/100.0 * params.fork_hight_offset_a_ + params.fork_hight_offset_b_;

    if(hight <= params.fork_hight_lower_limit_) hight = params.fork_hight_lower_limit_;
    if(params.fork_hight_upper_limit_ <= hight) hight = params.fork_hight_upper_limit_;

    if(params.target_hight_ != hight){
        params.target_hight_ = hight;
        params.upward_integral_ = 0;
        params.downward_integral_ = 0;
    }

    params.total_travel_ = abs(params.target_hight_ - params.current_hight_);
}

bool ForkController::initDynamicReconfigure(){
    cfg_Cb_ = boost::bind(&ForkController::dynamicReconfigureCallback, this, _1, _2);
    cfg_server_.setCallback(cfg_Cb_);
    return true;
}

void ForkController::dynamicReconfigureCallback(fork_controller::forkControllerParamConfig &config, uint32_t level){
    params.yaml_node_["ros_rate_"] = params.ros_rate_ = config.ros_rate_;
    ROS_INFO_STREAM("ros_rate_ "<<params.ros_rate_);

    params.yaml_node_["fork_hight_upper_limit_"] = params.fork_hight_upper_limit_ = config.fork_hight_upper_limit_;
    params.yaml_node_["fork_hight_lower_limit_"] = params.fork_hight_lower_limit_ = config.fork_hight_lower_limit_;
    ROS_INFO_STREAM("fork_hight_upper_limit_ "<<params.fork_hight_upper_limit_);
    ROS_INFO_STREAM("fork_hight_lower_limit_ "<<params.fork_hight_lower_limit_);

    params.yaml_node_["upward_min_speed_"] = params.upward_min_speed_ = config.upward_min_speed_;
    params.yaml_node_["downward_min_speed_"] = params.downward_min_speed_ = config.downward_min_speed_;
    ROS_INFO_STREAM("upward_min_speed_ "<<params.upward_min_speed_);
    ROS_INFO_STREAM("downward_min_speed_ "<<params.downward_min_speed_);

    params.yaml_node_["fork_hight_offset_a_"] = params.fork_hight_offset_a_ = config.fork_hight_offset_a_;
    ROS_INFO_STREAM("fork_hight_offset_a_ "<<params.fork_hight_offset_a_);
    params.yaml_node_["fork_hight_offset_b_"] = params.fork_hight_offset_b_ = config.fork_hight_offset_b_;
    ROS_INFO_STREAM("fork_hight_offset_b_ "<<params.fork_hight_offset_b_);

    params.yaml_node_["fork_hight_tolerance_"] = params.fork_hight_tolerance_ = config.fork_hight_tolerance_;
    ROS_INFO_STREAM("fork_hight_tolerance_ "<<config.fork_hight_tolerance_);

    params.yaml_node_["upward_kp_"] = params.upward_kp_ = config.upward_kp_;
    params.yaml_node_["upward_ki_"] = params.upward_ki_ = config.upward_ki_;
    params.yaml_node_["upward_kd_"] = params.upward_kd_ = config.upward_kd_;
    ROS_INFO_STREAM("upward_kp_ "<<config.upward_kp_);
    ROS_INFO_STREAM("upward_ki_ "<<config.upward_ki_);
    ROS_INFO_STREAM("upward_kd_ "<<config.upward_kd_);

    params.yaml_node_["downward_kp_"] = params.downward_kp_ = config.downward_kp_;
    params.yaml_node_["downward_ki_"] = params.downward_ki_ = config.downward_ki_;
    params.yaml_node_["downward_kd_"] = params.downward_kd_ = config.downward_kd_;
    ROS_INFO_STREAM("downward_kp_ "<<config.downward_kp_);
    ROS_INFO_STREAM("downward_ki_ "<<config.downward_ki_);
    ROS_INFO_STREAM("downward_kd_ "<<config.downward_kd_);

    // 將參數存回yaml檔
    ROS_INFO_STREAM("Save Params to YAML");
    std::ofstream fout(params.yaml_path_);
    fout << params.yaml_node_;

}

void ForkController::forkControlThread(){
    float sleep_time = 1.0 / params.ros_rate_;
    while (ros::ok()) {

        float hight_controll_speed;
        float error = params.target_hight_ - params.current_hight_;

        if(0 < error) hight_controll_speed =    forkHightUpwardPid(abs(error));
        if(error < 0) hight_controll_speed = -forkHightDownwardPid(abs(error));
        // {
        //     if(1 < params.total_travel_) hight_controll_speed = -forkHightDownwardPid(abs(error) / params.total_travel_);
        //     else hight_controll_speed = -forkHightDownwardPid(abs(error));
        // }

        if(abs(error) < params.fork_hight_tolerance_){
            hight_controll_speed = 0;
            params.upward_integral_ = 0;
            params.downward_integral_ = 0;
        }

        // ROS_INFO_STREAM("target_hight_ "<<params.target_hight_);
        // ROS_INFO_STREAM("current_hight_ "<<params.current_hight_);
        // ROS_INFO_STREAM("error "<<error);
        // ROS_INFO_STREAM("hight_controll_speed "<<hight_controll_speed);

        pubForkControlSpeed(hight_controll_speed);
        pubForkHight(params.current_hight_);

        ros::Duration(sleep_time).sleep();
    }
}

float ForkController::forkHightUpwardPid(float error){
    float dt = 1.0 / params.ros_rate_;
    // integral
    params.upward_integral_ += dt * error;
    // derivative
    float derivative = (error - params.upward_prev_error_) / dt;
    // output
    float output = error * params.upward_kp_ + params.upward_integral_ * params.upward_ki_ + params.upward_prev_error_ * params.upward_kd_;
    // update
    params.upward_prev_error_ = error;
    // return
    return output;
}

float ForkController::forkHightDownwardPid(float error){
    float dt = 1.0 / params.ros_rate_;
    // integral
    params.downward_integral_ += dt * error;
    // derivative
    float derivative = (error - params.downward_prev_error_) / dt;
    // output
    float output = error * params.downward_kp_ + params.downward_integral_ * params.downward_ki_ + params.downward_prev_error_ * params.downward_kd_;
    // update
    params.downward_prev_error_ = error;
    // return
    return output;
}

};  //end namespace fork_controller

int main(int argc, char **argv){
    ros::init(argc, argv, "fork_controller_node");
    ros::NodeHandle nh;
    fork_controller::ForkController forkcontroller(&nh);
    ros::spin();
    return 0;
}
