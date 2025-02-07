#define CANBUS_DRIVER_VERSION 1.0

#include <canbus_driver/canbus_driver.h>

namespace canbus_driver {

CanBusDriver::CanBusDriver(ros::NodeHandle *nh):
    pcan_handle_(PCAN_USBBUS1), //  假設使用 PCAN-USB 設備 1
    can_baud_rate_(PCAN_BAUD_250K)
{
    // 初始化參數
    params.add_msg_rate_                           =    50;  // hz

    params.receive_wheel_angle_lower_limit_         =  -120;  // deg
    params.receive_wheel_angle_upper_limit_         =   120;  // deg
    params.receive_wheel_speed_lower_limit_         = -4000;  // rpm
    params.receive_wheel_speed_upper_limit_         =  4000;  // rpm
    params.receive_moter_current_upper_limit_       =  1000;  // A
    params.receive_motor_temperature_lower_limit_   =  -100;  // °C
    params.receive_motor_temperature_upper_limit_   =   300;  // °C

    params.receive_fork_hight_upper_limit_          =     0;  // meter
    params.receive_fork_hight_lower_limit_          =     4;  // meter

    params.wheel_radius_                            =  0.15;  // meter
    params.wheel_reduction_ratio_                   = 21.96;  // Proportion

    // 顯示版本
    ROS_INFO_STREAM("CANBUS_DRIVER_VERSION " << CANBUS_DRIVER_VERSION);

    // 初始化各項功能
    while(!initCanBus(pcan_handle_, can_baud_rate_)){
        ROS_ERROR_STREAM("PCAN-USB initialization failed!");
        ros::Duration(1.0).sleep();
    }
    ROS_INFO_STREAM("PCAN-USB initialized successfully!");

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

    // 啟動定時發訊息執行序
    std::thread a(&CanBusDriver::addMsgToQueueThread, this);
    a.detach();  // 開始非阻塞執行緒

    // 啟動通信執行緒
    std::thread b(&CanBusDriver::transmitMsgThread, this);
    b.detach();  // 開始非阻塞執行緒
    std::thread c(&CanBusDriver::receiveMsgThread, this);
    c.detach();  // 開始非阻塞執行緒

    ROS_INFO_STREAM("Start canbus driver node...");
}

CanBusDriver::~CanBusDriver(){
    // 在析構時關閉 PCAN-USB
    CAN_Uninitialize(pcan_handle_);
    ROS_INFO_STREAM("PCAN-USB uninitialized.");
}

bool CanBusDriver::initCanBus(TPCANHandle handler, uint16_t baud_rate) {
    // init 0x203 buffer data
    msg_data_203_[0]                                =  0x01;
    msg_data_203_[1]                                =  0x00;
    msg_data_203_[2]                                =  0x00;
    msg_data_203_[3]                                =  0x02;
    msg_data_203_[4]                                =  0x02;
    msg_data_203_[5]                                =  0x00;
    msg_data_203_[6]                                =  0x00;
    msg_data_203_[7]                                =  0x00;

    // init 0x303 buffer data
    msg_data_303_[0]                                =  0x00;
    msg_data_303_[1]                                =  0x00;
    msg_data_303_[2]                                =  0x00;
    msg_data_303_[3]                                =  0x00;
    msg_data_303_[4]                                =  0x00;
    msg_data_303_[5]                                =  0x00;
    msg_data_303_[6]                                =  0x00;
    msg_data_303_[7]                                =  0x00;

    // 初始化 PCAN-USB 設備
    TPCANStatus status = CAN_Initialize(handler, baud_rate, 0, 0, 0);
    if (status != PCAN_ERROR_OK) {
        ROS_ERROR_STREAM("Error initializing PCAN device: " << status);
        return false;
    }else{
        return true;
    }
}

bool CanBusDriver::initPublisher(ros::NodeHandle *nh) {
    wheel_velocity_pub_             = nh->advertise<std_msgs::Float32>  ("canbus/get_wheel_velocity",           1000);
    wheel_angle_pub_                = nh->advertise<std_msgs::Float32>  ("canbus/get_wheel_angle",              1000);
    error_message_pub_              = nh->advertise<std_msgs::String>   ("canbus/get_error_message",            1000);
    motor_current_pub_              = nh->advertise<std_msgs::Float32>  ("canbus/get_motor_current",            1000);
    motor_driver_temperature_pub_   = nh->advertise<std_msgs::Float32>  ("canbus/get_motor_driver_temperature", 1000);
    fork_hight_in_cm_pub_           = nh->advertise<std_msgs::Float32>  ("canbus/get_fork_hight_in_cm",         1000);
    return true;
}

void CanBusDriver::pubWheelVelocity(float velocity){
    std_msgs::Float32 msg;
    msg.data = velocity;
    wheel_velocity_pub_.publish(msg);
}

void CanBusDriver::pubWheelAngle(float angle){
    std_msgs::Float32 msg;
    msg.data = angle;
    wheel_angle_pub_.publish(msg);
}

void CanBusDriver::pubErrorMessages(std::string errorMsg){
    std_msgs::String msg;
    msg.data = errorMsg;
    error_message_pub_.publish(msg);
}

void CanBusDriver::pubMotorCurrent(float motorCurrent){
    std_msgs::Float32 msg;
    msg.data = motorCurrent;
    motor_current_pub_.publish(msg);
}

void CanBusDriver::pubMotorDriverTemperature(float motorDriverTemperature){
    std_msgs::Float32 msg;
    msg.data = motorDriverTemperature;
    motor_driver_temperature_pub_.publish(msg);
}

void CanBusDriver::pubForkHight(float hight){
    std_msgs::Float32 msg;
    msg.data = hight;
    fork_hight_in_cm_pub_.publish(msg);
}

bool CanBusDriver::initSubscriber(ros::NodeHandle *nh){
    wheel_cmd_vel_sub_      = nh->subscribe("canbus/set_wheel_cmd_vel",             10, &CanBusDriver::cmdVelCallback,                  this);
    fork_control_speed_sub_ = nh->subscribe("canbus/set_fork_control_speed",        10, &CanBusDriver::forkControlSpeedCallback,        this);
    wheel_acceleration_sub_ = nh->subscribe("canbus/set_wheel_acceleration_ratio",  10, &CanBusDriver::wheelAccelerationRatioCallback,  this);
    wheel_deceleration_sub_ = nh->subscribe("canbus/set_wheel_deceleration_ratio",  10, &CanBusDriver::wheelDecelerationRatioCallback,  this);
    return true;
}

void CanBusDriver::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg){
    ROS_INFO_STREAM("Cmd Val Callback");

    float velocity          = msg->linear.x;   // m/s
    float angle_in_radian   = msg->angular.z;  // radian
    float speed_in_rpm      = velocity * 60.0 * params.wheel_reduction_ratio_ / (2.0 * M_PI * params.wheel_radius_);  // rmp
    float angle_in_deg      = angle_in_radian / M_PI * 180.0;  // deg
    bool  forward_enable;
    bool  backward_enable;

    if(speed_in_rpm <= params.receive_wheel_speed_lower_limit_) speed_in_rpm = params.receive_wheel_speed_lower_limit_;
    if(params.receive_wheel_speed_upper_limit_ <= speed_in_rpm) speed_in_rpm = params.receive_wheel_speed_upper_limit_;

    if(angle_in_deg <= params.receive_wheel_angle_lower_limit_) angle_in_deg = params.receive_wheel_angle_lower_limit_;
    if(params.receive_wheel_angle_upper_limit_ <= angle_in_deg) angle_in_deg = params.receive_wheel_angle_upper_limit_;

    if(0 < speed_in_rpm){
        forward_enable  = true;
        backward_enable = false;
    }else if(speed_in_rpm < 0){
        forward_enable  = false;
        backward_enable = true;
    }else{
        forward_enable  = false;
        backward_enable = false;
    }

    // speed control data range 0~4000 rpm
    u_int32_t speed = round(abs(speed_in_rpm));

    // angle control data range -12000~12000 (0.01deg)
    short angle = -int(round(angle_in_deg * 100.0));  // (curtis 左轉為負 右轉為正)

    update_curtis_mutex_.lock();

    // 203 byte0 bit1
    if(forward_enable){
        msg_data_203_[0] |=  (0x01 << 1);  // 0000 0010
    }else{
        msg_data_203_[0] &= ~(0x01 << 1);  // 1111 1101
    }

    // 203 byte0 bit2
    if(backward_enable){
        msg_data_203_[0] |=  (0x01 << 2);  // 0000 0100
    }else{
        msg_data_203_[0] &= ~(0x01 << 2);  // 1111 1011
    }

    // 203 byte1
    // 203 byte2
    msg_data_203_[1] = speed & 0xff;
    msg_data_203_[2] = (speed >> 8) & 0xff;

    // 203 byte5
    // 203 byte6
    msg_data_203_[5] = angle & 0xff;
    msg_data_203_[6] = (angle >> 8) & 0xff;

    update_curtis_mutex_.unlock();
}

void CanBusDriver::forkControlSpeedCallback(const std_msgs::Float32::ConstPtr& msg){
    ROS_INFO_STREAM("Fork Control Speed Callback");
    float speed = msg->data;

    if(100 <= speed)  speed =  100.0;
    if(speed <= -100) speed = -100.0;

    bool  fork_rise_enable;
    float fork_rise_speed;

    bool  fork_fall_enable;
    float fork_fall_speed;

    if(0 < speed){
        fork_rise_enable = true;
        fork_rise_speed = abs(speed);
        fork_fall_enable = false;
        fork_fall_speed = 0;
    }else if(speed < 0){
        fork_rise_enable = false;
        fork_rise_speed = 0;
        fork_fall_enable = true;
        fork_fall_speed = abs(speed);
    }else{
        fork_rise_enable = false;
        fork_rise_speed = 0;
        fork_fall_enable = false;
        fork_fall_speed = 0;
    }

    // fork fall data range 0~200
    u_int8_t fall_speed = int(round(fork_fall_speed * 2));

    // fork rise data range 0~100
    u_int8_t rise_speed = int(round(fork_rise_speed));

    update_curtis_mutex_.lock();

    // 203 byte0 bit4
    if(fork_fall_enable){
        msg_data_203_[0] |=  (0x01 << 4);  // 0001 0000
    }else{
        msg_data_203_[0] &= ~(0x01 << 4);  // 1110 1111
    }

    // 203 byte0 bit5
    if(fork_rise_enable){
        msg_data_203_[0] |=  (0x01 << 5);  // 0010 0000
    }else{
        msg_data_203_[0] &= ~(0x01 << 5);  // 1101 1111
    }

    // 203 byte7
    msg_data_203_[7] = fall_speed;

    // 303 byte2
    msg_data_303_[2] = rise_speed;

    update_curtis_mutex_.unlock();
}

void CanBusDriver::wheelAccelerationRatioCallback(const std_msgs::Float32::ConstPtr& msg){
    ROS_INFO_STREAM("Wheel Acceleration Callback");

    float acceleration_percentage = msg->data;

    if(acceleration_percentage <= 0)   acceleration_percentage =   0.0;
    if(100 <= acceleration_percentage) acceleration_percentage = 100.0;

    // data range min   2(fast => 100%)
    // data range max 255(solw =>   0%)

    u_int8_t acceleration = int(round(2.0 + (255.0 - 2.0) / 100.0 * (100.0 - acceleration_percentage)));

    update_curtis_mutex_.lock();

    // 203 byte3
    msg_data_203_[3] = acceleration;

    update_curtis_mutex_.unlock();
}

void CanBusDriver::wheelDecelerationRatioCallback(const std_msgs::Float32::ConstPtr& msg){
    ROS_INFO_STREAM("Wheel Deceleration Callback");

    float deceleration_percentage = msg->data;

    if(deceleration_percentage <= 0)   deceleration_percentage =   0.0;
    if(100 <= deceleration_percentage) deceleration_percentage = 100.0;

    // data range min   2(fast => 100%)
    // data range max 255(solw =>   0%)

    u_int8_t deceleration = int(round(2.0 + (255.0 - 2.0) / 100.0 * (100.0 - deceleration_percentage)));

    update_curtis_mutex_.lock();

    // 203 byte4
    msg_data_203_[4] = deceleration;

    update_curtis_mutex_.unlock();
}

void CanBusDriver::addMsgToQueueThread(){
    float sleep_time = 1.0 / params.add_msg_rate_;

    while (ros::ok()) {
        update_curtis_mutex_.lock();

        CanBusCmd msg203;
        msg203.can_id = DeviceID::Curtis0x203;
        msg203.can_dlc = 8;
        msg203.msg_type = MSGTYPE_STANDARD;
        memcpy(msg203.can_data, &msg_data_203_, msg203.can_dlc);

        CanBusCmd msg303;
        msg303.can_id = DeviceID::Curtis0x303;
        msg303.can_dlc = 8;
        msg303.msg_type = MSGTYPE_STANDARD;
        memcpy(msg303.can_data, &msg_data_303_, msg303.can_dlc);

        update_curtis_mutex_.unlock();

        msg_queue_mutex_.lock();

        msg_queue_.push(msg203);
        msg_queue_.push(msg303);

        msg_queue_mutex_.unlock();

        ros::Duration(sleep_time).sleep();
    }
}

void CanBusDriver::transmitMsgThread(){
    while (ros::ok()) {
        //? early return
        if(msg_queue_.size()==0) continue;

        msg_queue_mutex_.lock();
        CanBusCmd send_msg = msg_queue_.front();
        msg_queue_.pop();
        msg_queue_mutex_.unlock();

        // 發送訊息
        TPCANMsg msg;  // 用來存放發送的訊息
        msg.ID = send_msg.can_id;
        msg.LEN = send_msg.can_dlc;
        msg.MSGTYPE = send_msg.msg_type;
        memcpy(msg.DATA, &send_msg.can_data, send_msg.can_dlc);

        // 發送訊息
        TPCANStatus status = CAN_Write(pcan_handle_, &msg);
        if (status != PCAN_ERROR_OK) {
            ROS_ERROR_STREAM("Error sending CAN message: " << status);
            pubErrorMessages("Error sending CAN message");
        } else {
            // ROS_INFO_STREAM("Message sent successfully!");
        }
    }
}

void CanBusDriver::receiveMsgThread(){
    TPCANMsg msg;  // 用來存放發送的訊息
    TPCANTimestamp timestamp;  // 用來存放接收訊息的時間戳
    while(ros::ok()){
        // 接收訊息
        TPCANMsg receivedMsg;
        TPCANStatus status = CAN_Read(pcan_handle_, &receivedMsg, &timestamp);
        if(status != PCAN_ERROR_OK)continue;

        // ROS_INFO_STREAM("Receive canbus message from " << "0x" << std::hex << (int)receivedMsg.ID);

        switch(receivedMsg.ID) {
            case DeviceID::Curtis0x183:
                handleMsg183(receivedMsg.LEN, receivedMsg.DATA);
                break;
            case DeviceID::Curtis0x283:
                handleMsg283(receivedMsg.LEN, receivedMsg.DATA);
                break;
            case DeviceID::Curtis0x383:
                handleMsg383(receivedMsg.LEN, receivedMsg.DATA);
                break;
            case DeviceID::Stm0x778:
                handleMsg778(receivedMsg.LEN, receivedMsg.DATA);
                break;
            default:
                // ROS_WARN_STREAM("receive unknow message ID : " << "0x" << std::hex << (int)receivedMsg.ID);

                // for(int i=0; i<8; i++){
                //     std::bitset<8> binary((int)receivedMsg.DATA[i]);
                //     ROS_INFO_STREAM("byte "<< i << " : 0x" << std::setw(2) << std::setfill('0') << std::hex << (int)receivedMsg.DATA[i] << " " << binary);
                // }
                break;
        }

        // ROS_INFO_STREAM("Timestamp: " << timestamp.millis << " ms");
    }
}

void CanBusDriver::handleMsg183(unsigned char length, unsigned char data[8]){
    // byte0
    // byte1
    short motor_speed = (data[1] << 8) | data[0];  // rpm
    float wheel_velocity = 2.0 * M_PI * params.wheel_radius_ * motor_speed / (60.0 * params.wheel_reduction_ratio_);  // 2*pi*r*rpm/60 = m/s
    if(motor_speed < params.receive_wheel_speed_lower_limit_ || params.receive_wheel_speed_upper_limit_ < motor_speed){
        std::stringstream ss;
        ss << "[Steering speed] : Error " << motor_speed << " rpm";
        ROS_ERROR_STREAM(ss.str());
        pubErrorMessages(ss.str());
    }else{
        // ROS_INFO_STREAM("[Steering speed] : " << motor_speed << " rpm");
        // ROS_INFO_STREAM("[Steering velocity] : " << wheel_velocity << " m/s");
        pubWheelVelocity(wheel_velocity);
    }

    // byte3
    short raw_current = (data[3] << 8) | data[2];
    float current = raw_current / 10.0;
    if(0 <= current  || current <= params.receive_moter_current_upper_limit_){
        // ROS_INFO_STREAM("[Motor Current] : " << current << " A");
        pubMotorCurrent(current);
    }else{
        std::stringstream ss;
        ss << "[Motor Current] : Error " << current << " A";
        ROS_ERROR_STREAM(ss.str());
        pubErrorMessages(ss.str());
    }

    // byte4
    if(0x12 <= (int)data[4] && (int)data[4] <= 0xc6){
        std::stringstream ss;
        ss << "[Steering] : Error " << "0x" << std::setw(2) << std::setfill('0') << std::hex << (int)data[4];
        ROS_ERROR_STREAM(ss.str());
        pubErrorMessages(ss.str());
    }

    // byte6
    // byte7
    short raw_temperature = (data[7] << 8) | data[6];
    float temperature = raw_temperature / 10.0;
    if(params.receive_motor_temperature_lower_limit_ <= temperature && temperature <= params.receive_motor_temperature_upper_limit_){
        // ROS_INFO_STREAM("[Motor Driver Temperature] : " << temperature << " 'C");
        pubMotorDriverTemperature(temperature);
    }else{
        std::stringstream ss;
        ss << "[Motor Driver Temperature] : Error " << temperature << " 'C";
        ROS_ERROR_STREAM(ss.str());
        pubErrorMessages(ss.str());
    }

}

void CanBusDriver::handleMsg283(unsigned char length, unsigned char data[8]){
    // byte3
    int travel_distance = (data[3] << 24) | (data[2] << 16) | (data[1] << 8) | data[0];
    // ROS_INFO_STREAM("[Travel Odometer Encoder] : " << travel_distance << " cm");

    // byte6
    unsigned char bit0 = (data[6] >> 0) & 0x1;
    unsigned char bit2 = (data[6] >> 2) & 0x1;
    unsigned char bit3 = (data[6] >> 3) & 0x1;

    if((int)bit0 == 0){
        // ROS_INFO_STREAM("[Emergency Stop] : Software");
    }else{
        // ROS_INFO_STREAM("[Emergency Stop] : Hardware");
    }

    if((int)bit2){
        // ROS_INFO_STREAM("[Wheel Direction] : Forward");
    }

    if((int)bit3){
        // ROS_INFO_STREAM("[Wheel Direction] : Backward");
    }
}

void CanBusDriver::handleMsg383(unsigned char length, unsigned char data[8]){
    // byte0
    // byte1
    short raw_angle = (data[1] << 8) | data[0];
    float angle = raw_angle / 100.0;
    if(angle < params.receive_wheel_angle_lower_limit_  || params.receive_wheel_angle_upper_limit_ < angle){
        std::stringstream ss;
        ss << "[Steering angle] : Error " << angle << " deg";
        ROS_ERROR_STREAM(ss.str());
        pubErrorMessages(ss.str());
    }else{
        // ROS_INFO_STREAM("[Steering angle] : " << angle << " deg (raw data)");
        float angle_in_deg = angle;
        float angle_in_radian = -angle_in_deg / 180.0 * M_PI;  // (curtis 左轉為負 右轉為正)
        // ROS_INFO_STREAM("[Steering angle] : " << angle_in_deg << " deg (with offset)");
        // ROS_INFO_STREAM("[Steering angle] : " << angle_in_radian << " radian (with offset)");
        pubWheelAngle(angle_in_radian);
    }

    // byte3
    if(11 < (int)data[3] && (int)data[3] < 76){
        std::stringstream ss;
        ss << "[Steering angle] Error Code : " << "0x" << std::setw(2) << std::setfill('0') << std::hex << (int)data[3];
        ROS_ERROR_STREAM(ss.str());
        pubErrorMessages(ss.str());
    }
}

void CanBusDriver::handleMsg778(unsigned char length, unsigned char data[8]){
    // byte0
    // byte1
    // byte2
    // byte3
    int raw_hight = (data[3] << 24) | (data[2] << 16) | (data[1] << 8) | data[0];
    float hight_in_cm = raw_hight / 1000.0;  // from 0.1mm to cm
    pubForkHight(hight_in_cm);

    // ROS_INFO_STREAM("[fork hight] : " << hight_in_cm << " cm");

}

};  //end namespace canbus_driver

int main(int argc, char **argv){
    ros::init(argc, argv, "canbus_driver_node");
    ros::NodeHandle nh;
    canbus_driver::CanBusDriver canbusdriver(&nh);
    ros::spin();
    return 0;
}
