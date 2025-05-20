#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

class ConstantForwardMotion
{
public:
    ConstantForwardMotion();

private:
    void publish(const ros::TimerEvent& event);
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);

    ros::NodeHandle nh_;
    ros::Publisher vel_pub_;
    ros::Subscriber laser_sub_;
    ros::Timer timer_;
    int publish_count_; // 发布次数计数器
};

ConstantForwardMotion::ConstantForwardMotion() : publish_count_(0) // 初始化发布次数计数器
{
    // 创建一个Publisher，用于发布Twist消息到/cmd_vel话题上
    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1, true);

    // 创建一个Subscriber，用于订阅/scan话题上的激光雷达数据
    laser_sub_ = nh_.subscribe("/scan", 1000, &ConstantForwardMotion::scanCallback, this);

    // 设置发布频率为10 Hz
    timer_ = nh_.createTimer(ros::Duration(0.1), &ConstantForwardMotion::publish, this);
}

void ConstantForwardMotion::publish(const ros::TimerEvent& event)
{
    // 创建速度指令消息
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 1.0; // 匀速前进
    cmd_vel.angular.z = 0.0; // 不转向

    // 发布速度指令
    vel_pub_.publish(cmd_vel);

    // 增加发布次数计数器
    publish_count_++;

    // 打印发布的消息内容及其次数
   // ROS_INFO("Published cmd_vel: linear.x = %f, angular.z = %f, count = %d",
     //   cmd_vel.linear.x, cmd_vel.angular.z, publish_count_);
}

void ConstantForwardMotion::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    if (scan->ranges.empty()) {
        ROS_ERROR("Received empty laser scan data");
        return;
    }

    bool obstacle_detected = false;
    float min_range = scan->range_max;

    // 遍历激光雷达扫描数据
    for (const auto& range : scan->ranges) {
        if (range < min_range) {
            min_range = range;
        }
    }

    // 打印激光雷达数据
    ROS_INFO("Laser scan data:");
    ROS_INFO("Angle Min: %f, Angle Max: %f, Angle Increment: %f",
        scan->angle_min, scan->angle_max, scan->angle_increment);
    ROS_INFO("Range Min: %f, Range Max: %f", scan->range_min, scan->range_max);
    for (size_t i = 0; i < scan->ranges.size() && i < 10; ++i) { // 打印前10个数据点
        ROS_INFO("Range[%zu]: %f", i, scan->ranges[i]);
    }

    
}
int main(int argc, char** argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "forward_motion_node");

    // 创建ConstantForwardMotion对象
    ConstantForwardMotion constant_forward_motion;

    // 进入ROS循环
    ros::spin();

    return 0;
}
