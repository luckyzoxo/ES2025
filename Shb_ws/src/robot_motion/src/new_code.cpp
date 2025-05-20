#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <cmath>
#include <vector>

class ConstantForwardMotion
{
public:
    ConstantForwardMotion();

private:
    void publish(const ros::TimerEvent& event);
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
    void resumeMotion(const ros::TimerEvent& event);

    ros::NodeHandle nh_;
    ros::Publisher vel_pub_;
    ros::Subscriber laser_sub_;

    int publish_count_;
    std::vector<int> obj;
    std::vector<double> data;
    bool obstacle_detected;

    ros::Timer timer_;
    ros::Timer stop_timer_;
    bool is_stopped_;
};

ConstantForwardMotion::ConstantForwardMotion() : publish_count_(0), is_stopped_(false)
{
    // 初始化Publisher，发布Twist消息到/cmd_vel话题
    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1, true);

    // 初始化Subscriber，订阅/scan话题的激光雷达数据
    laser_sub_ = nh_.subscribe("/scan", 1000, &ConstantForwardMotion::scanCallback, this);

    // 设置定时器以10 Hz的频率调用publish函数
    timer_ = nh_.createTimer(ros::Duration(0.1), &ConstantForwardMotion::publish, this);
}

void ConstantForwardMotion::publish(const ros::TimerEvent& event)
{
    if (is_stopped_) return;

    // 创建并发布一个前进的速度指令
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 1.0; // 向前移动
    cmd_vel.angular.z = 0.0; // 不旋转
    vel_pub_.publish(cmd_vel);
}

void ConstantForwardMotion::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    if (scan->ranges.empty()) {
        ROS_ERROR("收到空的激光扫描数据");
        return;
    }

    if (is_stopped_) return;

    obstacle_detected = false;

    data.clear();
    obj.clear();

    for (const auto& distance : scan->ranges) {
        if (std::isinf(distance)) {
            continue;
        }
        data.push_back(distance);
    }

    int overcount = 0;
    for (const auto& distance : data) {
        if (distance < 0.5) {
            overcount++;
        }
        else {
            if (overcount > 0) {
                obj.push_back(overcount);
                overcount = 0;
            }
        }
    }
    if (overcount > 0) {
        obj.push_back(overcount);
        overcount = 0;
    }

    ROS_INFO("障碍物大小: %d", obj.size());
    for (size_t i = 0; i < obj.size(); i++) {
        if (obj[i] >= 50) {
            obstacle_detected = true;
        }
    }

    if (obstacle_detected) {
        ROS_WARN("检测到障碍物!");

        // 停止机器人
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = 0.0; // 停止
        cmd_vel.angular.z = 0.0;
        vel_pub_.publish(cmd_vel);

        is_stopped_ = true;
        stop_timer_ = nh_.createTimer(ros::Duration(2.0), &ConstantForwardMotion::resumeMotion, this, true);
    }
    else {
        ROS_INFO("未检测到障碍物");
    }
}

void ConstantForwardMotion::resumeMotion(const ros::TimerEvent& event)
{
    is_stopped_ = false;

    // 重新进行一次激光雷达数据的检测
    sensor_msgs::LaserScan::ConstPtr scan = ros::topic::waitForMessage<sensor_msgs::LaserScan>("/scan", nh_, ros::Duration(1.0));
    if (scan) {
        scanCallback(scan);
    }

    if (!obstacle_detected) {
        // 如果没有检测到障碍物，恢复运动
        is_stopped_ = false;
    }
    else {
        // 如果仍然检测到障碍物，再次停止并设置两秒钟后再检测
        stop_timer_ = nh_.createTimer(ros::Duration(2.0), &ConstantForwardMotion::resumeMotion, this, true);
    }
}

int main(int argc, char** argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "forward_motion_node");

    // 创建ConstantForwardMotion类的实例
    ConstantForwardMotion constant_forward_motion;

    // 进入ROS循环
    ros::spin();

    return 0;
}
