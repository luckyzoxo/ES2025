#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

class ObstacleAvoidance
{
public:
    ObstacleAvoidance()
    {
        // 初始化ROS节点句柄
        nh = ros::NodeHandle();

        // 订阅激光雷达的/scan话题
        laser_sub = nh.subscribe("/scan", 1000, &ObstacleAvoidance::scanCallback, this);

        // 创建一个Publisher，用于发布Twist消息到/cmd_vel话题上
        vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);

        // 初始化速度指令消息
        cmd_vel.linear.x = 1.0; // 初始状态下匀速前进
        cmd_vel.angular.z = 0.0;
    }

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    bool obstacle_detected = false;
    int count = scan->scan_time / scan->time_increment;
    float min_range = scan->range_max;
  

    // 遍历激光雷达扫描数据
    for (int i = 0; i < count; i++) {
        float range = scan->ranges[i];
        if (range < min_range) {
            min_range = range;
        }
    }

    // 如果检测到的最小距离小于阈值，则认为前方有障碍物
    if (min_range < 1.0) { // 阈值可以根据需要调整
        obstacle_detected = true;
    }

    // 根据障碍物检测结果发布速度指令
    if (obstacle_detected) {
        // 刹车停车条件：前车速度与加速度后刹车后间距不小于30cm
        float current_velocity = cmd_vel.linear.x; // 当前速度
        float current_acceleration = (cmd_vel.linear.x - prev_velocity) / scan->time_increment; // 当前加速度
        float stopping_distance = (current_velocity * current_velocity) / (2 * current_acceleration); // 刹车后的停车距离
        if (stopping_distance >= 0.3) { // 停车距离不小于30cm
            cmd_vel.linear.x = 0.0; // 停止移动
        }
    } else {
        cmd_vel.linear.x = 1.0; // 继续向前移动
    }

    vel_pub.publish(cmd_vel);

    // 保存当前速度，用于计算加速度
    prev_velocity = cmd_vel.linear.x;
}

private:
    ros::NodeHandle nh;
    ros::Subscriber laser_sub;
    ros::Publisher vel_pub;
    float prev_velocity;
    geometry_msgs::Twist cmd_vel;
};

int main(int argc, char** argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "obstacle_avoidance_node");

    // 创建障碍物检测对象
    ObstacleAvoidance obstacle_avoidance;

    // 进入ROS循环
    ros::spin();

    return 0;
}

