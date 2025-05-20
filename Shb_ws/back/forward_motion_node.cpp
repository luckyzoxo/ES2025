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
    int publish_count_; // ��������������
};

ConstantForwardMotion::ConstantForwardMotion() : publish_count_(0) // ��ʼ����������������
{
    // ����һ��Publisher�����ڷ���Twist��Ϣ��/cmd_vel������
    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1, true);

    // ����һ��Subscriber�����ڶ���/scan�����ϵļ����״�����
    laser_sub_ = nh_.subscribe("/scan", 1000, &ConstantForwardMotion::scanCallback, this);

    // ���÷���Ƶ��Ϊ10 Hz
    timer_ = nh_.createTimer(ros::Duration(0.1), &ConstantForwardMotion::publish, this);
}

void ConstantForwardMotion::publish(const ros::TimerEvent& event)
{
    // �����ٶ�ָ����Ϣ
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 1.0; // ����ǰ��
    cmd_vel.angular.z = 0.0; // ��ת��

    // �����ٶ�ָ��
    vel_pub_.publish(cmd_vel);

    // ���ӷ�������������
    publish_count_++;

    // ��ӡ��������Ϣ���ݼ������
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

    // ���������״�ɨ������
    for (const auto& range : scan->ranges) {
        if (range < min_range) {
            min_range = range;
        }
    }

    // ��ӡ�����״�����
    ROS_INFO("Laser scan data:");
    ROS_INFO("Angle Min: %f, Angle Max: %f, Angle Increment: %f",
        scan->angle_min, scan->angle_max, scan->angle_increment);
    ROS_INFO("Range Min: %f, Range Max: %f", scan->range_min, scan->range_max);
    for (size_t i = 0; i < scan->ranges.size() && i < 10; ++i) { // ��ӡǰ10�����ݵ�
        ROS_INFO("Range[%zu]: %f", i, scan->ranges[i]);
    }

    
}
int main(int argc, char** argv)
{
    // ��ʼ��ROS�ڵ�
    ros::init(argc, argv, "forward_motion_node");

    // ����ConstantForwardMotion����
    ConstantForwardMotion constant_forward_motion;

    // ����ROSѭ��
    ros::spin();

    return 0;
}
