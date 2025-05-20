#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <cmath>
#include <algorithm>
#include <vector>

#define OBSTACLE 50

class ConstantForwardMotion
{
public:
    ConstantForwardMotion();

private:
    void publish(const ros::TimerEvent& event);
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
    void resumeMotion(const ros::TimerEvent& event);
    float max(float a,float b);

    ros::NodeHandle nh_;
    ros::Publisher vel_pub_;
    ros::Subscriber laser_sub_;
    ros::Timer timer_;
    ros::Timer stop_timer_;
    int publish_count_;
    std::vector<int> obj;
    std::vector<double> data;
    bool obstacle_detected;
    bool stop_motion_;
    float velocity;
    static const float min_dis = 0.5f;
    static const float brake_factor = 2.0f;
};

ConstantForwardMotion::ConstantForwardMotion() : publish_count_(0), stop_motion_(false)
{
    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("aeb_control", 1, true);
    laser_sub_ = nh_.subscribe("/scan", 1000, &ConstantForwardMotion::scanCallback, this);
    timer_ = nh_.createTimer(ros::Duration(0.1), &ConstantForwardMotion::publish, this);
}

void ConstantForwardMotion::publish(const ros::TimerEvent& event)
{
    if (stop_motion_)
        return;

    geometry_msgs::Twist aeb_control;
    aeb_control.linear.x = 1.0;
    aeb_control.angular.z = 0.0;
    vel_pub_.publish(aeb_control);
}
float ConstantForwardMotion::max(float a,float b)
{
    return a>b?a:b;
}

void ConstantForwardMotion::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    if (stop_motion_)
        return;

    if (scan->ranges.empty()) {
        ROS_ERROR("Received empty laser scan data");
        return;
    }

    ROS_INFO("Laser data amount: %d",scan->ranges.size());

    obstacle_detected = false;

    data.clear();
    obj.clear();
    for (const auto& distance : scan->ranges) {
        if (std::isinf(distance)) {
            continue;
        }
        data.push_back(distance);
    }
    ROS_INFO("Laser scan data:");
    for (size_t i = 0; i < data.size() && i < 5; ++i) {
        ROS_INFO("Range[%zu]: %f", i, data[i]);
    }

    int overcount = 0;
    for (const auto& distance : data) {
        if (distance < max(min_dis,velocity * brake_factor)) {
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

    ROS_INFO("obstacle size: %d", obj.size());
    for (int i = 0; i < obj.size(); i++) {
        if (obj[i] >= OBSTACLE)
            obstacle_detected = true;
        ROS_INFO("range for %zu obstacle: %d", i, obj[i]);
    }

    if (obstacle_detected) {
        ROS_WARN("Obstacle detected!");

        geometry_msgs::Twist aeb_control;
        aeb_control.linear.x = 0.0;
        aeb_control.angular.z = 0.0;
        vel_pub_.publish(aeb_control);

        stop_motion_ = true;

        stop_timer_ = nh_.createTimer(ros::Duration(2.0), &ConstantForwardMotion::resumeMotion, this, true);
    }
    else {
        ROS_INFO("No obstacle detected");
    }
}

void ConstantForwardMotion::resumeMotion(const ros::TimerEvent& event)
{
    stop_motion_ = false;
    ROS_INFO("Resuming motion after obstacle detection");
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "forward_motion_node");

    ConstantForwardMotion constant_forward_motion;

    ros::spin();

    return 0;
}
