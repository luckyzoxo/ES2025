#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <mutex>

class MotionControl
{
public:
    MotionControl();

private:
    void aebCallback(const geometry_msgs::Twist::ConstPtr& msg);
    void keyboardCallback(const geometry_msgs::Twist::ConstPtr& msg);
    void publishCmd();

    ros::NodeHandle nh_;
    ros::Publisher cmd_vel_pub_;
    ros::Subscriber aeb_sub_;
    ros::Subscriber keyboard_sub_;
    geometry_msgs::Twist current_cmd_;
    std::mutex mutex_;
    bool aeb_active_ = false;
};

MotionControl::MotionControl()
{
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    aeb_sub_ = nh_.subscribe("/aeb_control", 1, &MotionControl::aebCallback, this);
    keyboard_sub_ = nh_.subscribe("/keyboard_control", 1, &MotionControl::keyboardCallback, this);
}

void MotionControl::aebCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (msg->linear.x == 0.0 && msg->angular.z == 0.0) {
        aeb_active_ = true;
        current_cmd_ = *msg;
    } else {
        aeb_active_ = false;
    }
    publishCmd();
}

void MotionControl::keyboardCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!aeb_active_) {
        current_cmd_ = *msg;
    }
    publishCmd();
}

void MotionControl::publishCmd()
{
    cmd_vel_pub_.publish(current_cmd_);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "motion_control_node");
    MotionControl motion_control;
    ros::spin();
    return 0;
}

