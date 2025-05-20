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
    if(is_stopped_) return;

    // �����ٶ�ָ����Ϣ
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 1.0; // ����ǰ��
    cmd_vel.angular.z = 0.0; // ��ת��

    // �����ٶ�ָ��
    vel_pub_.publish(cmd_vel);

  
}

void ConstantForwardMotion::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    if (scan->ranges.empty()) {
        ROS_ERROR("Received empty laser scan data");
        return;
    }

    if(is_stopped_) return;

    obstacle_detected = false;
   // float min_range = scan->range_max;
    // ���������״�ɨ������
   // auto ranges = scan->ranges;
    //for (int i = 0; i < scan->ranges.size();i++) {
        // if (range < min_range) {
        //     min_range = range;
        // }


    //}
    
    data.clear();
    obj.clear();
    for(const auto& distance : scan->ranges){
        if(std::isinf(distance)){
    	    continue;
   	    }
        data.push_back(distance);
    }
    
    ROS_INFO("Laser scan data:");
    for (size_t i = 0; i < data.size() && i < 5; ++i) { // ��ӡǰ30�����ݵ�
        ROS_INFO("Range[%zu]: %f", i, data[i]);
    }
    
    
 
    
    
    
    int overcount = 0;
    for(const auto& distance : data){
        //distance
        if(distance < 0.5){
            overcount++;
        }
        else{
            if(overcount > 0){
                obj.push_back(overcount);
                overcount = 0;
            }
        }
    }
    if(overcount > 0){
        obj.push_back(overcount);
        overcount = 0;
    }

    ROS_INFO("obstacle size: %d",obj.size());
    for(int i=0; i < obj.size(); i++){
        if(obj[i] >= 50)
            obstacle_detected = true;
        ROS_INFO("range for %zu obstacle: %d",i,obj[i]);
    }

    
    
    //if(std::isinf(min_range)){
    //	return;
    //}
    // �����⵽����С����С�ڵ���0.3�ף�����Ϊǰ�����ϰ���
   //if(min_range <=0.301){
   //	obstacle_detected = true;
   //}

    if (obstacle_detected) {
        ROS_WARN("Obstacle detected!");

        // ֹͣС��
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = 0.0; // ֹͣ
        cmd_vel.angular.z = 0.0;
        vel_pub_.publish(cmd_vel);

        is_stopped_=true;
        stop_timer_=nh_.createTimer(ros::Duration(2.0),&ConstantForwardMotion::resumeMotion,this,true);
    }
    else {
       // ROS_INFO("No obstacle. Min range: %f", min_range);
        ROS_INFO("No obstacle");

    }
}

void ConstantForwardMotion::resumeMotion(const ros::TimerEvent& event)
{
    is_stopped_=false;
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
