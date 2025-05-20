#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <std_msgs/Float32MultiArray.h>

void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& point_cloud_msg) {
    // Create a Float32MultiArray message to store distances
    std_msgs::Float32MultiArray distance_msg;
    
    // Iterate through the point cloud
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*point_cloud_msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*point_cloud_msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*point_cloud_msg, "z");
    
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        float x = *iter_x;
        float y = *iter_y;
        float z = *iter_z;
        float distance = sqrt(x * x + y * y + z * z); // Calculate distance from the origin
        distance_msg.data.push_back(distance);
    }
    
    // Publish the distances
    static ros::NodeHandle nh;
    static ros::Publisher pub = nh.advertise<std_msgs::Float32MultiArray>("/camera/distance", 10);
    pub.publish(distance_msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "point_cloud_to_distance");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/camera/depth/points", 10, pointCloudCallback);

    ros::spin();
    return 0;
}

