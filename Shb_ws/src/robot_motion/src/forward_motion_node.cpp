#include <ros/ros.h> // ROS的核心头文件
#include <sensor_msgs/LaserScan.h> // 激光扫描消息类型头文件
#include <geometry_msgs/Twist.h> // 速度消息类型头文件
#include <cmath> // 数学库头文件
#include <vector> // 向量容器头文件
#include <eigen3/Eigen/Dense> // Eigen库头文件，用于矩阵运算
#include <random> // 随机数生成

class ConstantForwardMotion
{
public:
    ConstantForwardMotion();

private:
    void publish(const ros::TimerEvent& event);
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
    void resumeMotion(const ros::TimerEvent& event);

    ros::NodeHandle nh_; // ROS节点句柄
    ros::Publisher vel_pub_; // 速度指令发布器
    ros::Subscriber laser_sub_; // 激光扫描订阅器
    ros::Timer timer_; // 定时器
    ros::Timer resume_timer_; // 用于恢复运动的定时器
    int publish_count_; // 发布计数器
    std::vector<double> data; // 激光扫描数据容器
    bool obstacle_detected; // 障碍物检测标志
    bool paused_; // 机器人是否处于暂停状态

    void kMeansClustering(const Eigen::MatrixXd& points, int k, Eigen::MatrixXd& centers, std::vector<int>& labels);
    void initializeCenters(const Eigen::MatrixXd& points, int k, Eigen::MatrixXd& centers);
    void assignLabels(const Eigen::MatrixXd& points, const Eigen::MatrixXd& centers, std::vector<int>& labels);
    void updateCenters(const Eigen::MatrixXd& points, int k, const std::vector<int>& labels, Eigen::MatrixXd& centers);
};

ConstantForwardMotion::ConstantForwardMotion() : publish_count_(0), obstacle_detected(false), paused_(false) // 构造函数初始化
{
    // 创建一个Publisher，用于发布Twist消息到/cmd_vel主题
    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1, true);

    // 创建一个Subscriber，用于订阅/scan主题的激光扫描数据
    laser_sub_ = nh_.subscribe("/scan", 1000, &ConstantForwardMotion::scanCallback, this);

    // 设置定时器频率为10 Hz
    timer_ = nh_.createTimer(ros::Duration(0.1), &ConstantForwardMotion::publish, this);
}

void ConstantForwardMotion::publish(const ros::TimerEvent& event)
{
    if (!paused_) {
        // 创建并设置速度指令消息
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = 1.0; // 向前移动
        cmd_vel.angular.z = 0.0; // 不转向

        // 发布速度指令
        vel_pub_.publish(cmd_vel);
    }
}

void ConstantForwardMotion::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    if (scan->ranges.empty()) { // 检查是否接收到空的激光扫描数据
        ROS_ERROR("Received empty laser scan data");
        return;
    }

    obstacle_detected = false; // 重置障碍物检测标志

    data.clear(); // 清空数据容器
    for (const auto& distance : scan->ranges) {
        if (std::isinf(distance)) {
            continue; // 跳过无穷大的距离值
        }
        data.push_back(distance); // 将有效距离值加入数据容器
    }

    // 使用Eigen库进行数据处理
    Eigen::MatrixXd points(data.size(), 1);
    for (size_t i = 0; i < data.size(); ++i) {
        points(i, 0) = data[i];
    }

    // 设置K-means聚类的参数
    int k = 2; // 假设有两个类，一个是障碍物，一个是背景
    Eigen::MatrixXd centers;
    std::vector<int> labels;

    // 进行聚类
    kMeansClustering(points, k, centers, labels);

    // 获取聚类结果
    std::vector<int> cluster_sizes(k, 0);
    for (int label : labels) {
        cluster_sizes[label]++;
    }

    // 判断是否检测到障碍物
    for (int i = 0; i < k; ++i) {
        if (cluster_sizes[i] > 50 && centers(i, 0) < 0.5) { // 假设障碍物类的中心距离小于0.5米且包含足够多的点
            obstacle_detected = true;
            break;
        }
    }

    if (obstacle_detected) {
        ROS_WARN("Obstacle detected!");

        // 停止机器人
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = 0.0; // 停止移动
        cmd_vel.angular.z = 0.0;
        vel_pub_.publish(cmd_vel);

        // 暂停机器人
        paused_ = true;

        // 启动恢复运动的定时器，2秒后恢复
        //resume_timer_ = nh_.createTimer(ros::Duration(2.0), &ConstantForwardMotion::resumeMotion, this, true);
    }
    else {
        ROS_INFO("No obstacle");
        paused_ = false;
    }
}

void ConstantForwardMotion::resumeMotion(const ros::TimerEvent& event)
{
    // 恢复机器人运动
    paused_ = false;
}

void ConstantForwardMotion::kMeansClustering(const Eigen::MatrixXd& points, int k, Eigen::MatrixXd& centers, std::vector<int>& labels)
{
    // 初始化聚类中心
    initializeCenters(points, k, centers);
    labels.resize(points.rows());

    // 迭代进行K-means聚类
    for (int iter = 0; iter < 100; ++iter) {
        assignLabels(points, centers, labels);
        updateCenters(points, k, labels, centers);
    }
}

void ConstantForwardMotion::initializeCenters(const Eigen::MatrixXd& points, int k, Eigen::MatrixXd& centers)
{
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, points.rows() - 1);

    centers.resize(k, points.cols());
    for (int i = 0; i < k; ++i) {
        centers.row(i) = points.row(dis(gen));
    }
}

void ConstantForwardMotion::assignLabels(const Eigen::MatrixXd& points, const Eigen::MatrixXd& centers, std::vector<int>& labels)
{
    for (int i = 0; i < points.rows(); ++i) {
        double min_dist = std::numeric_limits<double>::max();
        int best_label = 0;
        for (int j = 0; j < centers.rows(); ++j) {
            double dist = (points.row(i) - centers.row(j)).squaredNorm();
            if (dist < min_dist) {
                min_dist = dist;
                best_label = j;
            }
        }
        labels[i] = best_label;
    }
}

void ConstantForwardMotion::updateCenters(const Eigen::MatrixXd& points, int k, const std::vector<int>& labels, Eigen::MatrixXd& centers)
{
    centers.setZero();
    Eigen::VectorXd counts = Eigen::VectorXd::Zero(k);
    for (int i = 0; i < points.rows(); ++i) {
        centers.row(labels[i]) += points.row(i);
        counts(labels[i]) += 1.0;
    }
    for (int j = 0; j < k; ++j) {
        if (counts(j) > 0) {
            centers.row(j) /= counts(j);
        }
    }
}

int main(int argc, char** argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "forward_motion_node");

    // 创建ConstantForwardMotion对象
    ConstantForwardMotion constant_forward_motion;

    // 开始ROS循环
    ros::spin();

    return 0;
}
