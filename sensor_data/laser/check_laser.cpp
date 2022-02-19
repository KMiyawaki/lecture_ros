#include <vector>
#include <string>
#include <ros/ros.h>
#include <angles/angles.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/tf.h>

bool check_ros_node(const std::string &target = "/stageros")
{
    std::vector<std::string> nodes;
    if (ros::master::getNodes(nodes))
    {
        std::vector<std::string>::const_iterator it = std::find(nodes.cbegin(), nodes.cend(), target);
        return it != nodes.cend();
    }
    ROS_ERROR("%s:Failed to get node names", __PRETTY_FUNCTION__);
    return false;
}

void check_laser(ros::NodeHandle &n, double time_limit, const std::string &topic = "/base_scan", double msg_wait=1.0)
{
    ROS_INFO("Executing %s", __FUNCTION__);
    // シミュレーションかどうかをチェック
    bool is_simulation = check_ros_node();
    ROS_INFO("is_simulation = %s", (is_simulation ? "true" : "false"));
    double diff = 0;
    ros::Rate rate(10);
    double start_time = ros::Time::now().toSec();
    while (diff < time_limit)
    {
        boost::shared_ptr<sensor_msgs::LaserScan const> sensor_msg = ros::topic::waitForMessage<sensor_msgs::LaserScan>(topic, n, ros::Duration(msg_wait));
        if (sensor_msg)
        {
            ROS_INFO("Recv LaserScan. ");
            // http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html をよく見よう。
            // sensor_msg->ranges.size() がスキャンデータの個数
            // sensor_msg->ranges[i] で i 番目のセンサデータが分かる。
        }
        rate.sleep();
        diff = ros::Time::now().toSec() - start_time;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "check_laser");
    ros::NodeHandle n;
    ros::Duration(1.0).sleep(); // 起動直後は rospy.Time.now() がゼロを返す．
    check_laser(n, 3.0);
}