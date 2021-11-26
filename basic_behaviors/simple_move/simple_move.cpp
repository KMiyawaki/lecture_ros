#include <ros/ros.h>
#include <angles/angles.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>

double get_yaw(const nav_msgs::Odometry::ConstPtr& msg){
    tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
}

void go_straight_by_time(ros::NodeHandle& n, double time_limit, double linear_vel = 0.4, const std::string &cmd_vel = "/cmd_vel")
{
    ROS_INFO("Executing %s", __FUNCTION__);
    ros::Publisher pub = n.advertise<geometry_msgs::Twist>(cmd_vel, 10);
    geometry_msgs::Twist vel;
    vel.linear.x = linear_vel;
    vel.linear.y = 0;
    vel.linear.z = 0;
    vel.angular.x = 0;
    vel.angular.y = 0;
    vel.angular.z = 0;
    double diff = 0;
    ros::Rate rate(10);
    double start_time = ros::Time::now().toSec();
    while (diff < time_limit)
    {
        pub.publish(vel);
        rate.sleep();
        diff = ros::Time::now().toSec() - start_time;
    }
    vel.linear.x = 0.0;
    pub.publish(vel);
}

void go_straight_by_distance(ros::NodeHandle& n, double distance, double time_limit=999, double linear_vel=0.4, const std::string& topic="/odom", const std::string& cmd_vel="/cmd_vel", double msg_wait=1.0){
    ROS_INFO("Executing %s", __FUNCTION__);
    ros::Publisher pub = n.advertise<geometry_msgs::Twist>(cmd_vel, 10);
    geometry_msgs::Twist vel;
    vel.linear.x = linear_vel;
    vel.linear.y = 0;
    vel.linear.z = 0;
    vel.angular.x = 0;
    vel.angular.y = 0;
    vel.angular.z = 0;
    double diff = 0;
    ros::Rate rate(10);
    double start_time = ros::Time::now().toSec();
    while (diff < time_limit)
    {
        boost::shared_ptr<nav_msgs::Odometry const> odom = ros::topic::waitForMessage<nav_msgs::Odometry>(topic, n);
        if(odom){
            double x = odom->pose.pose.position.x;
            double y = odom->pose.pose.position.y;
            double yaw = get_yaw(odom);
            ROS_INFO("Recv odometry. (x, y, theta) = (%.2f, %.2f, %.2f)", x, y, angles::to_degrees(yaw));
            // http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html をよく見よう。
            // msg.pose.pose.position.x などで座標が分かる。
        }
        pub.publish(vel);
        rate.sleep();
        diff = ros::Time::now().toSec() - start_time;
    }
    vel.linear.x = 0.0;
    pub.publish(vel);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "simple_move");
    ros::NodeHandle n;
    ros::Duration(1.0).sleep(); // 起動直後は rospy.Time.now() がゼロを返す．
    go_straight_by_time(n, 2.0);
}
