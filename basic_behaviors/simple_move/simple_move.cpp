#include <ros/ros.h>
#include <angles/angles.h>
#include <geometry_msgs/Twist.h>

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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "simple_move");
    ros::NodeHandle n;
    ros::Duration(1.0).sleep(); // 起動直後は rospy.Time.now() がゼロを返す．
    go_straight_by_time(n, 2.0);
}
