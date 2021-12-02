#include <ros/ros.h>
#include <angles/angles.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

bool get_localized_pose(tf::TransformListener &listener, geometry_msgs::Pose2D &result,
                        double time_limit = 10.0, const std::string &target = "map", const std::string &source = "base_link")
{
    try
    {
        tf::StampedTransform transform;
        std::string error_msg;
        if (listener.waitForTransform(target, source, ros::Time(0), ros::Duration(time_limit), ros::Duration(0.01), &error_msg) == false)
        {
            ROS_ERROR("%s:%s", __FUNCTION__, error_msg.c_str());
            return false;
        }
        listener.lookupTransform(target, source, ros::Time(0), transform);
        tf::Matrix3x3 m(transform.getRotation());
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        result.x = transform.getOrigin().x();
        result.y = transform.getOrigin().y();
        result.theta = yaw;
        return true;
    }
    catch (tf::LookupException &ex)
    {
        ROS_ERROR("%s:%s", __FUNCTION__, ex.what());
        return false;
    }
    catch (tf::ConnectivityException &ex)
    {
        ROS_ERROR("%s:%s", __FUNCTION__, ex.what());
        return false;
    }
    catch (tf::ExtrapolationException &ex)
    {
        ROS_ERROR("%s:%s", __FUNCTION__, ex.what());
        return false;
    } 
}

void go_straight_by_distance_with_localization(ros::NodeHandle &n, tf::TransformListener &listener,
                                               double distance, double time_limit = 999, double linear_vel = 0.4,
                                               const std::string &cmd_vel = "/cmd_vel")
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
        geometry_msgs::Pose2D pose;
        if (get_localized_pose(listener, pose))
        {
            ROS_INFO("Recv localized pose. (x, y, theta) d = (%.2f, %.2f, %.2f)",
                     pose.x, pose.y, angles::to_degrees(pose.theta));
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
    ros::init(argc, argv, "simple_move_with_localization");
    ros::NodeHandle n;
    tf::TransformListener listener;                                    // このクラスのインスタンス生成は一度だけ。
    ros::Duration(1.0).sleep();                                        // 起動直後は rospy.Time.now() がゼロを返す．
    go_straight_by_distance_with_localization(n, listener, 2.0, 3.0);
}
