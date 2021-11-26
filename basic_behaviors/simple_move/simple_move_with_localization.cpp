#include <ros/ros.h>
#include <angles/angles.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

bool get_localized_pose(tf::TransformListener &listener, geometry_msgs::Pose2D& result, double time_limit = 10.0, const std::string &target = "map", const std::string &source = "base_link")
{
    try
    {
        tf::StampedTransform transform;
        listener.waitForTransform(target, source, ros::Time(0), ros::Duration(time_limit));
        listener.lookupTransform(target, source, ros::Time(0), transform);
        tf::Matrix3x3 m(transform.getRotation());
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        result.x = transform.getOrigin().x();
        result.y = transform.getOrigin().y();
        result.theta = yaw;
        return true;
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("%s", ex.what());
        return false;
    }
}
/*

def go_straight_by_distance_with_localization(listener, distance, time_limit=999, linear_vel=0.4, topic='/odom', cmd_vel="/cmd_vel", msg_wait=1.0):
    func = sys._getframe().f_code.co_name
    rospy.loginfo('Executing ' + func)
    pub = rospy.Publisher(cmd_vel, Twist, queue_size=10)
    vel = Twist()
    vel.linear.x = linear_vel
    vel.linear.y = 0
    vel.linear.z = 0
    vel.angular.x = 0
    vel.angular.y = 0
    vel.angular.z = 0
    diff = 0
    start_time = rospy.get_time()

    while diff < time_limit:
        try:
            (x, y, yaw) = get_localized_pose(listener)
        except Exception as e:
            rospy.logerr(str(e))
            return

        rospy.loginfo(
            'Recv localized pose. (x, y, theta) = (%.2f, %.2f, %.2f)', x, y, math.degrees(yaw))

        pub.publish(vel)
        rospy.sleep(0.1)
        diff = rospy.get_time() - start_time
    vel.linear.x = 0.0
    pub.publish(vel)


def main():
    rospy.init_node('simple_move')
    rospy.loginfo("C19XXX ロボット　太郎")  # 受講者の情報を表示する。
    rospy.sleep(1)  # 起動直後は rospy.Time.now() がゼロを返す．
    listener = tf.TransformListener()  # このクラスのインスタンス生成は一度だけ。
    go_straight_by_distance_with_localization(listener, 2.0, 3.0)


if __name__ == '__main__':
    main()
*/
