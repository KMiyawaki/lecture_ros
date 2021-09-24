#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "simple_navigation_goals");

  // Action Client
  MoveBaseClient ac("move_base", true);

  // アクションサーバーが起動するまで待つ。引数はタイムアウトの時間(秒）
  while (!ac.waitForServer(ros::Duration(5.0)))
  {
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  // ゴールの生成
  goal.target_pose.header.frame_id = "base_link"; // ロボットローカル座標系
  goal.target_pose.header.stamp = ros::Time::now(); // 現在時刻

  goal.target_pose.pose.position.x = 1.0;
  goal.target_pose.pose.orientation.w = 1.0;

  ROS_INFO("Sending goal");
  ac.sendGoal(goal);
  bool finished = ac.waitForResult(ros::Duration(30));
  actionlib::SimpleClientGoalState state = ac.getState();

  if (finished)
  {
    if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_INFO("SUCCEEDED");
    }
    else
    {
      ROS_ERROR("Failed (%s)", state.getText().c_str());
    }
  }
  else
  {
    ROS_ERROR("Time out");
  }
  return 0;
}
