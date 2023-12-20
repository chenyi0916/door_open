// ROS
#include <ros/ros.h>

// MoveIt!
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

// TF2
#include <eigen_conversions/eigen_msg.h>
#include <tf2/LinearMath/Quaternion.h>

const double tau = 2 * M_PI;

#if defined(__linux__) || defined(__APPLE__)
#include <fcntl.h>
#include <termios.h>
#define STDIN_FILENO 0
#elif defined(_WIN32) || defined(_WIN64)
#include <conio.h>
#endif

#include <stdlib.h>
#include <stdio.h>
#define ESC_ASCII_VALUE                 0x1b

int getch() {
#if defined(__linux__) || defined(__APPLE__)
  struct termios oldt, newt;
  int ch;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  ch = getchar();
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  return ch;
#elif defined(_WIN32) || defined(_WIN64)
  return _getch();
#endif
}

int kbhit(void) {
#if defined(__linux__) || defined(__APPLE__)
  struct termios oldt, newt;
  int ch;
  int oldf;

  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

  ch = getchar();

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);

  if (ch != EOF) {
    ungetc(ch, stdin);
    return 1;
  }

  return 0;
#elif defined(_WIN32) || defined(_WIN64)
  return _kbhit();
#endif
}

std::string tf_prefix_ = "robot1/";

moveit::core::MoveItErrorCode moveToCartesianPose(moveit::planning_interface::MoveGroupInterface &group,
                                                           geometry_msgs::Pose target_pose)
{
    group.setStartStateToCurrentState();
    group.setPoseTarget(target_pose);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    auto error_code = group.plan(my_plan);
    bool success = (error_code == moveit::core::MoveItErrorCode::SUCCESS);

    moveit_visual_tools::MoveItVisualTools visual_tools("world");
    // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

    ROS_INFO("Move planning (cartesian pose goal) %s", success ? "SUCCESS" : "FAILED");
    if (success)
    {
        error_code = group.execute(my_plan);
    }
    return error_code;
}

moveit::core::MoveItErrorCode moveToNamedPose(moveit::planning_interface::MoveGroupInterface &group,
                                                           std::string named_pose)
{
    robot_state::RobotState start_state(*group.getCurrentState());
    group.setStartState(start_state);
    group.setNamedTarget(named_pose);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    auto error_code = group.plan(my_plan);
    bool success = (error_code == moveit::core::MoveItErrorCode::SUCCESS);

    ROS_INFO("Move planning (named pose goal) %s", success ? "SUCCESS" : "FAILED");
    if (success)
    {
        error_code = group.execute(my_plan);
    }
    return error_code;
}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "robotarm_contorl");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::NodeHandle nh;

  ROS_INFO_STREAM("Setting up MoveIt.");

  moveit::planning_interface::MoveGroupInterface arm_group("arm1");
  moveit::planning_interface::MoveGroupInterface gripper_group("gripper1");
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  arm_group.setPlanningTime(25.0);
  arm_group.setPlannerId("RRTstar");
  arm_group.setMaxAccelerationScalingFactor(0.05);
  arm_group.setMaxVelocityScalingFactor(0.05);

  ROS_INFO_STREAM("Starting robotarm_test");
  moveToNamedPose(arm_group, "rest");

  ROS_INFO_STREAM("Gripper open");
  moveToNamedPose(gripper_group, "open");

  // experiment grasp pose
  geometry_msgs::Pose pre_grasp_pose;
  pre_grasp_pose.position.x = 0.5240900951064722+0.02+0.2;
  pre_grasp_pose.position.y = -0.18096880604576562;
  pre_grasp_pose.position.z = 1.00022676879561-0.505;
  pre_grasp_pose.orientation.w = 0.707;
  pre_grasp_pose.orientation.x = -0.707;
  pre_grasp_pose.orientation.y = 1e-6;
  pre_grasp_pose.orientation.z = 1e-6;

  //trajectory center
  geometry_msgs::Pose center_pose;
  center_pose.position.x = 0.5240900951064722+0.02+0.2;
  center_pose.position.y = -0.83;
  center_pose.position.z = 1.00022676879561-0.505;
  center_pose. orientation.w = 1e-6;


  ROS_INFO_STREAM("Move to pre_grasp pose");
  moveToCartesianPose(arm_group, pre_grasp_pose);

  ROS_INFO_STREAM("Gripper close");
  moveToNamedPose(gripper_group, "close");

  //open-door trajectory
  std::vector<geometry_msgs::Pose> waypoints;
  
  geometry_msgs::Pose target_pose;

  double centerA = center_pose.position.x;
  double centerB = center_pose.position.y;
  double radius = abs(center_pose.position.y - (-0.18096880604576562));

  tf2::Quaternion handleQuaternion;

  for(double th = 0.04; th < 0.167 * M_PI; th = th + 0.02)
  {
    handleQuaternion.setRPY( 0.5 * M_PI, 0, th );
    target_pose.position.x = pre_grasp_pose.position.x - radius * sin(th);
    target_pose.position.y = pre_grasp_pose.position.y - radius + radius * cos(th);
    target_pose.position.z = pre_grasp_pose.position.z;
    target_pose.orientation.w = handleQuaternion.getW();
    target_pose.orientation.x = handleQuaternion.getX();
    target_pose.orientation.y = handleQuaternion.getY();
    target_pose.orientation.z = handleQuaternion.getZ();

    ROS_INFO_STREAM("Opening the door");
    moveToCartesianPose(arm_group, target_pose);
    waypoints.push_back(target_pose);
  }

  // arm_group.setPlanningTime(15.0);
  // arm_group.setMaxAccelerationScalingFactor(0.05);
  // arm_group.setMaxVelocityScalingFactor(0.02);


  // moveit_msgs::RobotTrajectory trajectory;

  // const double jump_threshold = 0.0;
  // const double eef_step = 0.01;
  // double fraction = arm_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

  // ROS_INFO_STREAM("Opening the door");
  // moveit_visual_tools::MoveItVisualTools visual_tools("world");
  // arm_group.execute(trajectory);

  ROS_INFO_STREAM("Finished.");
  return 0;
}