#include <ros/ros.h>
#include <memory>
// MoveitCpp
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>
#include <moveit/robot_state/conversions.h>

#include <geometry_msgs/PointStamped.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <tf/transform_listener.h>

namespace rvt = rviz_visual_tools;

planning_interface::MotionPlanResponse plan_to_target(moveit_cpp::PlanningComponentPtr &planning_components, tf::StampedTransform transform)
{
    // set the start state of the plan to the current state of the robot
    planning_components->setStartStateToCurrentState();

    // set a goal using PoseStamped
    geometry_msgs::PoseStamped target_pose;
    target_pose.header.frame_id = "base_link";
    target_pose.pose.orientation.x = transform.getRotation().x();
    target_pose.pose.orientation.y = transform.getRotation().y();
    target_pose.pose.orientation.z = transform.getRotation().z();
    target_pose.pose.orientation.w = transform.getRotation().w();
    target_pose.pose.position.x = transform.getOrigin().x();
    target_pose.pose.position.x = transform.getOrigin().y();
    target_pose.pose.position.x = transform.getOrigin().z();
    planning_components->setGoal(target_pose, "link_6");

    ROS_INFO("Target:\n\tT:\n\t\tx:%f\n\t\ty:%f\n\t\tz:%f\n\tR:\n\t\tx:%f\n\t\ty:%f\n\t\tz:%f\n\t\tw:%f", 
        target_pose.pose.position.x, 
        target_pose.pose.position.y, 
        target_pose.pose.position.z, 
        target_pose.pose.orientation.x, 
        target_pose.pose.orientation.y, 
        target_pose.pose.orientation.z, 
        target_pose.pose.orientation.w);

    // PlanningComponents computes the plan and visualizes it.
    return planning_components->plan();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "target_planner");
    ros::NodeHandle nh("/target_planner");
    ros::AsyncSpinner spinner(4);
    spinner.start();

    static const std::string PLANNING_GROUP = "manipulator";

    // Otherwise robot with zeros joint_states
    ros::Duration(1.0).sleep();

    ROS_INFO("Starting MoveIt.");

    auto moveit_cpp_ptr = std::make_shared<moveit_cpp::MoveItCpp>(nh);
    moveit_cpp_ptr->getPlanningSceneMonitor()->providePlanningSceneService();

    moveit_cpp::PlanningComponentPtr planning_components = std::make_shared<moveit_cpp::PlanningComponent>(PLANNING_GROUP, moveit_cpp_ptr);
    auto robot_model_ptr = moveit_cpp_ptr->getRobotModel();
    auto robot_start_state = planning_components->getStartState();
    auto joint_model_group_ptr = robot_model_ptr->getJointModelGroup(PLANNING_GROUP);

    moveit_visual_tools::MoveItVisualTools visual_tools("base_link", rvt::RVIZ_MARKER_TOPIC,
                                                        moveit_cpp_ptr->getPlanningSceneMonitor());
    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();

    visual_tools.trigger();

    tf::TransformListener listener;

    while (nh.ok())
    {
        tf::StampedTransform transform;
        try
        {
            listener.waitForTransform("world", "object", ros::Time(0), ros::Duration(1));
            listener.lookupTransform("world", "object", ros::Time(0), transform);
            ROS_INFO("TF Lookup Success");
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR_ONCE("%s", ex.what());
            continue;
            // ros::Duration(1).sleep();
        }

        // Plan
        auto plan_solution = plan_to_target(planning_components, transform);

        // Check if PlanningComponents succeeded in finding the plan
        if (plan_solution)
        {
            visual_tools.publishTrajectoryLine(plan_solution.trajectory_, joint_model_group_ptr);
            visual_tools.trigger();
            planning_components->execute(); // Execute the plan
        }
    }

    // visual_tools.deleteAllMarkers();

    ROS_INFO("Shutting down.");
    ros::waitForShutdown();
}
