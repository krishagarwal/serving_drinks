#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <Eigen/Geometry>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv) {
    ros::init(argc, argv, "simle_navigation_goals");

    ROS_INFO_STREAM("Start new");
    MoveBaseClient ac("/marvin/move_base", true);

    ROS_INFO_STREAM("Started waiting for server");
    while (!ac.waitForServer(ros::Duration(1.0)))
        ROS_INFO_STREAM("Still waiting for signal");
    ROS_INFO_STREAM("Server ready");

    move_base_msgs::MoveBaseGoal goal;

    goal.target_pose.header.frame_id = "/marvin/base_footprint";
    goal.target_pose.header.stamp = ros::Time::now();
    // goal.child_frame_id = "marvin_dest";

    goal.target_pose.pose.position.x = 1.0;
    goal.target_pose.pose.position.y = 1.0;
    goal.target_pose.pose.position.z = 1.0;

    goal.target_pose.pose.orientation.x = 0.0;
    goal.target_pose.pose.orientation.y = 0.0;
    goal.target_pose.pose.orientation.z = 0.0;
    goal.target_pose.pose.orientation.w = 1.0;

    ROS_INFO_STREAM("Sending goal");

    ac.sendGoal(goal);

    ROS_INFO_STREAM("Sent goal");


    ac.waitForResult();
    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO_STREAM("Moved 1 m forward");
    else
        ROS_INFO_STREAM("Failed to move 1 m forward");
    return 0;
    
    // ros::NodeHandle node;
    // tf2_ros::Buffer tfBuffer;
    // tf2_ros::TransformListener tfListener(tfBuffer);



    // ros::Rate rate(30.0);
    // while (node.ok()) {
    //     geometry_msgs::TransformStamped transformStamped;
    //     try {
    //         transformStamped = tfBuffer.lookupTransform("odom", "base_link", ros::Time(0));
    //     } catch (tf2::TransformException &ex) {
    //         ROS_WARN("%s", ex.what());
    //         ros::Duration(1.0).sleep();
    //         continue;
    //     }
    //     tf2_ros::TransformBroadcaster br;
    //     geometry_msgs::TransformStamped outTF;
    //     outTF.header.stamp = transformStamped.header.stamp;
    //     outTF.header.frame_id = transformStamped.child_frame_id;
    //     outTF.child_frame_id = "demonstration_frame";
        
    //     outTF.transform.translation.x = 2;
    //     outTF.transform.translation.y = 0;
    //     outTF.transform.translation.z = 0;
        
    //     // outTF.transform.rotation.x = 0;
    //     // outTF.transform.rotation.y = 0;
    //     // outTF.transform.rotation.z = 0;
    //     // outTF.transform.rotation.w = 1;

    //     Eigen::MatrixXd axis(3, 1);
    //     axis(0, 0) = 0.0;
    //     axis(1, 0) = 0.0;
    //     axis(2, 0) = 1.0;

    //     Eigen::AngleAxisd aa(0.25 * M_PI, axis);
    //     Eigen::Quaterniond quat(aa);

    //     outTF.transform.rotation.x = quat.x();
    //     outTF.transform.rotation.y = quat.y();
    //     outTF.transform.rotation.z = quat.z();
    //     outTF.transform.rotation.w = quat.w();

    //     ROS_INFO_STREAM("translation x = " << outTF.transform.translation.x);
    //     ROS_INFO_STREAM("translation y = " << outTF.transform.translation.y);
    //     ROS_INFO_STREAM("translation z = " << outTF.transform.translation.z);
    //     ROS_INFO_STREAM("rotation x = " << outTF.transform.rotation.x);
    //     ROS_INFO_STREAM("rotation y = " << outTF.transform.rotation.y);
    //     ROS_INFO_STREAM("rotation z = " << outTF.transform.rotation.z);


    //     br.sendTransform(outTF);
    // }
}