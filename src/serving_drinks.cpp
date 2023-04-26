#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <eigen3/Eigen/Geometry>

#include "SKConfig.h"
#include "SKPFaceDetector.h"
#include "SKPVideoDisplay.h"
#include "SKWrapper.h"

#include <gtk/gtk.h>
#include <iostream>

// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()
#include <stdio.h>
#include <string.h>

bool keepRunning = true;
GtkWidget *window;

void *skwThread(void *data) {
 
    DoOnce *skw = (DoOnce *)data;
    while(keepRunning) {
        // take pic & save to file
        skw->doOnce(); 
        // move motor
    }
    return NULL;
}

gboolean exit_program(GtkWidget *widget, GdkEvent *event, gpointer data) {
    //if menu closed exit program entirely. 
    keepRunning = false;
    //exit(0);
    return TRUE;
}

static void buildUI (GtkApplication *app, gpointer user_data){
    window = gtk_application_window_new(app);
    gtk_window_set_title (GTK_WINDOW (window), "Single Kinect" );
    gtk_window_set_default_size(GTK_WINDOW(window), 1920, 1080 );

    gtk_widget_add_events(window, GDK_KEY_PRESS_MASK);

    SKPVideoDisplay* skpVideoDisplay = (SKPVideoDisplay*) user_data;

    skpVideoDisplay->buildWidgets(window);

    g_signal_connect(window, "destroy", G_CALLBACK(exit_program), NULL);
    gtk_widget_show_all (window);
}

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv) {
    g_thread_init(NULL);
    ros::init(argc, argv, "simple_navigation_goals");
    
    ROS_INFO_STREAM("Start new");
    MoveBaseClient ac("/move_base", true);
    ROS_INFO_STREAM("Started waiting for server");
    while (!ac.waitForServer(ros::Duration(1.0)))
        ROS_INFO_STREAM("Still waiting for signal");
    ROS_INFO_STREAM("Server ready");

    SKConfig skc;
    SKWrapper skw(skc);
    
    SKPFaceDetector spfd(skw);
    skw.addRecipient(&spfd);

    do {
        skw.doOnce();
    } while (!spfd.chooseTarget());

    while (!spfd.findTargetId())
        skw.doOnce();

    while (true) {
        while (!spfd.find3DTargetPose()) {
            skw.doOnce();
            spfd.findTargetId();
        }
        
        move_base_msgs::MoveBaseGoal goal;

        goal.target_pose.header.frame_id = "nav_kinect_depth_frame";
        goal.target_pose.header.stamp = ros::Time::now();

        k4a_float3_t target_pos = spfd.getTargetPosition();
        // goal.child_frame_id = "marvin_dest";

        goal.target_pose.pose.position.x = target_pos.xyz.x;
        goal.target_pose.pose.position.y = target_pos.xyz.y;
        goal.target_pose.pose.position.z = 0.0;

        goal.target_pose.pose.orientation.x = 0.0;
        goal.target_pose.pose.orientation.y = 0.0;
        goal.target_pose.pose.orientation.z = 0.0;
        goal.target_pose.pose.orientation.w = 1.0;

        ROS_INFO_STREAM("Sending goal");
        ac.sendGoal(goal);
        ROS_INFO_STREAM("Sent goal");
    }

    // ac.waitForResult();
    // if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    //     ROS_INFO_STREAM("Moved 1 m forward");
    // else
    //     ROS_INFO_STREAM("Failed to move 1 m forward");
    // return 0;
    
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
