/*refer the documentation on move_base
http://wiki.ros.org/move_base */

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <move_base_msgs/MoveBaseFeedback.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <actionlib_msgs/GoalID.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_broadcaster.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

enum location{
    INIT = 0,
    CORNER1 = 1,
    CORNER2 = 2,
    CORNER3 = 3,
    CORNER4 = 4
};

void getLocationValues(location loca, move_base_msgs::MoveBaseGoal& msgs){
    msgs.target_pose.header.frame_id = "world";
    msgs.target_pose.header.stamp = ros::Time::now();
    switch (loca)
    {
        case INIT:
            msgs.target_pose.pose.position.x = -1.5;
            msgs.target_pose.pose.position.y = -1.5;
            msgs.target_pose.pose.orientation.w = 1.0;
            break;

        case CORNER1:
            msgs.target_pose.pose.position.x = -14.0;
            msgs.target_pose.pose.position.y = 1.0;
            msgs.target_pose.pose.orientation.z = 0.0;
            msgs.target_pose.pose.orientation.w = 1.0;
            break;

        case CORNER2:
            msgs.target_pose.pose.position.x = 11.0;
            msgs.target_pose.pose.position.y = 1.0;
            msgs.target_pose.pose.orientation.z = 0.0;
            msgs.target_pose.pose.orientation.w = 1.0;
            break;

        case CORNER3:
            msgs.target_pose.pose.position.x = -14.0;
            msgs.target_pose.pose.position.y = -10.0;
            msgs.target_pose.pose.orientation.z = 0.0;
            msgs.target_pose.pose.orientation.w = 1.0;
            break;
        
        case CORNER4:
            msgs.target_pose.pose.position.x = 11.0;
            msgs.target_pose.pose.position.y = -10.0;
            msgs.target_pose.pose.orientation.z = 0.0;
            msgs.target_pose.pose.orientation.w = 1.0;
            break;
    
        default:
            msgs.target_pose.pose.position.x = 0;
            msgs.target_pose.pose.position.y = -1.5;
            msgs.target_pose.pose.orientation.w = 1.0;
            break;
    }
}


int main(int argc, char** argv){
    ros::init(argc, argv, "simple_goal_node");
    ros::NodeHandle nh;
    
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);
    ros::Publisher rev_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);

    while(!ac.waitForServer(ros::Duration(10.0))){ROS_INFO("Waiting for the move_base server");} 
    //while(!cancelAC.waitForServer(ros::Duration(10.0))){ROS_INFO("Waiting for the action server");} 

    move_base_msgs::MoveBaseGoal goal;
    geometry_msgs::Twist vel;
    // get initial goal
    getLocationValues(INIT, goal);
    ac.sendGoal(goal);
    ac.waitForResult();
    location loc = CORNER1;
    while(ros::ok()){
        location prev_loc = loc;
        switch (loc)
        {
            case CORNER1:
                getLocationValues(CORNER1, goal);
                loc = CORNER4;
                break;

            case CORNER2:
                getLocationValues(CORNER2, goal);
                loc = CORNER1;
                break;

            case CORNER3:
                getLocationValues(CORNER3, goal);
                loc = CORNER2;
                break;
            
            case CORNER4:
                getLocationValues(CORNER4, goal);
                loc = CORNER3;
                break;
            
            default:
                getLocationValues(INIT, goal);
                loc = CORNER1;
                break;
        }

        ac.sendGoal(goal);
        ac.waitForResult();
        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
            ROS_INFO("Reached goal");
        }else{
            ROS_INFO("Fail to reach the goal");
            ROS_INFO("Cancelling the current goal");
            ac.cancelAllGoals();
            ac.waitForResult();

            vel.linear.x = -0.5;
            ros::Rate r(10);
            while(ros::ok){
                rev_pub.publish(vel);
                ros::spinOnce();
                r.sleep();
            }
            ROS_INFO("--- Setting previous goal");
            loc = prev_loc;
        }
    }
    return 0;
}