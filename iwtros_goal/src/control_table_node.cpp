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
#include <geometry_msgs/Twist.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

#include <iwtros_goal/reversePID.h>

tf2_ros::Buffer tfBuffer;

double roll, pitch, yaw;
geometry_msgs::Quaternion quaternion;



static void quad_to_Euler(geometry_msgs::Quaternion& q, double& roll, double& pitch, double& yaw){
    //ROS_INFO("input quat %f, %f, %f, %f", q.x, q.y, q.z, q.w);
    double sinr_cosp = +2.0 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = +1.0 - 2.0 * (q.x * q.x + q.y * q.y);
    //ROS_ERROR("sinr %f, cosr %f", sinr_cosp, cosr_cosp);
    roll = atan2(sinr_cosp , cosr_cosp);

    double sinp = 2 * (q.w * q.y + q.z * q.x);
    if(fabs(sinp >= 1)) pitch = copysign(M_PI /2, sinp);
    else pitch = asin(sinp);

    double siny_cosp = +2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = +1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    yaw = atan2(siny_cosp, cosy_cosp); 
}

void getTransforms(std::string parent, std::string child, geometry_msgs::TransformStamped& stamped){
    try{
        stamped = tfBuffer.lookupTransform(parent, child, ros::Time(0));
    }catch(tf2::TransformException &e){
        ROS_WARN("%s", e.what());
        ros::Duration(0.1).sleep();
    }
}

int main(int argc, char** argv){
    ros::init(argc, argv, "control_table_node");
    ros::NodeHandle nh;

    /* here require defination for tf are set*/
    tf2_ros::TransformListener tfListener(tfBuffer);
    /*---------------------------------------*/

    
    /*Move base action library setings*/
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);
    while(!ac.waitForServer(ros::Duration(10.0))){ROS_INFO("Waiting for the move_base server");} 
    /*--------------------------------*/

    /**cmd_vel publisher**/
    ros::Publisher cmd_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 20);

    ros::Rate rate(20.0);
    double currentTime = ros::Time::now().toSec();
    double prevTime = currentTime;

    while(ros::ok()){
        geometry_msgs::TransformStamped stampedTfIIWA;
        geometry_msgs::TransformStamped stampedTfUR5;
        geometry_msgs::TransformStamped stampedFTS;
        geometry_msgs::Twist cmdVel;

        getTransforms("world", "iiwa_table_base", stampedTfIIWA);

        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = "world";
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.position.x = stampedTfIIWA.transform.translation.x;
        /* this method is not proficent it is better to get axis of rotation*/
        if(stampedTfIIWA.transform.translation.y < 0) goal.target_pose.pose.position.y = stampedTfIIWA.transform.translation.y - 1;
        if(stampedTfIIWA.transform.translation.y >= 0) goal.target_pose.pose.position.y = 1 + stampedTfIIWA.transform.translation.y;
        goal.target_pose.pose.position.z = 0;

        quad_to_Euler(stampedTfIIWA.transform.rotation, roll, pitch, yaw);
        //ROS_ERROR("roll = %f, pitch = %f, yaw = %f", roll, pitch, yaw);
        yaw += M_PI / 2;
        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        goal.target_pose.pose.orientation.x = q.x();
        goal.target_pose.pose.orientation.y = q.y();
        goal.target_pose.pose.orientation.z = q.z();
        goal.target_pose.pose.orientation.w = q.w();

        ac.sendGoal(goal);
        ac.waitForResult();
        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
            ROS_INFO("Reached goal");
        }else{
            ROS_INFO("Fail to reach the goal");
        }
        /*********PID move FTS reverse***************/
        iwtros::PID pid(10, 0.01, 0.1);
        double output;
        currentTime = ros::Time::now().toSec();
        prevTime = currentTime;
        while(output != 0 && ros::ok()){
            currentTime = ros::Time::now().toSec();
            double dt = currentTime - prevTime;
            getTransforms("world", "base_link", stampedFTS);
            output = pid.calculate(0.05, stampedTfIIWA.transform.translation.y, stampedFTS.transform.translation.y);
            ROS_ERROR("PID output = %f", output);
            cmdVel.linear.x = - output * dt;
            ROS_ERROR("cmd_vel output = %f", cmdVel.linear.x);
            cmd_pub.publish(cmdVel);
            ros::spinOnce();
            rate.sleep();
            prevTime = currentTime;
        }
        ros::spin();
    }
    return 0;
}