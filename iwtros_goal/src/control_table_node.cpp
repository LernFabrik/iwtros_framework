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
#include <dynamic_reconfigure/server.h>
#include <dynamic_reconfigure/client.h>
#include <dwa_local_planner/DWAPlannerConfig.h>
#include <move_base/MoveBaseConfig.h>

#include <iwtros_goal/reversePID.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <clear_costmap_recovery/clear_costmap_recovery.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

tf2_ros::Buffer tfBuffer;

double roll, pitch, yaw;
double fts_roll, fts_pitch, fts_yaw;
geometry_msgs::Quaternion quaternion;
double maxVel = -0.5;
double minAng = -0.3;
double maxAng = 0.3;

double x, y, z;
geometry_msgs::Quaternion quad;

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

void dwaCallback(dwa_local_planner::DWAPlannerConfig &config, uint32_t level){
    ROS_INFO("max vel = %f, min vel = %f", config.max_vel_x, config.min_vel_x);
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
    x = msg->pose.pose.position.x;
    y = msg->pose.pose.position.y;
    z = msg->pose.pose.position.z;
    quad.x = msg->pose.pose.orientation.x;
    quad.y = msg->pose.pose.orientation.y;
    quad.z = msg->pose.pose.orientation.z;
    quad.w = msg->pose.pose.orientation.w;
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
    ros::Publisher goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 20);
    /*--------------------------------*/

    /**Position publisher**/
    ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseWithCovariance>("/iiwa/tbl_cmd_vel", 20);
    ros::Subscriber odom_sub = nh.subscribe("odom", 100, odomCallback);
    //dynamic_reconfigure::Server<dwa_local_planner::DWAPlannerConfig> server;
    //dynamic_reconfigure::Server<dwa_local_planner::DWAPlannerConfig>::CallbackType f;
    //f = boost::bind(&dwaCallback, _1, _2);
    //server.setCallback(f);
    
    

    ros::Rate rate(30.0);
    double currentTime = ros::Time::now().toSec();
    double prevTime = currentTime;

    while(ros::ok()){
        geometry_msgs::TransformStamped stampedTfIIWA;
        geometry_msgs::TransformStamped stampedTfUR5;
        geometry_msgs::TransformStamped stampedFTS;
        geometry_msgs::PoseStamped goal2;
        geometry_msgs::PoseWithCovariance table_pose;

        getTransforms("world", "iiwa_table_base", stampedTfIIWA);

        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = "world";
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.position.x = stampedTfIIWA.transform.translation.x;
        /* this method is not proficent it is better to get axis of rotation*/
        if(stampedTfIIWA.transform.translation.y < 0) goal.target_pose.pose.position.y = stampedTfIIWA.transform.translation.y - 1.5;
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
        /*Go backwards*/
        double s;
        nh.getParam("/move_base/DWAPlannerROS/max_vel_x", s);
        ROS_INFO("max velocity %f", s);
        nh.getParam("/move_base/DWAPlannerROS/min_vel_x", s);
        ROS_INFO("min velocity %f", s);
        /*double stpr;
        stpr = 0;
        nh.setParam("/move_base/DWAPlannerROS/max_vel_x", stpr);
        stpr = -0.50;
        nh.setParam("/move_base/DWAPlannerROS/min_vel_x", stpr);
        //nh.setParam()*/

        /*dynamic_reconfigure::ReconfigureRequest srv_req;
        dynamic_reconfigure::ReconfigureResponse srv_res;
        dynamic_reconfigure::DoubleParameter dwaConf;
        dynamic_reconfigure::DoubleParameter dwaConf2;
        dynamic_reconfigure::Config config;

        dwaConf.name = "min_vel_x";
        dwaConf.value = -0.5;
        config.doubles.push_back(dwaConf);
        srv_req.config = config;
        ros::service::call("/move_base/DWAPlannerROS", srv_req, srv_res);*/
        system("rosrun dynamic_reconfigure dynparam set /move_base/DWAPlannerROS max_vel_x 0.0");
        system("rosrun dynamic_reconfigure dynparam set /move_base/DWAPlannerROS min_vel_x -0.5");
        system("rosrun dynamic_reconfigure dynparam set /move_base/DWAPlannerROS xy_goal_tolerance 0.0125");
        goal.target_pose.pose.position.y = stampedTfIIWA.transform.translation.y - 1;
        ac.sendGoal(goal);
        ac.waitForResult();

        system("rosrun dynamic_reconfigure dynparam set /move_base/DWAPlannerROS max_rot_vel 0.3");
        system("rosrun dynamic_reconfigure dynparam set /move_base/DWAPlannerROS min_rot_vel -0.3");
        system("rosrun dynamic_reconfigure dynparam set /move_base/DWAPlannerROS yaw_goal_tolerance 0.001");
        

        nh.getParam("/move_base/DWAPlannerROS/max_vel_x", s);
        ROS_INFO("---max velocity %f", s);
        nh.getParam("/move_base/DWAPlannerROS/min_vel_x", s);
        ROS_INFO("---min velocity %f", s);

        goal.target_pose.pose.position.y = stampedTfIIWA.transform.translation.y - 0.75;
        ac.sendGoal(goal);
        ac.waitForResult();

        system("rosrun dynamic_reconfigure dynparam set /move_base/DWAPlannerROS xy_goal_tolerance 0.25");
        system("rosrun dynamic_reconfigure dynparam set /move_base/DWAPlannerROS max_rot_vel 0.075");
        system("rosrun dynamic_reconfigure dynparam set /move_base/DWAPlannerROS min_rot_vel -0.075");
        
        goal.target_pose.pose.position.y = stampedTfIIWA.transform.translation.y;
        ac.sendGoal(goal);
        ac.waitForResult();
        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
            ROS_INFO("Reached goal");
        }else{
            ROS_INFO("Fail to reach the goal");
        }

        /* Tabel move with FTS Test*/
        system("rosrun dynamic_reconfigure dynparam set /move_base/DWAPlannerROS max_vel_x 0.25");
        system("rosrun dynamic_reconfigure dynparam set /move_base/DWAPlannerROS min_vel_x 0.0");
        system("rosrun dynamic_reconfigure dynparam set /move_base/DWAPlannerROS max_rot_vel 0.5");
        system("rosrun dynamic_reconfigure dynparam set /move_base/DWAPlannerROS min_rot_vel -0.025");
        goal2.header.frame_id = "world";
        goal2.header.stamp = ros::Time::now();
        goal2.pose.position.x = -8.0;
        goal2.pose.position.y = 1.7;
        goal2.pose.orientation.z = -0.6923232;
        goal2.pose.orientation.w = 0.7215876;
        //goal_pub.publish(goal2);
        while(ros::ok()){
            table_pose.pose.position.x = x;
            table_pose.pose.position.y = y;
            table_pose.pose.position.z = 0;
            quad_to_Euler(quad, roll, pitch, yaw);
            yaw -= M_PI / 2;
            tf2::Quaternion q1;
            q1.setRPY(0,0,yaw);
            table_pose.pose.orientation.x = q1.x();
            table_pose.pose.orientation.y = q1.y();
            table_pose.pose.orientation.z = q1.z();
            table_pose.pose.orientation.w = q1.w();
            pose_pub.publish(table_pose);
        
            rate.sleep();
            ros::spinOnce();
        }
        
        ros::spin();
    }
    return 0;
}