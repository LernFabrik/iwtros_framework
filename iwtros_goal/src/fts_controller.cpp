#include <iwtros_goal/fts_controller.hpp>

namespace iwtros{
    ftsControl::ftsControl() : ac("move_base", true){
        ros::NodeHandle nh_("~");

        nh_.getParam("iiwa_table", iiwa_table_topic);
        nh_.getParam("ur5_table", ur5_table_topic);
        nh_.getParam("panda_table", panda_table_topic);
        nh_.getParam("worldFrame", worldFrame);
        nh_.getParam("iiwaFrame", iiwaFrame);
        nh_.getParam("ur5Frame", ur5Frame);
        nh_.getParam("pandaFrame", pandaFrame);

        /* Subscribers*/
        ros::Subscriber startSub = node_.subscribe<iwtros_msgs::ftsControl>("startFtsOperation", 10, boost::bind(&ftsControl::ftsStartCallback, this, _1, _2));
        ftsOdom = node_.subscribe("odom", 100, &ftsControl::ftsOdomCallback, this);
        
        /*Table position control publishers*/
        this->tableControl_iiwaPub = node_.advertise<geometry_msgs::PoseWithCovariance>(iiwa_table_topic.c_str(), 20);
        this->tableControl_ur5Pub = node_.advertise<geometry_msgs::PoseWithCovariance>(ur5_table_topic.c_str(), 20);
        this->tableControl_pandaPub = node_.advertise<geometry_msgs::PoseWithCovariance>(panda_table_topic.c_str(), 20);

        /*FTS goal action client & simple goal publisher*/
        while(!ac.waitForServer(ros::Duration(10.0))){ROS_INFO("Waiting for the move_base server");}
        fts_goalPub = node_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 20);

        /* Initialize tf2*/
        tf2_ros::TransformListener tf2Listener(tf2Buffer);

        this->lockCell =  false;
    }

    void ftsControl::ftsOdomCallback(const nav_msgs::Odometry::ConstPtr &msg){
        if(this->lockCell == true){
            this->ftsPose.pose.position.x = msg->pose.pose.position.x;
            this->ftsPose.pose.position.y = msg->pose.pose.position.y;
            this->ftsPose.pose.position.z = msg->pose.pose.position.z;
            this->ftsPose.pose.orientation = msg->pose.pose.orientation;
            ftsControl::quadToEuler(this->ftsPose.pose.orientation, this->roll, this->pitch, this->yaw);
            this->yaw -= M_PI / 2;
            tf2::Quaternion q;
            q.setRPY(0,0,this->yaw);
            this->ftsPose.pose.orientation.x = q.x();
            this->ftsPose.pose.orientation.y = q.y();
            this->ftsPose.pose.orientation.z = q.z();
            this->ftsPose.pose.orientation.w = q.w();

            switch(seletTable){
                case IIWA:
                    this->tableControl_iiwaPub.publish(this->ftsPose);
                    break;
                case UR5:
                    this->tableControl_ur5Pub.publish(this->ftsPose);
                    break;
                case PANDA:
                    this->tableControl_pandaPub.publish(this->ftsPose);
                    break;
                default:
                    break;
            }
        }
    }

    void ftsControl::getTransforms(std::string parent, std::string child,  geometry_msgs::TransformStamped& stamped){
        try{
            stamped = tf2Buffer.lookupTransform(parent, child, ros::Time(0));
        }catch(tf2::TransformException& e){
            ROS_INFO("%s", e.what());
            ros::Duration(0.1).sleep();
        }   
    }

    void ftsControl::goToTableLocation(select_table sel){
        switch (sel){
            case IIWA:
                ftsControl::getTransforms(this->worldFrame, this->iiwaFrame, this->stampedtf2Cell);
                break;
            case UR5:
                ftsControl::getTransforms(this->worldFrame, this->ur5Frame, this->stampedtf2Cell);
                break;
            case PANDA:
                ftsControl::getTransforms(this->worldFrame, this->pandaFrame, this->stampedtf2Cell);
                break;
            default:
                break;
        }

        this->goal.target_pose.header.frame_id = this->worldFrame;
        this->goal.target_pose.header.stamp = ros::Time::now();
        this->goal.target_pose.pose.position.x = this->stampedtf2Cell.transform.translation.x;
        /* Dirty method of offsetting the y - axis because first FTS should go in front of the Standardzell
        Here we assume that Standardzell's y-axis lies either positively in positive side of the y-axis 
        or negetively in the negetiveside of the y-axis. If any the above condition is not met then this 
        method will not work. Solution would be offset correction according to the orientation of the axis*/
        if(this->stampedtf2Cell.transform.translation.y < 0) this->goal.target_pose.pose.position.y = this->stampedtf2Cell.transform.translation.y - 1.5;
        else if(this->stampedtf2Cell.transform.translation.y >= 0) this->goal.target_pose.pose.position.y = this->stampedtf2Cell.transform.translation.y + 1.5;
        this->goal.target_pose.pose.position.z = 0;
        ftsControl::quadToEuler(this->stampedtf2Cell.transform.rotation, this->roll, this->pitch, this->yaw);
        this->yaw += M_PI / 2;
        tf2::Quaternion q;
        q.setRPY(this->roll, this->pitch, this->yaw);
        this->goal.target_pose.pose.orientation.x = q.x();
        this->goal.target_pose.pose.orientation.y = q.y();
        this->goal.target_pose.pose.orientation.z = q.z();
        this->goal.target_pose.pose.orientation.w = q.w();

        this->ac.sendGoal(this->goal);
        this->ac.waitForResult();
        if(ac.getState == actionlib::SimpleClientGoalState::SUCCEEDED) ROS_INFO("Reached goal infront of Standardzelle");
        else ("Failed to reach goal");
        //ftsControl::reverseDocking();
    }

    void ftsControl::reverseDocking(){
        /* This function is used only after "goToTableLocation"*/
        ftsControl::setDynamicParam();
        this->goal.target_pose.pose.position.y = this->stampedtf2Cell.transform.translation.y;
        this->ac.sendGoal(this->goal);
        ac.waitForResult();
        if(ac.getState == actionlib::SimpleClientGoalState::SUCCEEDED) ROS_INFO("Reached goal");
        else ROS_INFO("Failed to reach goal");
        ftsControl::withTableDynamicParam();

        /* Once the FTS is under the Standardzelle, Lock the position FTS to the Standardzelle*/
        this->lockCell = true;
            
    }

    void ftsControl::ftsStartCallback(const iwtros_msgs::ftsControl::ConstPtr& msg){
        this->endGoal.target_pose.pose.position.x = msg->pose.position.x;
        this->endGoal.target_pose.pose.position.y = msg->pose.position.y;
        this->endGoal.target_pose.pose.position.z = msg->pose.position.z;
        this->endGoal.target_pose.pose.orientation.x = msg->pose.orientation.x;
        this->endGoal.target_pose.pose.orientation.y = msg->pose.orientation.y;
        this->endGoal.target_pose.pose.orientation.z = msg->pose.orientation.z;
        this->endGoal.target_pose.pose.orientation.w = msg->pose.orientation.w;

        this->chooseTable = msg->selRobot;
        if(this->chooseTable == 0) seletTable = IIWA;
        else if(this->chooseTable == 1) seletTable = UR5;
        else if(this->chooseTable == 2) seletTable = PANDA;
    }
}