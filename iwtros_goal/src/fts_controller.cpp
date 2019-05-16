#include <iwtros_goal/fts_controller.hpp>

namespace iwtros{
    ftsControl::ftsControl() : ac("move_base", true){
        ros::NodeHandle nh_("~");

        nh_.getParam("iiwa_table_topic", iiwa_table_topic);
        nh_.getParam("ur5_table_topic", ur5_table_topic);
        nh_.getParam("panda_table_topic", panda_table_topic);
        nh_.getParam("worldFrame", worldFrame);
        nh_.getParam("iiwaFrame", iiwaFrame);
        nh_.getParam("ur5Frame", ur5Frame);
        nh_.getParam("pandaFrame", pandaFrame);

        tf2_ros::TransformListener tfListner(this->tf2Buffer);

        /* Subscribers*/
        this->startSub = node_.subscribe<iwtros_msgs::ftsControl>("startFtsOperation", 10, boost::bind(&ftsControl::ftsStartCallback, this, _1));
        ftsOdom = node_.subscribe("odom", 100, &ftsControl::ftsOdomCallback, this);
        
        /*Table position control publishers*/
        this->tableControl_iiwaPub = node_.advertise<geometry_msgs::PoseWithCovariance>(iiwa_table_topic.c_str(), 20);
        this->tableControl_ur5Pub = node_.advertise<geometry_msgs::PoseWithCovariance>(ur5_table_topic.c_str(), 20);
        this->tableControl_pandaPub = node_.advertise<geometry_msgs::PoseWithCovariance>(panda_table_topic.c_str(), 20);
        this->deactScan_pub = node_.advertise<std_msgs::Bool>("deactivateScan", 10);
        this->cmdVel_pub = node_.advertise<geometry_msgs::Twist>("cmd_vel", 20);
        /*FTS goal action client & simple goal publisher*/
        while(!ac.waitForServer(ros::Duration(10.0))){ROS_INFO("Waiting for the move_base server");}
        fts_goalPub = node_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 20);

        this->lockCell =  false;
    }

    ftsControl::~ftsControl(){
        node_.shutdown();
    }

    void ftsControl::ftsOdomCallback(const nav_msgs::Odometry::ConstPtr &msg){
        //ROS_INFO("Odom callback is working");
        if(this->lockCell == true){
            //ROS_INFO("attching the table");
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

            switch(this->selCell){
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
        ROS_INFO("Looking for the transformation parent: %s, child: %s", parent.c_str(), child.c_str());
        try{
            this->tf2Buffer.clear();            //this will clear the old transforms and read the latest.
            stamped = this->tf2Buffer.lookupTransform(parent, child, ros::Time(0));
            ROS_INFO("Got the transforms");
        }catch(tf2::TransformException &e){
            ROS_ERROR("%s", e.what());
            ros::Duration(0.1).sleep();
        }   
    }

    template <class T>
    std::vector<std::vector<T> > ftsControl::Multiply(std::vector<std::vector<T> > &a, std::vector<std::vector<T> > &b){
        const int n = a.size();     //a rows
        const int m = a[0].size();  //a cols     
        const int p = b[0].size();  //b cols

        std::vector<std::vector<T> > c(n, std::vector<T>(p, 0));
        for (auto j = 0; j < p; ++j){                           //no. of cols of B Matrix = increment the cols of the matrix B
            for (auto i = 0; i < n; ++i){                       //no. of rows of A Matrix = increment the rows ofo the matrix A
                for (auto k = 0; k < m; ++k){                   //no. of cols of A MAtrix = col. element of the matrix A is multi. and added with row element of the matrix B
                    c[i][j] += a[i][k] * b[k][j];
                }
            }
        }
        for(auto i = 0; i < c[0].size(); ++i){
            for(auto j = 0; j < c.size(); ++j){
                ROS_ERROR ("%f", c[j][i]);
            }
        }
        return c;
    }

    void ftsControl::rotationalMatrix(geometry_msgs::Point& poseWrtWorld, geometry_msgs::Vector3 crntTablePosition, double yaw, double offvalue){
        std::vector<std::vector<double> > matrix(4, std::vector<double>(4));
        matrix[0][0] = cos(yaw); 
        matrix[0][1] = -sin(yaw); 
        matrix[0][2] = 0; 
        matrix[0][3] = crntTablePosition.x;
        matrix[1][0] = sin(yaw); 
        matrix[1][1] = cos(yaw); 
        matrix[1][2] = 0; 
        matrix[1][3] = crntTablePosition.y;
        matrix[2][0] = 0; 
        matrix[2][1] = 0; 
        matrix[2][2] = 1; 
        matrix[2][3] = crntTablePosition.z;
        matrix[3][0] = 0; 
        matrix[3][1] = 0; 
        matrix[3][2] = 0; 
        matrix[3][3] = 1;

        /*for(auto i = 0; i < matrix.size(); ++i){
            for(auto j = 0; j < matrix[0].size(); ++j){
                ROS_WARN ("%f", matrix[i][j]);
            }
        }*/

        std::vector<std::vector<double> > offset(4, std::vector<double>(1));
        offset[0][0] = 0;
        offset[1][0] = offvalue;
        offset[2][0] = 0;
        offset[3][0] = 1;

        /*for(auto i = 0; i < offset.size(); ++i){
            for(auto j = 0; j < offset[0].size(); ++j){
                ROS_WARN ("%f", offset[i][j]);
            }
        }*/

        auto c = ftsControl::Multiply(matrix, offset);
        poseWrtWorld.x = c[0][0];
        poseWrtWorld.y = c[1][0];
        poseWrtWorld.z = c[2][0];
    }

    void ftsControl::goToTableLocation(select_table sel){
        switch (sel){
            case IIWA:
                ROS_INFO("Moving IIWA Standarzelle");
                ftsControl::getTransforms(this->worldFrame.c_str(), this->iiwaFrame.c_str(), this->stampedtf2Cell);
                break;
            case UR5:
                ROS_INFO("Moving UR5 Standarzelle");
                ftsControl::getTransforms(this->worldFrame.c_str(), this->ur5Frame.c_str(), this->stampedtf2Cell);
                break;
            case PANDA:
                ROS_INFO("Moving PANDA Standarzelle");
                ftsControl::getTransforms(this->worldFrame.c_str(), this->pandaFrame.c_str(), this->stampedtf2Cell);
                break;
            default:
                break;
        }

        this->goal.target_pose.header.frame_id = this->worldFrame.c_str();
        this->goal.target_pose.header.stamp = ros::Time::now();
        ftsControl::quadToEuler(this->stampedtf2Cell.transform.rotation, this->roll, this->pitch, this->yaw);
        ftsControl::rotationalMatrix(this->goal.target_pose.pose.position, this->stampedtf2Cell.transform.translation, this->yaw, 1.5);
        this->goal.target_pose.pose.position.z = 0;
        this->yaw += M_PI / 2;
        tf2::Quaternion q;
        q.setRPY(this->roll, this->pitch, this->yaw);
        this->goal.target_pose.pose.orientation = tf2::toMsg(q);

        this->ac.sendGoal(this->goal);
        this->ac.waitForResult();
        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) ROS_INFO("Reached goal infront of Standardzelle");
        else ("Failed to reach goal");
        //ftsControl::reverseDocking();
    }

    void ftsControl::reverseDocking(){
        /* This function is used only after "goToTableLocation"*/
        ftsControl::setDynamicParam();
        ftsControl::rotationalMatrix(this->goal.target_pose.pose.position, this->stampedtf2Cell.transform.translation, this->yaw - M_PI / 2, 0.65);
        this->goal.target_pose.pose.position.z = 0;
        this->ac.sendGoal(this->goal);
        ac.waitForResult();
        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) ROS_INFO("Docking Reached goal");
        else ROS_INFO("Failed to reach goal");

        ftsControl::withTableDynamicParam();

        /* Once the FTS is under the Standardzelle, Lock the position FTS to the Standardzelle*/
        this->lockCell = true;
            
    }

    void ftsControl::carryCellToGoal(move_base_msgs::MoveBaseGoal dropLoc){
        ac.sendGoal(dropLoc);

        ros::Rate rate(30);
        while(ac.getState() !=  actionlib::SimpleClientGoalState::SUCCEEDED &&
                ac.getState() != actionlib::SimpleClientGoalState::ABORTED && ros::ok()){
            ros::spinOnce();
            rate.sleep();
        }
        
        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) ROS_INFO("Reached goal");
        else ROS_INFO("Failed to reach goal");
        this->lockCell = false;

        /*This is a ugly method of moving the FTS from under the standardzelle because of the following reason
        1.  We should not give new goal because scanner are not detecting the standardzelle's leg 
            if the robot rotate the it will collide with the standardzelle
        2.  In  the Navigation algorithm if FTS detect the obstacle in very close proximity then 
            the FTS will not move forward therefore back scanner can not be activated untile the FTS moved
            from under the standardzelle*/
        ros::Rate r(20);
        int couter = 0;
        geometry_msgs::Twist vel;
        vel.linear.x = 2.0;
        while(ros::ok() && couter <= 15){
            this->cmdVel_pub.publish(vel);
            r.sleep();
            couter ++;
            ros::spinOnce();
        }
        vel.linear.x = 0;
        this->cmdVel_pub.publish(vel);
    }

    void ftsControl::ftsStartCallback(const iwtros_msgs::ftsControl::ConstPtr& msg){
        this->endGoal.target_pose.header.frame_id = this->worldFrame.c_str();
        this->endGoal.target_pose.header.stamp = ros::Time::now();
        this->endGoal.target_pose.pose.position.x = msg->pose.position.x;
        this->endGoal.target_pose.pose.position.y = msg->pose.position.y;
        this->endGoal.target_pose.pose.position.z = msg->pose.position.z;
        this->endGoal.target_pose.pose.orientation.x = msg->pose.orientation.x;
        this->endGoal.target_pose.pose.orientation.y = msg->pose.orientation.y;
        this->endGoal.target_pose.pose.orientation.z = msg->pose.orientation.z;
        this->endGoal.target_pose.pose.orientation.w = msg->pose.orientation.w;

        this->chooseTable = msg->selRobot;
        if(this->chooseTable == 0) this->selCell = IIWA;
        else if(this->chooseTable == 1) this->selCell = UR5;
        else if(this->chooseTable == 2) this->selCell = PANDA;

        /*FTS Dockin to the standartdzelle*/
        ftsControl::goToTableLocation(this->selCell);
        ftsControl::reverseDocking();

        /*Disable the back laser scanner*/
        this->deatBackScaner.data = true;
        this->deactScan_pub.publish(this->deatBackScaner);

        /* Carry the Standardzelle to goal location */
        ftsControl::carryCellToGoal(this->endGoal);

        /*Unable the back laser scanner*/
        this->deatBackScaner.data = false;
        this->deactScan_pub.publish(this->deatBackScaner);

    }

    void ftsControl::quadToEuler(geometry_msgs::Quaternion& q, double& roll, double& pitch, double& yaw){
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

    void ftsControl::setDynamicParam(){
        system("rosrun dynamic_reconfigure dynparam set /move_base/DWAPlannerROS max_vel_x 0.0");
        system("rosrun dynamic_reconfigure dynparam set /move_base/DWAPlannerROS min_vel_x -0.7");
        system("rosrun dynamic_reconfigure dynparam set /move_base/DWAPlannerROS max_rot_vel 0.1");
        system("rosrun dynamic_reconfigure dynparam set /move_base/DWAPlannerROS min_rot_vel -0.1");
        system("rosrun dynamic_reconfigure dynparam set /move_base/local_costmap/inflation_layer inflation_radius 0.01");
        system("rosrun dynamic_reconfigure dynparam set /move_base/global_costmap/inflation_layer inflation_radius 0.01");
    }

    void ftsControl::withTableDynamicParam(){
        system("rosrun dynamic_reconfigure dynparam set /move_base/DWAPlannerROS max_vel_x 1.0");
        system("rosrun dynamic_reconfigure dynparam set /move_base/DWAPlannerROS min_vel_x 0.0");
        system("rosrun dynamic_reconfigure dynparam set /move_base/DWAPlannerROS max_rot_vel 0.8");
        system("rosrun dynamic_reconfigure dynparam set /move_base/DWAPlannerROS min_rot_vel 0.02");
        system("rosrun dynamic_reconfigure dynparam set /move_base/DWAPlannerROS xy_goal_tolerance 0.10");
        system("rosrun dynamic_reconfigure dynparam set /move_base/DWAPlannerROS yaw_goal_tolerance 0.05");
        system("rosrun dynamic_reconfigure dynparam set /move_base/local_costmap/inflation_layer inflation_radius 0.55");
        system("rosrun dynamic_reconfigure dynparam set /move_base/global_costmap/inflation_layer inflation_radius 0.55");
    }

    void ftsControl::resetDynamicParam(){

    }
}