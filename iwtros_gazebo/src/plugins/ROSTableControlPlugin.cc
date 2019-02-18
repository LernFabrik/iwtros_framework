#include "iwtros_gazebo/plugins/ROSTableControlPlugin.hh"

#include <cstdlib>
#include <string>

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(ROSTableControlPlugin);

//////////////////////////////////////////////////////

ROSTableControlPlugin::ROSTableControlPlugin(){
}
ROSTableControlPlugin::~ROSTableControlPlugin(){
    this->rosNode->shutdown();
}
//////////////////////////////////////////////////////////

void ROSTableControlPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf){
    this->model = _parent;
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&ROSTableControlPlugin::OnUpdate, this));
    this->old_sec = ros::Time::now().toSec();
    
    if(_sdf->HasElement("table_cmd_vel"))
        this->table_cmd_vel = _sdf->Get<std::string>("table_cmd_vel");
    
     if(_sdf->HasElement("table_base_link"))
        this->table_base_link = _sdf->Get<std::string>("table_base_link");
    
    if(_sdf->HasElement("robot_namespace")){
        this->robotNamespace_ = _sdf->GetElement("robot_namespace")->Get<std::string>() + "/";
    }

    if(_sdf->HasElement("offset_x")){
        this->offsets.translation.x = (float)atof(_sdf->Get<std::string>("offset_x").c_str());
    }

    if(_sdf->HasElement("offset_y")){
        this->offsets.translation.y = (float)atof(_sdf->Get<std::string>("offset_y").c_str());
    }

    if(_sdf->HasElement("offset_z")){
        this->offsets.translation.z = (float)atof(_sdf->Get<std::string>("offset_z").c_str());
    }

    if(_sdf->HasElement("offset_yaw")){
        std::string yaw = _sdf->Get<std::string>("offset_yaw");
        this->off_yaw = (float)atof(yaw.c_str());
    }

    if(!ros::isInitialized){
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "tablemover_node", ros::init_options::NoSigintHandler);
    }

    //Create out ROS node. This acts in a similar manner to the gazebo node
    this->rosNode.reset(new ros::NodeHandle("tablemover_node"));
    //ROS Subcriber table pose
    ros::SubscribeOptions so = 
        ros::SubscribeOptions::create<geometry_msgs::Twist>(this->table_cmd_vel,
        1,  boost::bind(&ROSTableControlPlugin::OnRosMsg_Pos, this, _1),
        ros::VoidPtr(), &this->rosQueue);
    this->rosSub = this->rosNode->subscribe(so);
    //Spin up the queue helper thread
    this->rosQueueThread = std::thread(std::bind(&ROSTableControlPlugin::QueueTHread, this));

    ROS_WARN("Loaded Table move plugin with praent.... %s", this->model->GetName().c_str());
    this->current_time = ros::Time::now().toSec();
    this->prev_time = this->current_time;
}

void ROSTableControlPlugin::OnRosMsg_Pos(const geometry_msgs::TwistConstPtr &msg){
    this->current_time = ros::Time::now().toSec(); //Begining of the loop
    this->MoveModel(msg->linear.x, msg->linear.y, msg->linear.z, msg->angular.x, msg->angular.y, msg->angular.z);
}

void ROSTableControlPlugin::MoveModel(float lin_x, float lin_y, float lin_z, float ang_x, float ang_y, float ang_z){
    std::string model_name = this->model->GetName();
    ROS_DEBUG("Moving table = %s", model_name.c_str());

    /*--- Get the current pose of the model
    ------ models should not change in z axis, roll and pitch
    ------- we can add one more function to calculate the old pose in the programe 
    ------- and in current pose in the gazebo---------*/
    
    if(this->loop_counter == 0){
        math::Pose current_pose = this->model->GetRelativePose();
        current_pose.pos.z = 0;
        current_pose.rot.x = 0;
        current_pose.rot.y = 0;
        this->old_x = current_pose.pos.x;
        this->old_y = current_pose.pos.y;
        this->old_theta = current_pose.rot.GetYaw();
        gzerr << "old pose of the model x = " << this->old_x << " y = " << this->old_y << " theta = " << this->old_theta << "\n";

        this->model->SetWorldPose(current_pose);
        this->prev_time = this->current_time;
    } 
    this->dt = this->current_time - this->prev_time;
    gzerr << "dt = " << this->dt << "\n";
    if(this->dt > 0){
        this->delta_x = lin_x * this->dt;
        this->delta_y = lin_y * this->dt;
        this->delta_theta = ang_z * this->dt;
        gzerr << "delta pose of the model x = " << this->delta_x << " y = " << this->delta_y << " theta = " << this->delta_theta << "\n";

        this->detlta_d = sqrt((this->delta_x * this->delta_x) + (this->delta_y * this->delta_y));
        gzerr << "detla_d = " << this->detlta_d << "\n";
<<<<<<< HEAD
        this->crnt_x = this->old_x + this->delta_x;//this->detlta_d * sin(this->old_theta + this->delta_theta / 2);
        this->crnt_y = this->old_y + this->delta_y;//this->detlta_d * cos(this->old_theta + this->delta_theta / 2);
=======
        this->crnt_x = this->old_x + this->detlta_d * sin(this->old_theta + this->delta_theta / 2);
        this->crnt_y = this->old_y - this->detlta_d * cos(this->old_theta + this->delta_theta / 2);
>>>>>>> c0bf87e824a1b8f059592ec683b76ab2d1d5183f
        this->crnt_theta = this->old_theta + this->delta_theta;
        gzerr << "calculated current pose of the model x = " << this->crnt_x << " y = " << this->crnt_y << " theta = " << this->crnt_theta << "\n";
    }else{
        //this->model->SetLinearVel(ignition::math::Vector3d(lin_x, lin_y, lin_z));
        //this->model->SetAngularVel(ignition::math::Vector3d(ang_x, ang_y, ang_z));
        this->crnt_x = this->old_x;
        this->crnt_y = this->old_y;
        this->crnt_theta = this->old_theta;
    }
    math::Pose set_pose;
    set_pose.pos.x = this->crnt_x;
    set_pose.pos.y = this->crnt_y;
    set_pose.pos.z = 0;
    tf2::Quaternion calQuad;
    calQuad.setRPY(0, 0, crnt_theta);
    set_pose.rot.x = calQuad.x();                             // Hardcoding this value might not be good idea !!!!
    set_pose.rot.y = calQuad.y();
    set_pose.rot.z = calQuad.z();
    set_pose.rot.w = calQuad.w();
    gzerr << "setting current pose of the model x = " << set_pose.pos.x << " y = " << set_pose.pos.y << " theta = " << this->crnt_theta << "\n";
    this->model->SetWorldPose(set_pose);

    this->old_x = this->crnt_x;
    this->old_y = this->crnt_y;
    this->old_theta = this->crnt_theta;

    geometry_msgs::Transform crnt_pose2;
    crnt_pose2.translation.x = this->crnt_x; 
    crnt_pose2.translation.y = this->crnt_y;
    crnt_pose2.translation.z = 0;

    float yaw = this->crnt_theta + this->off_yaw;
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);                        // some of the roll and pitch are set to zero bacause the model motion is in 2D
    crnt_pose2.rotation.x = q.x();
    crnt_pose2.rotation.y = q.y();
    crnt_pose2.rotation.z = q.z();
    crnt_pose2.rotation.w = q.w();
    
    this->tfBroadCater(crnt_pose2);
                                            
    ROS_DEBUG("Moving Table = %s .... END", model_name.c_str());
}

void ROSTableControlPlugin::tfBroadCater(geometry_msgs::Transform crnt_pose){
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped stampedTransforms;

    stampedTransforms.header.stamp = ros::Time::now();
    stampedTransforms.header.frame_id = "world";
    stampedTransforms.child_frame_id = this->table_base_link;    
    stampedTransforms.transform.translation.x = crnt_pose.translation.x + this->offsets.translation.x;
    stampedTransforms.transform.translation.y = crnt_pose.translation.y + this->offsets.translation.y;
    stampedTransforms.transform.translation.z = crnt_pose.translation.z + this->offsets.translation.z;
    stampedTransforms.transform.rotation.x = crnt_pose.rotation.x;
    stampedTransforms.transform.rotation.y = crnt_pose.rotation.y;
    stampedTransforms.transform.rotation.z = crnt_pose.rotation.z;
    stampedTransforms.transform.rotation.w = crnt_pose.rotation.w;
    br.sendTransform(stampedTransforms);

    // End of the loop here
    this->loop_counter += 1;
    this->prev_time = this->current_time;
}

void ROSTableControlPlugin::OnUpdate(){
}

void ROSTableControlPlugin::QueueTHread(){
    static const double timeout = 0.01;
    while(this->rosNode->ok()){
        this->rosQueue.callAvailable(ros::WallDuration(timeout));
    }
    
}

    