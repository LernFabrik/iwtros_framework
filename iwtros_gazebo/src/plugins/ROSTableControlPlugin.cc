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
        ros::SubscribeOptions::create<geometry_msgs::PoseWithCovariance>(this->table_cmd_vel,
        1,  boost::bind(&ROSTableControlPlugin::OnRosMsg_Pos, this, _1),
        ros::VoidPtr(), &this->rosQueue);
    this->rosSub = this->rosNode->subscribe(so);
    //Spin up the queue helper thread
    this->rosQueueThread = std::thread(std::bind(&ROSTableControlPlugin::QueueTHread, this));

    ROS_WARN("Loaded Table move plugin with praent.... %s", this->model->GetName().c_str());
    this->current_time = ros::Time::now().toSec();
    this->prev_time = this->current_time;
}

void ROSTableControlPlugin::OnRosMsg_Pos(const geometry_msgs::PoseWithCovarianceConstPtr &msg){
    this->current_time = ros::Time::now().toSec(); //Begining of the loop
    this->MoveModel(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z, 
                    msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
}

void ROSTableControlPlugin::MoveModel(double lin_x, double lin_y, double lin_z, double ang_x, double ang_y, double ang_z, double ang_w){
    std::string model_name = this->model->GetName();
    ROS_DEBUG("Moving table = %s", model_name.c_str());
    math::Pose getPose1 = this->model->GetWorldPose();
    gzerr << model_name << " Pose 1 x = " << getPose1.pos.x << " y = " << getPose1.pos.y << " z = " << getPose1.pos.z << "\n";
    gzerr << model_name << " Orientation 1 x = " << getPose1.rot.x << " y = " << getPose1.rot.y << " z = " << getPose1.rot.z << " w = " << getPose1.rot.w << "\n";
    
    math::Pose setPose;
    setPose.pos.x = lin_x;
    setPose.pos.y = lin_y;
    setPose.pos.z = 0 + this->offsets.translation.z;

    /*Get the orientation either offseted orientation from the publisher 
    publishing the current FTS pose or offset here.
    1. Subscriber is subscribing the offseted pose change and*/
    setPose.rot.x = 0;
    setPose.rot.y = 0;
    setPose.rot.z = ang_z;
    setPose.rot.w = ang_w;
    this->model->SetWorldPose(setPose);
    
    math::Pose getPose2 = this->model->GetWorldPose();
    gzerr << model_name << " Pose x = " << getPose2.pos.x << " y = " << getPose2.pos.y << " z = " << getPose2.pos.z << "\n";
    gzerr << model_name << " Orientation x = " << getPose2.rot.x << " y = " << getPose2.rot.y << " z = " << getPose2.rot.z << " w = " << getPose2.rot.w << "\n";
      
    ROS_DEBUG("Moving Table = %s .... END", model_name.c_str());
}

void ROSTableControlPlugin::tfBroadCaster(geometry_msgs::Transform crnt_pose){
    static tf2_ros::StaticTransformBroadcaster br;
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

    