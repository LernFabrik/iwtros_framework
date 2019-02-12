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
    
    if(_sdf->HasElement("robot_namespace")){
        this->robotNamespace_ = _sdf->GetElement("robot_namespace")->Get<std::string>() + "/";
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
        1,  
        boost::bind(&ROSTableControlPlugin::OnRosMsg_Pos, this, _1),
        ros::VoidPtr(), &this->rosQueue);
    this->rosSub = this->rosNode->subscribe(so);
    //Spin up the queue helper thread
    this->rosQueueThread = std::thread(std::bind(&ROSTableControlPlugin::QueueTHread, this));

    ROS_WARN("Loaded Table move plugin with praent.... %s", this->model->GetName().c_str());
}

void ROSTableControlPlugin::OnRosMsg_Pos(const geometry_msgs::TwistConstPtr &msg){
    this->MoveModel(msg->linear.x, msg->linear.y, msg->linear.z, msg->angular.x, msg->angular.y, msg->angular.z);
}

void ROSTableControlPlugin::MoveModel(float lin_x, float lin_y, float lin_z, float ang_x, float ang_y, float ang_z){
    std::string model_name = this->model->GetName();
    ROS_DEBUG("Moving table = %s", model_name.c_str());

    this->model->SetLinearVel(ignition::math::Vector3d(lin_x, lin_y, lin_z));
    this->model->SetAngularVel(ignition::math::Vector3d(ang_x, ang_y, ang_z));
    math::Pose current_pose = this->model->GetWorldPose();
    current_pose.pos.z = 0;
    current_pose.rot.x = 0;
    current_pose.rot.y = 0;

    this->model->SetWorldPose(current_pose);
    
    ROS_DEBUG("Moving Table = %s .... END", model_name.c_str());
}

void ROSTableControlPlugin::OnUpdate(){
}

void ROSTableControlPlugin::QueueTHread(){
    static const double timeout = 0.01;
    while(this->rosNode->ok()){
        this->rosQueue.callAvailable(ros::WallDuration(timeout));
    }
    
}

    