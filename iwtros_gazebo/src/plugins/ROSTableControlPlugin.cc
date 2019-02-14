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
}

void ROSTableControlPlugin::OnRosMsg_Pos(const geometry_msgs::TwistConstPtr &msg){
    this->MoveModel(msg->linear.x, msg->linear.y, msg->linear.z, msg->angular.x, msg->angular.y, msg->angular.z);
}

void ROSTableControlPlugin::MoveModel(float lin_x, float lin_y, float lin_z, float ang_x, float ang_y, float ang_z){
    std::string model_name = this->model->GetName();
    ROS_DEBUG("Moving table = %s", model_name.c_str());

    this->model->SetLinearVel(ignition::math::Vector3d(lin_x, lin_y, lin_z));
    this->model->SetAngularVel(ignition::math::Vector3d(ang_x, ang_y, ang_z));
    this->yawValue = ang_z;
    this->lin_X = lin_x;
    this->lin_Y = lin_y;
    math::Pose current_pose = this->model->GetWorldPose();
    current_pose.pos.z = 0;
    current_pose.rot.x = 0;
    current_pose.rot.y = 0;

    this->model->SetWorldPose(current_pose);

    /* get the current pose of the model again to send the tf2 broadcaster*/
    math::Pose current_pose2 = this->model->GetWorldPose();

    geometry_msgs::Transform crnt_pose2;
    crnt_pose2.translation.x = current_pose2.pos.x; 
    crnt_pose2.translation.y = current_pose2.pos.y;
    crnt_pose2.translation.z = current_pose2.pos.z;

    float yaw = (float)current_pose2.rot.GetYaw() + this->off_yaw;
    tf2::Quaternion q;
    q.setRPY(current_pose2.rot.GetRoll(), current_pose2.rot.GetPitch(), yaw);
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

    /*Dirty coding --------:(---------*/
    stampedTransforms.header.stamp = ros::Time::now();
    stampedTransforms.header.frame_id = "world";
    stampedTransforms.child_frame_id = this->table_base_link;
    if(this->yawValue != 0 && this->lin_Y == 0){
        stampedTransforms.transform.translation.x = this->prev_offsets.translation.x;
        stampedTransforms.transform.translation.y = this->prev_offsets.translation.y;
        stampedTransforms.transform.translation.z = this->prev_offsets.translation.z;
    }
    if(this->yawValue == 0  && this->lin_Y != 0 ){
        stampedTransforms.transform.translation.x = crnt_pose.translation.x + this->offsets.translation.x;
        stampedTransforms.transform.translation.y = crnt_pose.translation.y + this->offsets.translation.y;
        stampedTransforms.transform.translation.z = crnt_pose.translation.z + this->offsets.translation.z;
    }
    if(this->yawValue != 0 &&  this->lin_Y != 0){
        float r = sqrt((this->offsets.translation.x * this->offsets.translation.x) + (this->offsets.translation.y * this->offsets.translation.y));
        stampedTransforms.transform.translation.x = crnt_pose.translation.x + r * cos(this->yawValue);
        stampedTransforms.transform.translation.y = crnt_pose.translation.y - r * sin(this->yawValue);
        stampedTransforms.transform.translation.z = crnt_pose.translation.z + this->offsets.translation.z;
    }
    
    stampedTransforms.transform.rotation.x = crnt_pose.rotation.x;
    stampedTransforms.transform.rotation.y = crnt_pose.rotation.y;
    stampedTransforms.transform.rotation.z = crnt_pose.rotation.z;
    stampedTransforms.transform.rotation.w = crnt_pose.rotation.w;
    br.sendTransform(stampedTransforms);

    this->prev_offsets.translation.x = stampedTransforms.transform.translation.x;
    this->prev_offsets.translation.y = stampedTransforms.transform.translation.y;
    this->prev_offsets.translation.z = stampedTransforms.transform.translation.z;
}

void ROSTableControlPlugin::OnUpdate(){
}

void ROSTableControlPlugin::QueueTHread(){
    static const double timeout = 0.01;
    while(this->rosNode->ok()){
        this->rosQueue.callAvailable(ros::WallDuration(timeout));
    }
    
}

    