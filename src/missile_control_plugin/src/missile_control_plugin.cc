#include "missile_control_plugin.hh"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(MissileControl)

MissileControl::MissileControl()
{
    this->booster_is_attached_ = true;
    this->detaching_cmd = false;
    this->thrust_force_.Set(0,0,0);
    this->thrust_torque_.Set(0,0,0);
}
MissileControl::~MissileControl()
{

}

void MissileControl::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    GZ_ASSERT(_model, "MissileControl _model pointer is NULL");
    GZ_ASSERT(_sdf, "MissileControl _sdf pointer is NULL");

    this->model_ = _model;

    if (!this->FindLink("missile_body", _sdf, this->missile_body_link_, this->missile_body_link_name_))
        return;
    if (!this->FindJoint("booster_joint", _sdf, this->booster_joint_, this->booster_joint_name_))
        return;
    if (!this->FindLink("booster", _sdf, this->booster_link_, this->booster_link_name_))
        return;

    // Controller time control.
    this->lastControllerUpdateTime = this->model_->GetWorld()->SimTime();

    // Listen to the update event. This event is broadcast every simulation iteration.
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&MissileControl::OnUpdate, this, std::placeholders::_1));

    gzlog << "Missile ready to fly. The force will be with you" << std::endl;
    ROS_WARN("Loaded MissileControl Plugin with parent...%s", this->model_->GetName().c_str());

    // Initialize ros, if it has not already bee initialized.
    if (!ros::isInitialized())
    {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "exocet_missile_ctrl",
            ros::init_options::NoSigintHandler);
    }

    // // Create our ROS node. This acts in a similar manner to the Gazebo node
    this->rosNode.reset(new ros::NodeHandle("exocet_missile_ctrl"));


    // // Create a named topic, and subscribe to it.
    ros::SubscribeOptions so =
    ros::SubscribeOptions::create<exocet_msgs::ExocetCMD>(
        "ExocetCMD", 
        1,
        boost::bind(&MissileControl::OnExocetCommand, this, _1),
        ros::VoidPtr(), &this->rosQueue);
    this->rosSub = this->rosNode->subscribe(so);
    
    // // Spin up the queue helper thread.
    this->rosQueueThread = std::thread(std::bind(&MissileControl::QueueThread, this));

}

bool MissileControl::FindJoint(const std::string &_sdfParam, sdf::ElementPtr _sdf, physics::JointPtr &_joint, std::string &jointName)
{
    // Check if the required element is found
    if (!_sdf->HasElement(_sdfParam))
    {
        gzerr << "Unable to find the <" << _sdfParam << "> parameter." << std::endl;
        return false;
    }

    // Load the joint name
    jointName = _sdf->Get<std::string>(_sdfParam);
    _joint = this->model_->GetJoint(jointName);
    if (!_joint)
    {
        gzerr << "Failed to find joint [" << jointName << "] aborting plugin load." << std::endl;
        return false;
    }
    return true;    
}

bool MissileControl::FindLink(const std::string &_sdfParam, sdf::ElementPtr _sdf, physics::LinkPtr &_link, std::string &linkName)
{
    // Check if the required element is found
    if (!_sdf->HasElement(_sdfParam))
    {
        gzerr << "Unable to find the <" << _sdfParam << "> parameter." << std::endl;
        return false;
    }

    // Load the joint name
    linkName = _sdf->Get<std::string>(_sdfParam);
    _link = this->model_->GetLink(linkName);
    if (!_link)
    {
        gzerr << "Failed to find joint [" << linkName << "] aborting plugin load." << std::endl;
        return false;
    }
    return true;    
}

void MissileControl::OnUpdate(const common::UpdateInfo &_info)
{   
    this->applyThrust();
    if (this->booster_is_attached_ && this->detaching_cmd)
    {
        gazebo::common::Time curTime = this->model_->GetWorld()->SimTime();
        double seconds_since_last_update = ( curTime - lastControllerUpdateTime ).Double();

        if (seconds_since_last_update > 5)
        {
            ROS_WARN("Detaching Booster");
            this->detachingBooster();
        }
    }
}

void MissileControl::OnExocetCommand(const exocet_msgs::ExocetCMDConstPtr &_msg)
{
    double val = _msg->propulsion.force.z;
    this->thrust_force_.Set(0,0,val);
    this->detaching_cmd = _msg->detach_booster;
}

void MissileControl::QueueThread()
{
    static const double timeout = 0.01;
    while (this->rosNode->ok())
    {
        this->rosQueue.callAvailable(ros::WallDuration(timeout));
    }
}

void MissileControl::Reset()
{
    if (!this->booster_is_attached_)
    {
        this->booster_link_->Reset();
        this->booster_link_->ResetPhysicsStates();
        this->missile_body_link_->Reset();
        this->missile_body_link_->ResetPhysicsStates();
        this->booster_joint_->Reset();
        this->booster_joint_->Attach(this->missile_body_link_, this->booster_link_);
        this->booster_is_attached_ = true;
    }
    
}

void MissileControl::applyThrust()
{
    if(this->booster_is_attached_){
        this->booster_link_->AddRelativeForce(this->thrust_force_);
    } else
    {
        this->missile_body_link_->AddRelativeForce(this->thrust_force_);
    }
}

void MissileControl::detachingBooster()
{
    this->booster_joint_->Detach();
    this->missile_body_link_->RemoveChildJoint(this->booster_joint_name_);
    this->missile_body_link_->RemoveChild(this->booster_link_name_);
    // this->missile_body_link_->RemoveChild(booster_link_);
    this->booster_is_attached_ = false;
}