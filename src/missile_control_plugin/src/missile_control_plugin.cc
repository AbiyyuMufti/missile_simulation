#include "missile_control_plugin.hh"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(MissileControl)

MissileControl::MissileControl()
{
    this->booster_is_attached_ = false;
    this->thrust_force_.x = 0.0;
    this->thrust_force_.y = 0.0;
    this->thrust_force_.z = 0.0;

    this->thrust_torque_.x = 0.0;
    this->thrust_torque_.y = 0.0;
    this->thrust_torque_.z = 0.0;    
}
MissileControl::~MissileControl()
{

}

void MissileControl::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    GZ_ASSERT(_model, "MissileControl _model pointer is NULL");
    GZ_ASSERT(_sdf, "MissileControl _sdf pointer is NULL");

    this->model_ = _model;

    if (!this->FindLink("missile_body", _sdf, this->missile_body_link_))
        return;
    if (!this->FindJoint("booster_joint", _sdf, this->booster_joint_))
        return;
    if (!this->FindLink("booster", _sdf, this->booster_link_))
        return;

    // Listen to the update event. This event is broadcast every simulation iteration.
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&MissileControl::OnUpdate, this));

}

bool MissileControl::FindJoint(const std::string &_sdfParam, sdf::ElementPtr _sdf, physics::JointPtr &_joint)
{
    // Check if the required element is found
    if (!_sdf->HasElement(_sdfParam))
    {
        gzerr << "Unable to find the <" << _sdfParam << "> parameter." << std::endl;
        return false;
    }

    // Load the joint name
    std::string jointName = _sdf->Get<std::string>(_sdfParam);
    _joint = this->model_->GetJoint(jointName);
    if (!_joint)
    {
        gzerr << "Failed to find joint [" << jointName << "] aborting plugin load." << std::endl;
        return false;
    }
    return true;    
}

bool MissileControl::FindLink(const std::string &_sdfParam, sdf::ElementPtr _sdf, physics::LinkPtr &_link)
{
    // Check if the required element is found
    if (!_sdf->HasElement(_sdfParam))
    {
        gzerr << "Unable to find the <" << _sdfParam << "> parameter." << std::endl;
        return false;
    }

    // Load the joint name
    std::string linkName = _sdf->Get<std::string>(_sdfParam);
    _link = this->model_->GetLink(linkName);
    if (!_link)
    {
        gzerr << "Failed to find joint [" << linkName << "] aborting plugin load." << std::endl;
        return false;
    }
    return true;    
}

void MissileControl::OnUpdate()
{

}

void MissileControl::OnExocetCommand()
{

}

void MissileControl::applyThrust()
{

}

void MissileControl::detachingBooster()
{

}