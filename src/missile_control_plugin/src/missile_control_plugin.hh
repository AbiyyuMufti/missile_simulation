#ifndef __MissileControl__
#define __MissileControl__

#include <functional>
#include <mutex>
#include <thread>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/Vector3.h>
#include <exocet_msgs/ExocetCMD.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>


namespace gazebo
{
    /// \brief Control the movement of the Exocet Missile by appying thrust to 
    /// the missile's body or booster. It has the capability of detaching the booster
    ///
    /// The plugin requires the following parameters:
    /// <missile_body>      Link name of the missile's main body
    /// <booster>           Link name of the booster
    /// <booster_joint>     Name of the joint controlling the left aileron.

    class GZ_PLUGIN_VISIBLE MissileControl : public ModelPlugin
    {
    
    public: 
        MissileControl();
        virtual ~MissileControl();
        virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
        bool FindJoint(const std::string &_sdfParam, sdf::ElementPtr _sdf, physics::JointPtr &_joint, std::string &jointName);
        bool FindLink(const std::string &_sdfParam, sdf::ElementPtr _sdf, physics::LinkPtr &_link, std::string &linkName);
        void OnUpdate(const common::UpdateInfo &_info);
        void OnExocetCommand(const exocet_msgs::ExocetCMDConstPtr &_msg);
        void QueueThread();
        void Reset();
        void applyThrust();
        void detachingBooster();

    private: 

        /// \brief Pointer to the update event connection.
        event::ConnectionPtr updateConnection;
        
        /// \brief keep track of controller update sim-time.
        gazebo::common::Time lastControllerUpdateTime;

        /// \brief Controller update mutex.
        boost::mutex mutex_;
        
        physics::ModelPtr model_;
        physics::LinkPtr booster_link_;
        std::string booster_link_name_;
        physics::LinkPtr missile_body_link_;
        std::string missile_body_link_name_;
        physics::JointPtr booster_joint_;
        std::string booster_joint_name_;

        bool booster_is_attached_;
        bool detaching_cmd;
        ignition::math::Vector3d thrust_force_;
        ignition::math::Vector3d thrust_torque_;


        /// \brief A node use for ROS transport
        std::unique_ptr<ros::NodeHandle> rosNode;

        /// \brief A ROS subscriber
        ros::Subscriber rosSub;

        /// \brief A ROS callbackqueue that helps process messages
        ros::CallbackQueue rosQueue;

        /// \brief A thread the keeps running the rosQueue
        std::thread rosQueueThread;

    };
}

#endif