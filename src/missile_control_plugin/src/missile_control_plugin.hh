#ifndef __MissileControl__
#define __MissileControl__

#include <functional>
// Gazebo
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Assert.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/transport/transport.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/common/Profiler.hh>
#include <ignition/transport/Node.hh>

// ROS
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/Vector3.h>
#include <exocet_msgs/ExocetCMD.h>
// #include <nav_msgs/Odometry.h>

// Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

// Boost
#include <boost/thread.hpp>
#include <boost/bind.hpp>

#include <sdf/sdf.hh>
#include <array>
#include <mutex>





namespace gazebo
{
    /// \brief Control the movement of the Exocet Missile by appying thrust to 
    /// the missile's body or booster. It has the capability of detaching the booster
    ///
    /// The plugin requires the following parameters:
    /// <missile_body>      Link name of the missile's main body
    /// <booster>           Link name of the booster
    /// <booster_joint>     Name of the joint controlling the left aileron.

    class MissileControl : public ModelPlugin
    {
    
    public: 
        MissileControl();
        virtual ~MissileControl();
        virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
        bool FindJoint(const std::string &_sdfParam, sdf::ElementPtr _sdf, physics::JointPtr &_joint);
        bool FindLink(const std::string &_sdfParam, sdf::ElementPtr _sdf, physics::LinkPtr &_link);
        void OnUpdate();
        void OnExocetCommand();
        void applyThrust();
        void detachingBooster();

    private: 

        /// \brief Pointer to the update event connection.
        event::ConnectionPtr updateConnection;

        /// \brief keep track of controller update sim-time.
        private: gazebo::common::Time lastControllerUpdateTime;

        /// \brief Controller update mutex.
        boost::mutex mutex_;

        physics::ModelPtr model_;
        physics::LinkPtr booster_link_;
        physics::LinkPtr missile_body_link_;
        physics::JointPtr booster_joint_;

        

        bool booster_is_attached_;
        geometry_msgs::Vector3 thrust_force_;
        geometry_msgs::Vector3 thrust_torque_;

    };
}

#endif