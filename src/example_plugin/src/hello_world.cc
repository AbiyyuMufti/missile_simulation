#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>

namespace gazebo
{
  class WorldPluginTutorial : public WorldPlugin
  {
    public: WorldPluginTutorial() : WorldPlugin()
            {
              
            }

    public: void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
            {
                if(!ros::isInitialized())
                {
                    ROS_FATAL_STREAM("Load ROS node for Gazebo First");
                    return;
                }

                ROS_WARN("Hello World!!!!!!!!!!!!!!!!");
            }
  };
  GZ_REGISTER_WORLD_PLUGIN(WorldPluginTutorial)
}