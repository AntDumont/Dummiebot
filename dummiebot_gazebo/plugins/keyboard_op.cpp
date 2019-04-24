#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>

namespace gazebo{

  class KbOperator : public ModelPlugin{

  public:
    KbOperator() : ModelPlugin(){}

    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf){
      if (!ros::isInitialized())
      {
        ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
          << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
        return;
      }

      ROS_INFO("Hello World!");
    }
  };
  GZ_REGISTER_MODEL_PLUGIN(KbOperator)
}
