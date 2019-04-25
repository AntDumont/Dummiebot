#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/gazebo_client.hh>
#include "dummiebot_control/Dummie.h"
#include <ros/ros.h>

namespace gazebo{

  typedef const boost::shared_ptr<const dummiebot_control::Dummie> KbOperatorPtr;

  class KbOperator : public ModelPlugin{

  public:
    KbOperator(){}

    virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf){
      if (!ros::isInitialized())
      {
        ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
          << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
        return;
      }

      this->model = _model;
      //this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&KbOperator::OnUpdate, this));
      /*this->node = transport::NodePtr(new transport::Node());
      this->node->Init(this->model->GetWorld()->Name());*/

      //this->sub = this->node->Subscribe("dummie_move", &KbOperator::OnMsg, this);
      this->sub = this->node.subscribe("dummie_move", 1, &KbOperator::OnMsg, this);
    }

    private: void OnMsg(KbOperatorPtr &msg){

        if(msg->linear == 1){
          this->model->GetJoint("right_wheel_hinge")->SetVelocity(0, 10);
          this->model->GetJoint("left_wheel_hinge")->SetVelocity(0, 10);
        }
        else if(msg->linear == -1){
          this->model->GetJoint("right_wheel_hinge")->SetVelocity(0, -10);
          this->model->GetJoint("left_wheel_hinge")->SetVelocity(0, -10);
        }
        else if(msg->angular == 1){
          this->model->GetJoint("right_wheel_hinge")->SetVelocity(0, 10);
          this->model->GetJoint("left_wheel_hinge")->SetVelocity(0, 0);
        }
        else if(msg->angular == -1){
          this->model->GetJoint("right_wheel_hinge")->SetVelocity(0, 0);
          this->model->GetJoint("left_wheel_hinge")->SetVelocity(0, 10);
        }
        else if(msg->body_yaw != 0){
          this->model->GetJoint("chassis_body_joint")->SetVelocity(0, msg->body_yaw * 10);
        }
        else if(msg->body_pitch != 0){
          this->model->GetJoint("body_joint_body")->SetVelocity(0, msg->body_pitch * 10);
        }
    }


  private:
    physics::ModelPtr model;
    //transport::NodePtr node;
    ros::NodeHandle node;
    //transport::SubscriberPtr sub;
    ros::Subscriber sub;
  };
  GZ_REGISTER_MODEL_PLUGIN(KbOperator)


}
