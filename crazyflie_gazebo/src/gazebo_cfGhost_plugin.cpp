#include "gazebo_cfGhost_plugin.h"

#include "_version.h"

// #include <tf/transform_datatypes.h>
#define DEG_2_RAD (3.14159265359 / 180.0)

namespace gazebo {

GZ_REGISTER_MODEL_PLUGIN(GazeboCfGHostPlugin);

GazeboCfGHostPlugin::~GazeboCfGHostPlugin()
{
#if GAZEBO_9
#else
  event::Events::DisconnectWorldUpdateBegin(updateConnection_);
#endif
}

void
GazeboCfGHostPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  gzdbg << __FUNCTION__ << "() called." << std::endl;

  model_ = _model;

  world_ = model_->GetWorld();

  node_handle_ = transport::NodePtr(new transport::Node());
  node_handle_->Init();

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  updateConnection_ = event::Events::ConnectWorldUpdateBegin(
    boost::bind(&GazeboCfGHostPlugin::OnUpdate, this, _1));

  motor_velocity_reference_pub_ =
    node_handle_->Advertise<gz_mav_msgs::CommandMotorSpeed>(
      model_->GetName() + "/gazebo/command/motor_speed", 1);

  // Get parameter from launch file
  getSdfParam<std::string>(_sdf, "poseTopic", m_pose_topic, m_pose_topic);

  // Create ROS subscriber and ros handler
  ros::NodeHandle n;
  poseSubscriber = n.subscribe(
    m_pose_topic, 1, &GazeboCfGHostPlugin::poseReceivedCallback, this);
  model_->SetGravityMode(0);

#if GAZEBO_9
  // last_pose = P3(model_->WorldPose().Pos().X(),
  //                model_->WorldPose().Pos().Y(),
  //                model_->WorldPose().Pos().Z(),
  //                model_->WorldPose().Rot().W(),
  //                model_->WorldPose().Rot().X(),
  //                model_->WorldPose().Rot().Y(),
  //                model_->WorldPose().Rot().Z());
  last_pose = P3(model_->WorldPose());
#else
  // last_pose = P3(model_->GetWorldPose().pos.x,
  //                model_->GetWorldPose().pos.y,
  //                model_->GetWorldPose().pos.z,
  //                model_->GetWorldPose().rot.w,
  //                model_->GetWorldPose().rot.x,
  //                model_->GetWorldPose().rot.y,
  //                model_->GetWorldPose().rot.z);
  last_pose = P3(model_->GetWorldPose());
#endif
}

void
GazeboCfGHostPlugin::OnUpdate(const common::UpdateInfo& /*_info*/)
{
  model_->SetGravityMode(0);
  gz_mav_msgs::CommandMotorSpeed m_motor_speed;
  m_motor_speed.add_motor_speed(1000); // 0
  m_motor_speed.add_motor_speed(1000); // 1
  m_motor_speed.add_motor_speed(1000); // 2
  m_motor_speed.add_motor_speed(1000); // 3
  motor_velocity_reference_pub_->Publish(m_motor_speed);
  model_->SetWorldPose(last_pose);
}

void
GazeboCfGHostPlugin::poseReceivedCallback(
  const crazyflie_driver::GenericLogData::ConstPtr& position)
{

  /*last_pose=ignition::math::Pose3d(position->pose.position.x ,
     position->pose.position.y , position->pose.position.z,
          position->pose.orientation.w , position->pose.orientation.x ,
     position->pose.orientation.y , position->pose.orientation.z);*/
  if (position->values.size() == 3)
    last_pose = P3(position->values[0],
                   position->values[1],
                   position->values[2],
                   0,
                   0,
                   0);
  else if (position->values.size() == 6)
    last_pose = P3(position->values[0],
                   position->values[1],
                   position->values[2],
                   position->values[3] * DEG_2_RAD,
                   position->values[4] * DEG_2_RAD,
                   position->values[5] * DEG_2_RAD);
}
}