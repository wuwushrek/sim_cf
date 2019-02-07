/*
 * Copyright 2018 Eric Goubault, Cosynus, LIX, France
 * Copyright 2018 Sylve Putot, Cosynus, LIX, France
 * Copyright 2018 Franck Djeumou, Cosynus, LIX, France
 */

#include <iostream>
#include <sdf/sdf.hh>

#include <boost/bind.hpp>

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>

#include "_version.h"
#if GAZEBO_9
#include <ignition/math/Vector3.hh>
#else
#include <gazebo/math/gzmath.hh>
#endif

#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>

#include "ros/ros.h"

#include <crazyflie_driver/GenericLogData.h>

#include "common.h"

#include <CommandMotorSpeed.pb.h>

namespace gazebo {

static const std::string kDefaultPoseTopic = "/cf1/pos";

class GazeboCfGHostPlugin : public ModelPlugin
{
public:
  GazeboCfGHostPlugin()
    : ModelPlugin()
    , model_{}
    , world_(nullptr)
    , m_pose_topic(kDefaultPoseTopic)
  {}

  ~GazeboCfGHostPlugin();

protected:
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  void OnUpdate(const common::UpdateInfo& /*_info*/);

private:
#if GAZEBO_9
  typedef ignition::math::Pose3d P3;
#else
  typedef gazebo::math::Pose P3;
#endif

  std::string m_pose_topic;
  P3 last_pose;

  transport::NodePtr node_handle_;
  transport::PublisherPtr motor_velocity_reference_pub_;
  ros::Subscriber poseSubscriber;

  physics::ModelPtr model_;
  physics::WorldPtr world_;

  /// \brief Pointer to the update event connection.
  event::ConnectionPtr updateConnection_;

  void poseReceivedCallback(
    const crazyflie_driver::GenericLogData::ConstPtr& pos);
};
}