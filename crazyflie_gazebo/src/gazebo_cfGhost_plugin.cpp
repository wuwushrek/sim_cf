#include "gazebo_cfGhost_plugin.h"

// #include <tf/transform_datatypes.h>

namespace gazebo {

GZ_REGISTER_MODEL_PLUGIN(GazeboCfGHostPlugin);

GazeboCfGHostPlugin::~GazeboCfGHostPlugin()
{
	event::Events::DisconnectWorldUpdateBegin(updateConnection_);
}

void GazeboCfGHostPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
	gzdbg << __FUNCTION__ << "() called." << std::endl;

	model_ = _model;

	world_ = model_->GetWorld();

	node_handle_ =  transport::NodePtr(new transport::Node());
	node_handle_->Init();

	// Listen to the update event. This event is broadcast every
	// simulation iteration.
	updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboCfGHostPlugin::OnUpdate, this, _1));

	motor_velocity_reference_pub_ = node_handle_->Advertise<gz_mav_msgs::CommandMotorSpeed>(model_->GetName() + "/gazebo/command/motor_speed", 1);

	// Get parameter from launch file
	getSdfParam<std::string>(_sdf,"poseTopic",m_pose_topic , m_pose_topic);

	// Create ROS subscriber and ros handler
	ros::NodeHandle n;
	poseSubscriber = n.subscribe(m_pose_topic, 1, &GazeboCfGHostPlugin::poseReceivedCallback, this);


}

void GazeboCfGHostPlugin::OnUpdate(const common::UpdateInfo& /*_info*/)
{
	gz_mav_msgs::CommandMotorSpeed m_motor_speed;
	m_motor_speed.add_motor_speed(1000); // 0
	m_motor_speed.add_motor_speed(1000); // 1
	m_motor_speed.add_motor_speed(1000); // 2
	m_motor_speed.add_motor_speed(1000); // 3
	motor_velocity_reference_pub_->Publish(m_motor_speed);
}

void GazeboCfGHostPlugin::poseReceivedCallback(const geometry_msgs::PoseStamped::ConstPtr& position)
{

	model_->SetWorldPose(ignition::math::Pose3d(position->pose.position.x , position->pose.position.y , position->pose.position.z, 
		position->pose.orientation.w , position->pose.orientation.x , position->pose.orientation.y , position->pose.orientation.z));
}

}