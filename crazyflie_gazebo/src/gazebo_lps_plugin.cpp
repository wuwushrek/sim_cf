#include "gazebo_lps_plugin.h"

// USER HEADERS
#include "ConnectGazeboToRosTopic.pb.h"

namespace gazebo {
	GZ_REGISTER_MODEL_PLUGIN(LpsPlugin)

	LpsPlugin::LpsPlugin() 
	: ModelPlugin()
	,lps_delay_(kDefaultLpsDelay)
	{ }

	LpsPlugin::~LpsPlugin()
	{
		updateConnection_->~Connection();
	}

	void LpsPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
	{
		model_ = _model;
		world_ = model_->GetWorld();

		if (_sdf->HasElement("lpsStdDev")){
			getSdfParam<double>(_sdf, "lpsStdDev", lps_std_dev_, 0);
			lps_noise_ = lps_std_dev_ > 0;
		} else {
			lps_noise_ = false;
		}
		namespace_.clear();
		if (_sdf->HasElement("robotNamespace")){
			namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
		} else {
			gzerr << "[gazebo_lps_plugin] Please specify a robotNamespace.\n";
		}

		  // Retrieve pressure rate
		int lps_rate = -1;
		getSdfParam<int>(_sdf,"rate",lps_rate , lps_rate);
		if (lps_rate > 0){
			lps_delay_ = 1.0 / lps_rate;
		}

		node_handle_ = transport::NodePtr(new transport::Node());
		node_handle_->Init(namespace_);

		// Listen to the update event. This event is broadcast every simulation iteration.
		updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&LpsPlugin::OnUpdate, this, _1));

		// Populate static part of the lps message
		lps_msg.mutable_header()->set_frame_id("lps_link");

#if GAZEBO_MAJOR_VERSION >= 9
		last_time_ = world_->SimTime();
#else
		last_time_ = world_->GetSimTime();
#endif
		pubs_and_subs_created_ = false;
	}

	void LpsPlugin::OnUpdate(const common::UpdateInfo&)
	{
		if (!pubs_and_subs_created_){
			CreatePubsAndSubs();
			pubs_and_subs_created_ =  true;
		}

#if GAZEBO_MAJOR_VERSION >= 9
		common::Time current_time = world_->SimTime();
#else
		common::Time current_time = world_->GetSimTime();
#endif
		double dt = (current_time - last_time_).Double();

		if (dt < lps_delay_)
			return;

#if GAZEBO_MAJOR_VERSION >= 9
		ignition::math::Pose3d T_W_I = model_->WorldPose();
#else
		ignition::math::Pose3d T_W_I = ignitionFromGazeboMath(model_->GetWorldPose());
#endif
		ignition::math::Vector3d& pos_W_I = T_W_I.Pos(); // Use the models' world position for LPS

		// update noise parameters if lps_noise_ is set
		if (lps_noise_){
			noise_lps_pos_.X() = lps_std_dev_ * randn_(rand_);
			noise_lps_pos_.Y() = lps_std_dev_ * randn_(rand_);
			noise_lps_pos_.Z() = lps_std_dev_ * randn_(rand_);
		} else {
			noise_lps_pos_.X() = 0.0;
			noise_lps_pos_.Y() = 0.0;
			noise_lps_pos_.Z() = 0.0;
		}

		auto pos_with_noise = pos_W_I + noise_lps_pos_;
		lps_msg.mutable_header()->mutable_stamp()->set_sec(current_time.sec);
		lps_msg.mutable_header()->mutable_stamp()->set_nsec(current_time.nsec);
		
		lps_msg.mutable_position()->set_x(pos_with_noise.X());
		lps_msg.mutable_position()->set_y(pos_with_noise.Y());
		lps_msg.mutable_position()->set_z(pos_with_noise.Z());

		lps_pub_->Publish(lps_msg);
		last_time_ =  current_time;
	}

	void LpsPlugin::CreatePubsAndSubs() {
  		// Create temporary "ConnectGazeboToRosTopic" publisher and message.
		// gazebo::transport::PublisherPtr connect_gazebo_to_ros_topic_pub =
		// node_handle_->Advertise<gz_std_msgs::ConnectGazeboToRosTopic>("~/" + kConnectGazeboToRosSubtopic, 1);

		lps_pub_ = node_handle_->Advertise<gz_geometry_msgs::Vector3dStamped>(model_->GetName() + "/gazebo/lps", 1);

		// gz_std_msgs::ConnectGazeboToRosTopic connect_gazebo_to_ros_topic_msg;
		// connect_gazebo_to_ros_topic_msg.set_gazebo_topic("~/"+ model_->GetName() + "/gazebo/lps");
		// connect_gazebo_to_ros_topic_msg.set_ros_topic(model_->GetName() + "/gazebo/lps");
		// connect_gazebo_to_ros_topic_msg.set_msgtype(gz_std_msgs::ConnectGazeboToRosTopic::VECTOR_3D_STAMPED);
		// connect_gazebo_to_ros_topic_pub->Publish(connect_gazebo_to_ros_topic_msg,true);
	}
}