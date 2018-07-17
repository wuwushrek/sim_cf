 /*
  * Copyright 2018 Eric Goubault, Cosynus, LIX, France
  * Copyright 2018 Sylve Putot, Cosynus, LIX, France
  * Copyright 2018 Franck Djeumou, Cosynus, LIX, France
  */

/**
 *@brief Local Positioning System (LPS) plugin
 *This plugin plublishes LPS data to be used and propagated
 */

#ifndef _GAZEBO_LPS_PLUGIN_HH_
#define _GAZEBO_LPS_PLUGIN_HH_

#include <random>

#include <sdf/sdf.hh>
#include <common.h>

#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/util/system.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math.hh>

#include <Vector3dStamped.pb.h>

namespace gazebo
{

static constexpr double kDefaultLpsDelay = 0;

class GAZEBO_VISIBLE LpsPlugin : public ModelPlugin
{
public:
	LpsPlugin();
	virtual ~LpsPlugin();

protected:
	virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
	virtual void OnUpdate(const common::UpdateInfo&);

private:
	void CreatePubsAndSubs();

	std::string namespace_;

	bool pubs_and_subs_created_;

	bool lps_noise_;
	double lps_std_dev_;

	physics::ModelPtr model_;
	physics::WorldPtr world_;
	event::ConnectionPtr updateConnection_;

	transport::NodePtr node_handle_;
	transport::PublisherPtr lps_pub_;

	gz_geometry_msgs::Vector3dStamped lps_msg;

	double lps_delay_;
	common::Time last_time_;

	// For the moment we just consider white noise
	ignition::math::Vector3d noise_lps_pos_;

	// lps noise parameter
	std::default_random_engine rand_;
	std::normal_distribution<float> randn_;

	static constexpr double lps_std_dev = 0.015; // meters --> DWM error datasheet divide by 3
};
}
#endif //_GAZEBO_LPS_PLUGIN_HH_