 /*
  * Copyright 2018 Eric Goubault, Cosynus, LIX, France
  * Copyright 2018 Sylve Putot, Cosynus, LIX, France
  * Copyright 2018 Franck Djeumou, Cosynus, LIX, France
  */

#include <iostream>
#include <mutex>
#include <atomic>
#include <math.h>
#include <deque>
#include <stdio.h>
#include <sdf/sdf.hh>

#include <boost/bind.hpp>
#include <Eigen/Eigen>

#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/math/Vector3.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#include "common.h"

#include "gazebo_pressure_plugin.h" // TODO include mannually constants

#include <CommandMotorSpeed.pb.h>
#include <Imu.pb.h>
#include <MagneticField.pb.h>
#include <FluidPressure.pb.h>
#include <Vector3dStamped.pb.h>
// #include <NavSatFix.pb.h>

#include <crazyflie_ros/CrazyflieROS.h>
#include <crazyflie_ros/CrtpUtils.h>

#define PI_CF 3.14159265359
#define DEG_TO_RAD_CF (PI_CF/180.0)
#define RAD_TO_DEG_CF (180.0/PI_CF)

#define GRAVITY_MAGNITUDE_CF (9.81) // we use the magnitude such that the sign/direction is explicit in calculations

#define PWM2OMEGA(pwm) ((pwm) < (1000) ? (0) : ((0.04076521f*pwm) + 380.8359f)) 

namespace gazebo {

	typedef const boost::shared_ptr<const gz_mav_msgs::CommandMotorSpeed> CommandMotorSpeedPtr;
	typedef const boost::shared_ptr<const gz_sensor_msgs::Imu> ImuPtr;
	typedef const boost::shared_ptr<const gz_sensor_msgs::MagneticField> MagneticFieldPtr;
	typedef const boost::shared_ptr<const gz_sensor_msgs::FluidPressure> FluidPressurePtr;
	typedef const boost::shared_ptr<const gz_geometry_msgs::Vector3dStamped> LpsPtr;

	static const std::string kDefaultMotorVelocityReferencePubTopic = "/gazebo/command/motor_speed";
	static const std::string kDefaultImuTopic = "gazebo/imu";
	static const std::string kDefaultNamespace = "";
	static const std::string kDefaultMagneticFieldTopic = "gazebo/magnetic_field";
	static const std::string kDefaultFluidPressureTopic = "gazebo/air_pressure";
	static const std::string kDefaultLpsTopic = "gazebo/lps";

	static const std::string kDefaultCrazyflieUriLink = "usb://0";

	//static const std::vector<crazyflie_gazebo::LogBlock> kEmptyList = std::vector<crazyflie_gazebo::LogBlock>();
	class GazeboCRTPInterface : public ModelPlugin {
	public:
		GazeboCRTPInterface() :
			ModelPlugin(),
			namespace_(kDefaultNamespace),
			motor_velocity_reference_pub_topic_(kDefaultMotorVelocityReferencePubTopic),
			imu_sub_topic_(kDefaultImuTopic),
			magnetic_field_sub_topic_(kDefaultMagneticFieldTopic),
			fluid_pressure_sub_topic_(kDefaultFluidPressureTopic),
			lps_sub_topic_(kDefaultLpsTopic),
			model_{},
			world_(nullptr),
			crazyflie_uri_(kDefaultCrazyflieUriLink),
			is_hitl_(true),
			cfROS_(nullptr),
			new_motors_data_(false)
			{}
		~GazeboCRTPInterface();

	protected:
		void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
		void OnUpdate(const common::UpdateInfo& /*_info*/);

	private:
		std::string namespace_;
		std::string motor_velocity_reference_pub_topic_;

		std::string imu_sub_topic_;
		std::string magnetic_field_sub_topic_;
		std::string fluid_pressure_sub_topic_;
		std::string lps_sub_topic_;

		std::string crazyflie_uri_;
		bool is_hitl_;
		std::vector<crazyflie_gazebo::LogBlock> logBlock_;
		Crazyradio::Ack ack_;
		CrazyflieROS *cfROS_;

		transport::NodePtr node_handle_;
		transport::PublisherPtr motor_velocity_reference_pub_;

		transport::SubscriberPtr mav_control_sub_;
		transport::SubscriberPtr imu_sub_;
		transport::SubscriberPtr magnetic_field_sub_;
		transport::SubscriberPtr fluid_pressure_sub_;
		transport::SubscriberPtr lps_sub_;

		physics::ModelPtr model_;
		physics::WorldPtr world_;

		std::mutex motors_mutex_;
		std::atomic<bool> new_motors_data_;
		struct MotorsCommand {
			float m1;
			float m2;
			float m3;
			float m4;
		} m_motor_command_;


		/// \brief Pointer to the update event connection.
		event::ConnectionPtr updateConnection_;

		void ImuCallback(ImuPtr& imu_msg);
		void FluidPressureCallback(FluidPressurePtr& press_msg);
		void MagneticFieldCallback(MagneticFieldPtr& mag_msgs);
		void LpsCallback(LpsPtr& lps_msg);

		void onMotorsDataCallback(const crtpMotorsDataResponse* motorsData);
	};
}