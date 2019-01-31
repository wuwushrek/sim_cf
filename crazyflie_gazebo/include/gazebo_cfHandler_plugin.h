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

#include "_version.h"
#if GAZEBO_9
#include <ignition/math/Vector3.hh>
#else
#include <gazebo/math/gzmath.hh>
#endif


#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#include "common.h"

/* Include constants variables store inside the pressure plugin */
#include "gazebo_pressure_plugin.h"

#include <CommandMotorSpeed.pb.h>
#include <Imu.pb.h>
#include <MagneticField.pb.h>
#include <FluidPressure.pb.h>
#include <Vector3dStamped.pb.h>

#include <crazyflie_ros/CrazyflieROS.h>
#include <crazyflie_ros/CrtpUtils.h>

#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>

/* Fast library for a reader writer queue */
#include "readerwriterqueue.h"

#define MAX_QUADS	6

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
	static const std::string kDefaultCfPrefix = "cf";
	static const std::string kDefaultURI = "INADDR_ANY://19950";
	static const int kDefaultCfNbQuads = 1;

	typedef struct _SensorsData {
		int index;
		uint8_t data[sizeof(struct imu_s)];
	} SensorsData;

	//static const std::vector<crazyflie_gazebo::LogBlock> kEmptyList = std::vector<crazyflie_gazebo::LogBlock>();
	class GazeboCfHandler : public ModelPlugin {
	public:
		GazeboCfHandler() :
			ModelPlugin(),
			namespace_(kDefaultNamespace),
			motor_velocity_reference_pub_topic_(kDefaultMotorVelocityReferencePubTopic),
			imu_sub_topic_(kDefaultImuTopic),
			magnetic_field_sub_topic_(kDefaultMagneticFieldTopic),
			fluid_pressure_sub_topic_(kDefaultFluidPressureTopic),
			lps_sub_topic_(kDefaultLpsTopic),
			cf_prefix(kDefaultCfPrefix),
			model_{},
			world_(nullptr),
			isPluginOn(true),
			nbQuads(kDefaultCfNbQuads),
			uri(kDefaultURI),
			is_hitl(false),
			first_index(1)
			{}
		~GazeboCfHandler();

	protected:
		void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
		void OnUpdate(const common::UpdateInfo& /*_info*/);

	private:
		int nbQuads;
		int first_index;
		std::string namespace_;

		std::string motor_velocity_reference_pub_topic_;
		std::string imu_sub_topic_;
		std::string magnetic_field_sub_topic_;
		std::string fluid_pressure_sub_topic_;
		std::string lps_sub_topic_;

		std::string uri;
		bool enable_logging;
		bool enable_logging_imu;
		bool enable_logging_magnetic_field;
		bool enable_logging_temperature;
		bool enable_parameters;
		bool enable_logging_pressure;
		bool enable_logging_battery;
		bool enable_logging_packets;
		bool use_ros_time;
		std::vector<crazyflie_driver::LogBlock> logBlocks;
		bool is_hitl;

		// ITransport::Ack ack_;
		// std::vector<crazyflie_gazebo::LogBlock> logBlock_;
		CrazyflieROS* cfROS_[MAX_QUADS];
		std::string cf_prefix;

		transport::NodePtr node_handle_;
		physics::ModelPtr model_;
		physics::WorldPtr world_;

		/// \brief Pointer to the update event connection.
		event::ConnectionPtr updateConnection_;

		void ImuCallback(ImuPtr& imu_msg , int index);
		void FluidPressureCallback(FluidPressurePtr& press_msg , int index);
		void MagneticFieldCallback(MagneticFieldPtr& mag_msgs , int index);
		void LpsCallback(LpsPtr& lps_msg , int index);

		// send and recv functions
		bool send(const uint8_t* data, uint32_t length, int index);
		void recv(ITransport::Ack &ack , int64_t timeout , int index);
		
		// Threads main function
		void handleMessage(uint8_t* data , int len, uint8_t index);
		void handleMotorsMessage(const uint8_t* data , uint8_t index);

		// server port and remote address for the crazyflies
		int port;
		int fd;
		struct sockaddr_in myaddr;

		struct sockaddr_in remaddr_rcv;
		socklen_t addrlen_rcv;

		struct sockaddr_in remaddr[MAX_QUADS];
		socklen_t addrlen[MAX_QUADS];
		bool socketInit[MAX_QUADS];

		void initializeCf(ros::NodeHandle &n);
		void initializeSubsAndPub();

		bool isInit[MAX_QUADS];
		bool isPluginOn;

		// Queue for exchanging messages between recv and ros/sender threads
		moodycamel::BlockingReaderWriterQueue<ITransport::Ack> m_queue[MAX_QUADS];
		moodycamel::BlockingReaderWriterQueue<SensorsData> m_queueSend;
		moodycamel::ReaderWriterQueue<int> cfToInitialize;
		moodycamel::ReaderWriterQueue<int> subPubToInitialize;
		
		// sensors subscribers
		transport::SubscriberPtr imu_sub_[MAX_QUADS];
		transport::SubscriberPtr magnetic_field_sub_[MAX_QUADS];
		transport::SubscriberPtr fluid_pressure_sub_[MAX_QUADS];
		transport::SubscriberPtr lps_sub_[MAX_QUADS];

		// mutex and messages for motors command
		gz_mav_msgs::CommandMotorSpeed m_motor_speed;
		std::mutex motors_mutex[MAX_QUADS];

		struct MotorsCommand {
			float m1;
			float m2;
			float m3;
			float m4;
		} m_motor_command_[MAX_QUADS];
		transport::PublisherPtr motor_velocity_reference_pub_[MAX_QUADS];
		void writeMotors();
		void onMotorsDataCallback(const crtpMotorsDataResponse* motorsData);

		// recv thread
		std::thread receiverThread;
		void recvThread();

		// LOg and Ros data thread
		std::thread cfThread;
		void cfROSThread();

		// Sensors data sender thread
		std::thread senderThread;
		void sendThread();

		// ros callback queue
		ros::CallbackQueue m_callback_queue;

		/* Callback specifications : Dirty trick */
		void ImuCallback_1(ImuPtr& imu_msg ){ImuCallback(imu_msg , 0);};
		void ImuCallback_2(ImuPtr& imu_msg ){ImuCallback(imu_msg , 1);};
		void ImuCallback_3(ImuPtr& imu_msg ){ImuCallback(imu_msg , 2);};
		void ImuCallback_4(ImuPtr& imu_msg ){ImuCallback(imu_msg , 3);};
		void ImuCallback_5(ImuPtr& imu_msg ){ImuCallback(imu_msg , 4);};
		void ImuCallback_6(ImuPtr& imu_msg ){ImuCallback(imu_msg , 5);};

		void FluidPressureCallback_1(FluidPressurePtr& press_msg){FluidPressureCallback(press_msg , 0);};
		void FluidPressureCallback_2(FluidPressurePtr& press_msg){FluidPressureCallback(press_msg , 1);};
		void FluidPressureCallback_3(FluidPressurePtr& press_msg){FluidPressureCallback(press_msg , 2);};
		void FluidPressureCallback_4(FluidPressurePtr& press_msg){FluidPressureCallback(press_msg , 3);};
		void FluidPressureCallback_5(FluidPressurePtr& press_msg){FluidPressureCallback(press_msg , 4);};
		void FluidPressureCallback_6(FluidPressurePtr& press_msg){FluidPressureCallback(press_msg , 5);};

		void MagneticFieldCallback_1(MagneticFieldPtr& mag_msgs){MagneticFieldCallback(mag_msgs , 0);};
		void MagneticFieldCallback_2(MagneticFieldPtr& mag_msgs){MagneticFieldCallback(mag_msgs , 1);};
		void MagneticFieldCallback_3(MagneticFieldPtr& mag_msgs){MagneticFieldCallback(mag_msgs , 2);};
		void MagneticFieldCallback_4(MagneticFieldPtr& mag_msgs){MagneticFieldCallback(mag_msgs , 3);};
		void MagneticFieldCallback_5(MagneticFieldPtr& mag_msgs){MagneticFieldCallback(mag_msgs , 4);};
		void MagneticFieldCallback_6(MagneticFieldPtr& mag_msgs){MagneticFieldCallback(mag_msgs , 5);};

		void LpsCallback_1(LpsPtr& lps_msg){LpsCallback(lps_msg , 0);};
		void LpsCallback_2(LpsPtr& lps_msg){LpsCallback(lps_msg , 1);};
		void LpsCallback_3(LpsPtr& lps_msg){LpsCallback(lps_msg , 2);};
		void LpsCallback_4(LpsPtr& lps_msg){LpsCallback(lps_msg , 3);};
		void LpsCallback_5(LpsPtr& lps_msg){LpsCallback(lps_msg , 4);};
		void LpsCallback_6(LpsPtr& lps_msg){LpsCallback(lps_msg , 5);};
	};
}