#ifndef __CRAZYFLIE_ROS_H__
#define __CRAZYFLIE_ROS_H__

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include "crazyflie_driver/AddCrazyflie.h"
#include "crazyflie_driver/GoTo.h"
#include "crazyflie_driver/Land.h"
#include "crazyflie_driver/RemoveCrazyflie.h"
#include "crazyflie_driver/SetGroupMask.h"
#include "crazyflie_driver/StartTrajectory.h"
#include "crazyflie_driver/Stop.h"
#include "crazyflie_driver/Takeoff.h"
#include "crazyflie_driver/UpdateParams.h"
#include "crazyflie_driver/UploadTrajectory.h"
#include "crazyflie_driver/sendPacket.h"

#include "crazyflie_driver/LogBlock.h"
#include "crazyflie_driver/GenericLogData.h"
#include "crazyflie_driver/FullState.h"
#include "crazyflie_driver/Hover.h"
#include "crazyflie_driver/Stop.h"
#include "crazyflie_driver/Position.h"
#include "crazyflie_driver/crtpPacket.h"

#include "crazyflie_cpp/Crazyradio.h"
#include "crazyflie_cpp/crtp.h"
#include "crazyflie_cpp/Crazyflie.h"

#include "std_srvs/Empty.h"
#include "std_msgs/Empty.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PointStamped.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/Temperature.h"
#include "sensor_msgs/MagneticField.h"
#include "std_msgs/Float32.h"

//#include <regex>
#include <thread>
#include <mutex>
#include <atomic>

#include <string>
#include <map>

class ROSLogger : public Logger
{
public:
  ROSLogger()
    : Logger()
  {
  }

  virtual ~ROSLogger() {}

  virtual void info(const std::string& msg)
  {
    ROS_INFO("%s", msg.c_str());
  }

  virtual void warning(const std::string& msg)
  {
    ROS_WARN("%s", msg.c_str());
  }

  virtual void error(const std::string& msg)
  {
    ROS_ERROR("%s", msg.c_str());
  }
};

static ROSLogger rosLogger;

class CrazyflieROS
{

public:

/*************** Additions/Modifications for Simulation ****************/
	CrazyflieROS(
		const std::string& cf_uri,
		const std::string& tf_prefix,
		bool enable_logging,
		bool enable_parameters,
		std::vector<crazyflie_driver::LogBlock>& log_blocks,
		bool use_ros_time,
		bool enable_logging_imu,
		bool enable_logging_temperature,
		bool enable_logging_magnetic_field,
		bool enable_logging_pressure,
		bool enable_logging_battery,
		bool enable_logging_packets,
		std::function<void(const uint8_t* , uint32_t)> sendDataFunc,
		std::function<void(ITransport::Ack &, int64_t)> recvDataFunc);

	CrazyflieROS(
		const std::string& cf_uri,
		const std::string& tf_prefix,
		float roll_trim,
		float pitch_trim,
		bool enable_logging,
		bool enable_parameters,
		std::vector<crazyflie_driver::LogBlock>& log_blocks,
		bool use_ros_time,
		bool enable_logging_imu,
		bool enable_logging_temperature,
		bool enable_logging_magnetic_field,
		bool enable_logging_pressure,
		bool enable_logging_battery,
		bool enable_logging_packets,
		bool enable_logging_pose = false,
		bool enable_logging_setpoint_pose = false);

	void setOnMotorsData(std::function<void(const crtpMotorsDataResponse*)> cb);

	void sendSensorsPacket(
		const uint8_t* data,
		uint32_t length);

	void sendExternalPositionUpdate(
		float x,
		float y,
		float z);

	/* Initialization process of ROS routines */
	void initalizeRosRoutines(ros::NodeHandle &n);

	/* Update steps --> Action to be done at every iteration */
	void updateInformation();

	/* Reset kalman filter when first gyro bias found is received */
	// void resetKalmanFilter();

/***********************************************************************/
	~CrazyflieROS();

	void stop();

	/**
	* Service callback which transmits a packet to the crazyflie
	* @param  req The service request, which contains a crtpPacket to transmit.
	* @param  res The service response, which is not used.
	* @return     returns true always
	*/
	bool sendPacket (
		crazyflie_driver::sendPacket::Request &req,
		crazyflie_driver::sendPacket::Response &res);

// private:
//	ros::ServiceServer m_sendPacketServer;
	/**
	* Publishes any generic packets en-queued by the crazyflie to a crtpPacket
	* topic.
	*/
//	void publishPackets();

private:
	struct logImu {
		float acc_x;
		float acc_y;
		float acc_z;
		float gyro_x;
		float gyro_y;
		float gyro_z;
	} __attribute__((packed));

/*************** Additions/Modifications for Simulation ****************/
	struct logLinkStats {
		uint16_t rxRate;
		uint16_t rxDrpRte;
		uint16_t txRate;
	} __attribute__((packed));

	struct logState {
		float x;
		float y;
		float z;
		float roll;
		float pitch;
		float yaw;
	} __attribute__((packed));
/***********************************************************************/
	struct log2 {
		float mag_x;
		float mag_y;
		float mag_z;
		float baro_temp;
		float baro_pressure;
		float pm_vbat;
	} __attribute__((packed));

private:

	bool emergency(
		std_srvs::Empty::Request& req,
		std_srvs::Empty::Response& res);

  template<class T, class U>
	void updateParam(uint8_t id, const std::string& ros_param)
	{
		U value;
		ros::param::get(ros_param, value);
		m_cf.setParam<T>(id , (T) value);
	} 

	void cmdHoverSetpoint(
		const crazyflie_driver::Hover::ConstPtr& msg);

	void cmdStop(
		const std_msgs::Empty::ConstPtr& msg);

	void cmdPositionSetpoint(
		const crazyflie_driver::Position::ConstPtr& msg);

	bool updateParams(
		crazyflie_driver::UpdateParams::Request& req,
		crazyflie_driver::UpdateParams::Response& res);

	void cmdVelChanged(
		const geometry_msgs::Twist::ConstPtr& msg);

	void cmdFullStateSetpoint(
		const crazyflie_driver::FullState::ConstPtr& msg);

	void positionMeasurementChanged(
		const geometry_msgs::PointStamped::ConstPtr& msg);

	void run();

	void onImuData(uint32_t time_in_ms, logImu* data);

	void onLog2Data(uint32_t time_in_ms, log2* data);

	void onLogCustom(uint32_t time_in_ms, std::vector<double>* values, void* userData);

/*************** Additions/Modifications for Simulation ****************/
	// void onLogLinkStatsData(uint32_t time_in_ms, logLinkStats* data);

	void onImuSimDataResponse(const crtpImuSimDataResponse* data);

	void onLogStateData(uint32_t time_in_ms, logState* data);

	void onLogSetpointStateData(uint32_t time_in_ms, logState* data);
/**********************************************************************/

	void onEmptyAck(const crtpPlatformRSSIAck* data);

	void onLinkQuality(float linkQuality);

	void onConsole(const char* msg) ;

	void onGenericPacket(const ITransport::Ack& ack);

	bool setGroupMask(
		crazyflie_driver::SetGroupMask::Request& req,
		crazyflie_driver::SetGroupMask::Response& res);

	bool takeoff(
		crazyflie_driver::Takeoff::Request& req,
		crazyflie_driver::Takeoff::Response& res);

	bool land(
		crazyflie_driver::Land::Request& req,
		crazyflie_driver::Land::Response& res);

	bool stop(
		crazyflie_driver::Stop::Request& req,
		crazyflie_driver::Stop::Response& res);

	bool goTo(
		crazyflie_driver::GoTo::Request& req,
		crazyflie_driver::GoTo::Response& res);

	bool uploadTrajectory(
		crazyflie_driver::UploadTrajectory::Request& req,
		crazyflie_driver::UploadTrajectory::Response& res);

	bool startTrajectory(
		crazyflie_driver::StartTrajectory::Request& req,
		crazyflie_driver::StartTrajectory::Response& res);

private:
	Crazyflie m_cf;
	std::string m_tf_prefix;
	bool m_isEmergency;
	float m_roll_trim;
	float m_pitch_trim;
	bool m_enableLogging;
	bool m_enableParameters;
	std::vector<crazyflie_driver::LogBlock> m_logBlocks;
	bool m_use_ros_time;
	bool m_enable_logging_imu;
	bool m_enable_logging_temperature;
	bool m_enable_logging_magnetic_field;
	bool m_enable_logging_pressure;
	bool m_enable_logging_battery;
	bool m_enable_logging_packets;

	ros::ServiceServer m_serviceEmergency;
	ros::ServiceServer m_serviceUpdateParams;
	ros::ServiceServer m_sendPacketServer;

  	// High-level setpoints
	ros::ServiceServer m_serviceSetGroupMask;
	ros::ServiceServer m_serviceTakeoff;
	ros::ServiceServer m_serviceLand;
	ros::ServiceServer m_serviceStop;
	ros::ServiceServer m_serviceGoTo;
	ros::ServiceServer m_serviceUploadTrajectory;
	ros::ServiceServer m_serviceStartTrajectory;

	ros::Subscriber m_subscribeCmdVel;
	ros::Subscriber m_subscribeCmdFullState;
	ros::Subscriber m_subscribeCmdHover;
	ros::Subscriber m_subscribeCmdStop;
	ros::Subscriber m_subscribeCmdPosition;
	ros::Subscriber m_subscribeExternalPosition;
	ros::Publisher m_pubImu;
	ros::Publisher m_pubTemp;
	ros::Publisher m_pubMag;
	ros::Publisher m_pubPressure;
	ros::Publisher m_pubBattery;
	ros::Publisher m_pubPackets;
	ros::Publisher m_pubRssi;

	ros::Publisher m_pubState;
	ros::Publisher m_pubSetpointState;

	std::vector<ros::Publisher> m_pubLogDataGeneric;

	bool m_sentSetpoint;
	bool m_sentExternalPosition;
	
/*************** Additions/Modifications for Simulation ****************/
	bool m_enable_logging_pose;
	bool m_enable_logging_setpoint_pose;

	std::unique_ptr<LogBlock<logImu> > logBlockImu;
	std::unique_ptr<LogBlock<log2> > logBlock2;
	// std::unique_ptr<LogBlock<logLinkStats> > logStatsData;
	std::unique_ptr<LogBlock<logState> > logStateData;
	std::unique_ptr<LogBlock<logState> > logSetpointStateData;
	std::vector<std::unique_ptr<LogBlockGeneric> > logBlocksGeneric;

	bool first_pos_sent;
	bool m_gyrobias_found;

	float x_init , y_init ,z_init;
/***********************************************************************/

	std::thread m_thread;
	ros::CallbackQueue m_callback_queue;
};

class CrazyflieServer
{

public:
	CrazyflieServer();
	~CrazyflieServer();
	void run();

private:

	bool add_crazyflie(
		crazyflie_driver::AddCrazyflie::Request  &req,
		crazyflie_driver::AddCrazyflie::Response &res);

	bool remove_crazyflie(
		crazyflie_driver::RemoveCrazyflie::Request  &req,
		crazyflie_driver::RemoveCrazyflie::Response &res);

private:
	std::map<std::string, CrazyflieROS*> m_crazyflies;
};

#endif //__CRAZYFLIE_ROS_H__