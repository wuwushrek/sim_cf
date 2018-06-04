#include "gazebo_crtp_interface.h"

namespace gazebo {

GZ_REGISTER_MODEL_PLUGIN(GazeboCRTPInterface);

GazeboCRTPInterface::~GazeboCRTPInterface(){
	event::Events::DisconnectWorldUpdateBegin(updateConnection_);
	delete cfROS_;
}

void GazeboCRTPInterface::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf){
	gzdbg << __FUNCTION__ << "() called." << std::endl;

	model_ = _model;

	world_ = model_->GetWorld();

	namespace_.clear();
	if(_sdf->HasElement("robotNamespace")){
		namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
		gzdbg << "namespace_ = \"" << namespace_ << "\"." << std::endl;
	} else {
		gzerr << "[gazebo_crtp_interface] Please specify a robotNamespace.\n";
	}

	// Create and init gazebo node
	node_handle_ =  transport::NodePtr(new transport::Node());
	node_handle_->Init(namespace_);

	getSdfParam<std::string>(_sdf, "motorSpeedsPubTopic", motor_velocity_reference_pub_topic_,motor_velocity_reference_pub_topic_);
	gzdbg << "motorSpeedsPubTopic = \"" << motor_velocity_reference_pub_topic_ << "\"." << std::endl;

	getSdfParam<std::string>(_sdf, "imuSubTopic", imu_sub_topic_, imu_sub_topic_);
	gzdbg << "imuSubTopic = \"" << imu_sub_topic_ << "\"." << std::endl;

	getSdfParam<std::string>(_sdf, "magnetometerTopic", magnetic_field_sub_topic_, magnetic_field_sub_topic_);
	gzdbg << "magnetometerTopic = \"" << magnetic_field_sub_topic_ << "\"." << std::endl;

	getSdfParam<std::string>(_sdf, "pressureTopic", fluid_pressure_sub_topic_, fluid_pressure_sub_topic_);
	gzdbg << "pressureTopic = \"" << fluid_pressure_sub_topic_ << "\"." << std::endl;

	lps_sub_topic_ = kDefaultLpsTopic;
	gzdbg << "lpsTopic = \"" << lps_sub_topic_ << "\"." << std::endl;
	
	// Listen to the update event. This event is broadcast every
	// simulation iteration.
	updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboCRTPInterface::OnUpdate, this, _1));

	// gazebo publisher and subscriber setup
	gzdbg << "Creating Gazebo subscriber on topic \"" << "~/" + model_->GetName() + "/" + imu_sub_topic_ << "\"." << std::endl;
	imu_sub_ = node_handle_->Subscribe("~/" + model_->GetName() + "/" + imu_sub_topic_, &GazeboCRTPInterface::ImuCallback, this);

	gzdbg << "Creating Gazebo subscriber on topic \"" << "~/" + model_->GetName() + "/" + magnetic_field_sub_topic_ << "\"." << std::endl;
	magnetic_field_sub_ = node_handle_->Subscribe("~/" + model_->GetName() + "/" + magnetic_field_sub_topic_, &GazeboCRTPInterface::MagneticFieldCallback, this);

	gzdbg << "Creating Gazebo subscriber on topic \"" << "~/" + model_->GetName() + "/" + fluid_pressure_sub_topic_ << "\"." << std::endl;
	fluid_pressure_sub_ = node_handle_->Subscribe("~/" + model_->GetName() +  "/" + fluid_pressure_sub_topic_, &GazeboCRTPInterface::FluidPressureCallback, this);
	
	gzdbg << "Creating Gazebo subscriber on topic \"" << "~/" + model_->GetName() + "/" + lps_sub_topic_ << "\"." << std::endl;
	lps_sub_ = node_handle_->Subscribe("~/" + model_->GetName() + "/"+ lps_sub_topic_ , &GazeboCRTPInterface::LpsCallback, this);

	// Publish gazebo's motor_speed message
	gzdbg << "Creating Gazebo publisher on topic \"" << "~/" + model_->GetName() + "/" + motor_velocity_reference_pub_topic_ << "\"." << std::endl;
	motor_velocity_reference_pub_ = node_handle_->Advertise<gz_mav_msgs::CommandMotorSpeed>("~/" + model_->GetName() + "/" + motor_velocity_reference_pub_topic_, 1);

	// Initialize crazyflie interface
	getSdfParam<std::string>(_sdf, "crazyflieURI", crazyflie_uri_, crazyflie_uri_);

	std::string m_tf_prefix = namespace_+"/"+model_->GetName();

	bool enable_logging, enable_logging_imu, enable_logging_magnetic_field,enable_logging_temperature,enable_parameters;
	bool enable_logging_pressure, enable_logging_battery, enable_logging_packets, use_ros_time;

	getSdfParam<bool>(_sdf,"enableLogging",enable_logging , true);
	getSdfParam<bool>(_sdf,"enableParameters",enable_parameters , true);
	getSdfParam<bool>(_sdf,"useRosTime",use_ros_time , true);
	getSdfParam<bool>(_sdf,"enableLoggingImu",enable_logging_imu , true);
	getSdfParam<bool>(_sdf,"enableLoggingTemperature",enable_logging_temperature , true);
	getSdfParam<bool>(_sdf,"enableLoggingMagneticField",enable_logging_magnetic_field , true);
	getSdfParam<bool>(_sdf,"enableLoggingPressure",enable_logging_pressure , true);
	getSdfParam<bool>(_sdf,"enableLoggingBattery",enable_logging_battery , true);
	getSdfParam<bool>(_sdf,"enableLoggingPackets",enable_logging_packets , true);

	float roll_trim, pitch_trim;
	getSdfParam<float>(_sdf,"rollTrim",roll_trim , 0.0);
	getSdfParam<float>(_sdf,"pitchTrim",pitch_trim , 0.0);

	logBlock_ = std::vector<crazyflie_gazebo::LogBlock>();
	if (is_hitl_){
		cfROS_ = new CrazyflieROS(crazyflie_uri_,
			m_tf_prefix,
			roll_trim,
			pitch_trim,
			enable_logging,
			enable_parameters,
			logBlock_,
			use_ros_time,
			enable_logging_imu,
			enable_logging_temperature,
			enable_logging_magnetic_field,
			enable_logging_pressure,
			enable_logging_battery,
			enable_logging_packets);

		// Set motors callback
		std::function<void(const crtpMotorsDataResponse*)> cb_motors = std::bind(&GazeboCRTPInterface::onMotorsDataCallback, this, std::placeholders::_1);
		cfROS_->setOnMotorsData(cb_motors);
	}

	//cf_ = new Crazyflie(crazyflie_uri_, rosLogger);
}

void GazeboCRTPInterface::OnUpdate(const common::UpdateInfo& /*_info*/){
	if (new_motors_data_.load()){
		gz_mav_msgs::CommandMotorSpeed m_motor_speed;
		motors_mutex_.lock();
		m_motor_speed.add_motor_speed(m_motor_command_.m1); // 0
		m_motor_speed.add_motor_speed(m_motor_command_.m2); // 1
		m_motor_speed.add_motor_speed(m_motor_command_.m3); // 2
		m_motor_speed.add_motor_speed(m_motor_command_.m4); // 3
		motors_mutex_.unlock();
		motor_velocity_reference_pub_->Publish(m_motor_speed);
		new_motors_data_.store(false);
	}
}

void GazeboCRTPInterface::ImuCallback(ImuPtr& imu_msg){
	struct imu_s m_imu_info = {
		.header = crtp(CRTP_PORT_SETPOINT_SIM, 0),
		.type = SENSOR_GYRO_ACC_SIM,
		.acc = { static_cast<int16_t>(imu_msg->linear_acceleration().x()  	/ 	SENSORS_G_PER_LSB_CFG 		/ GRAVITY_MAGNITUDE_CF),
				 static_cast<int16_t>(imu_msg->linear_acceleration().y()  	/ 	SENSORS_G_PER_LSB_CFG 		/ GRAVITY_MAGNITUDE_CF),
				 static_cast<int16_t>(imu_msg->linear_acceleration().z()  	/ 	SENSORS_G_PER_LSB_CFG 		/ GRAVITY_MAGNITUDE_CF)},
		.gyro = {static_cast<int16_t>(imu_msg->angular_velocity().x()	   	/ 	SENSORS_DEG_PER_LSB_CFG	 	/ DEG_TO_RAD_CF),
				 static_cast<int16_t>(imu_msg->angular_velocity().y()    	/ 	SENSORS_DEG_PER_LSB_CFG 	/ DEG_TO_RAD_CF),
				 static_cast<int16_t>(imu_msg->angular_velocity().z()    	/	SENSORS_DEG_PER_LSB_CFG		/ DEG_TO_RAD_CF)}};

	// gzmsg << " " << m_imu_info.acc.x << " ; "<< m_imu_info.acc.y << " ; " << m_imu_info.acc.z << std::endl;
	cfROS_->sendSensorsPacket((const uint8_t*) &m_imu_info , sizeof(m_imu_info), ack_);
	// static int imu_msg_count = 0;
	// if(imu_msg_count >= 1) {
	// 	gzmsg << "{ "
	// 	", xacc: " << imu_msg->linear_acceleration().x() <<
	// 	", yacc: " << imu_msg->linear_acceleration().y() <<
	// 	", zacc: " << imu_msg->linear_acceleration().z() <<
	// 	", xgyro: " << imu_msg->angular_velocity().x() <<
	// 	", ygyro: " << imu_msg->angular_velocity().y() <<
	// 	", zgyro: " << imu_msg->angular_velocity().z() <<
	// 	" }" << std::endl;
	// 	imu_msg_count = 0;
	// }
	// imu_msg_count++;
}

void GazeboCRTPInterface::FluidPressureCallback(FluidPressurePtr& press_msg){
	double temperature_at_altitude_kelvin = kSeaLevelTempKelvin * exp(- log(press_msg->fluid_pressure() / kPressureOneAtmospherePascals)/kAirConstantDimensionless);
	double height_geopotential_m = (kSeaLevelTempKelvin - temperature_at_altitude_kelvin)/kTempLapseKelvinPerMeter;
	double height_geometric_m = height_geopotential_m * kEarthRadiusMeters / (kEarthRadiusMeters - height_geopotential_m);
	
	struct baro_s m_baro_info = {
		.header = crtp(CRTP_PORT_SETPOINT_SIM, 0),
		.type = SENSOR_BARO_SIM,
		.pressure = static_cast<float>(press_msg->fluid_pressure() * 0.01), // Convert in mbar
		.temperature = static_cast<float>(temperature_at_altitude_kelvin - 273.15),
		.asl = static_cast<float>(height_geometric_m)
	};
	cfROS_->sendSensorsPacket((const uint8_t*) &m_baro_info , sizeof(m_baro_info) , ack_);
	// static int press_msg_count = 0;
	// if(press_msg_count >= 100) {
	// 	gzmsg << "{ "
	// 	"time_usec: " << press_msg->header().stamp().sec() <<
	// 	", fluid pressure(Pa) : " << press_msg->fluid_pressure() <<
	// 	", height(m) : " << height_geometric_m <<
	// 	", temperature (K): " << temperature_at_altitude_kelvin <<
	// 	" }" << std::endl;
	// 	press_msg_count = 0;
	// }
	// press_msg_count++;
}

void GazeboCRTPInterface::MagneticFieldCallback(MagneticFieldPtr& mag_msg){
	// Need to be convert in gauss
	struct mag_s m_mag_info = {
		.header = crtp(CRTP_PORT_SETPOINT_SIM, 0),
		.type = SENSOR_MAG_SIM,
		.mag = { static_cast<int16_t>(mag_msg->magnetic_field().x()	  * 10000.0 *	MAG_GAUSS_PER_LSB),
				 static_cast<int16_t>(mag_msg->magnetic_field().y()   * 10000.0 *	MAG_GAUSS_PER_LSB),
				 static_cast<int16_t>(mag_msg->magnetic_field().z()   *	10000.0 *	MAG_GAUSS_PER_LSB)}
	};
	cfROS_->sendSensorsPacket((const uint8_t*) &m_mag_info , sizeof(m_mag_info) , ack_);
	/*static int mag_msg_count = 0;
	if(mag_msg_count >= 100) {
		gzmsg << "{ "
		"time_usec: " << mag_msg->header().stamp().sec() <<
		", xmag: " << mag_msg->magnetic_field().x() <<
		", ymag: " << mag_msg->magnetic_field().y() <<
		", zmag: " << mag_msg->magnetic_field().z() <<
		" }" << std::endl;
		mag_msg_count = 0;
	}
	mag_msg_count++;*/
}

void GazeboCRTPInterface::LpsCallback(LpsPtr& lps_msg){
	// Just need to transfer to the fcu the external position
	// gzdbg << "size of x : " << sizeof(lps_msg->position().x()) << std::endl;
	cfROS_->sendExternalPositionUpdate(
		static_cast<float>(lps_msg->position().x()),
		static_cast<float>(lps_msg->position().y()),
		static_cast<float>(lps_msg->position().z()));
}

void GazeboCRTPInterface::onMotorsDataCallback(const crtpMotorsDataResponse* motorsData){
	// static int motor_msg_count = 0;
	// if (motor_msg_count >= 100){
	// 	gzdbg << " m1 = " << PWM2OMEGA(motorsData->m1) << " , m2 = " << motorsData->m2 << " , m3 = " << motorsData->m3 << " , m4 = " << motorsData->m4 << std::endl;
	// 	motor_msg_count=0;
	// }
	// motor_msg_count++;
	motors_mutex_.lock();
	m_motor_command_.m1 = PWM2OMEGA(motorsData->m1);
	m_motor_command_.m2 = PWM2OMEGA(motorsData->m2);
	m_motor_command_.m3 = PWM2OMEGA(motorsData->m3);
	m_motor_command_.m4 = PWM2OMEGA(motorsData->m4);
	motors_mutex_.unlock();
	// gzdbg << " m1 = " << m_motor_command_.m1 << " , m2 = " << m_motor_command_.m2 << " , m3 = " << m_motor_command_.m3 << " , m4 = " << m_motor_command_.m4 << std::endl;
	new_motors_data_.store(true);
	// m_motor_speed.add_motor_speed(PWM2OMEGA(motorsData->m1)); // 0
	// m_motor_speed.add_motor_speed(PWM2OMEGA(motorsData->m3)); // 1
	// m_motor_speed.add_motor_speed(PWM2OMEGA(motorsData->m4)); // 2
	// m_motor_speed.add_motor_speed(PWM2OMEGA(motorsData->m2)); // 3
	// m_motor_speed.add_motor_speed(1000); // 0
	// m_motor_speed.add_motor_speed(1000); // 1
	// m_motor_speed.add_motor_speed(1000); // 2
	// m_motor_speed.add_motor_speed(1000); // 3
	// motor_velocity_reference_pub_->Publish(m_motor_speed);
}

}