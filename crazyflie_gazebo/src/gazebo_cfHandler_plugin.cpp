#include "gazebo_cfHandler_plugin.h"
#include <future>

namespace gazebo {

GZ_REGISTER_MODEL_PLUGIN(GazeboCfHandler);

GazeboCfHandler::~GazeboCfHandler(){
	event::Events::DisconnectWorldUpdateBegin(updateConnection_);
	isPluginOn = false;
	for(uint8_t i=0 ; i<nbQuads ; i++){
		socketInit[i] = false;
		isInit[i] = false;
		delete cfROS_[i];
	}
	senderThread.join();
	receiverThread.join();
}

void GazeboCfHandler::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf){
	gzdbg << __FUNCTION__ << "() called." << std::endl;

	model_ = _model;

	world_ = model_->GetWorld();

	namespace_.clear();
	if(_sdf->HasElement("robotNamespace")){
		namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
		gzdbg << "namespace_ = \"" << namespace_ << "\"." << std::endl;
	} else {
		gzerr << "[gazebo_cfHandler_plugin] Please specify a robotNamespace.\n";
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
	getSdfParam<std::string>(_sdf, "lpsTopic", lps_sub_topic_, lps_sub_topic_);
	gzdbg << "lpsTopic = \"" << lps_sub_topic_ << "\"." << std::endl;
	
	// Listen to the update event. This event is broadcast every
	// simulation iteration.
	updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboCfHandler::OnUpdate, this, _1));

	/* Get Crazylfie ROS parameters */	
	getSdfParam<std::string>(_sdf, "cfPrefix", cf_prefix , cf_prefix);
	getSdfParam<bool>(_sdf,"enableLogging",enable_logging , true);
	getSdfParam<bool>(_sdf,"enableParameters",enable_parameters , true);
	getSdfParam<bool>(_sdf,"useRosTime",use_ros_time , true);
	getSdfParam<bool>(_sdf,"enableLoggingImu",enable_logging_imu , true);
	getSdfParam<bool>(_sdf,"enableLoggingTemperature",enable_logging_temperature , false);
	getSdfParam<bool>(_sdf,"enableLoggingMagneticField",enable_logging_magnetic_field , false);
	getSdfParam<bool>(_sdf,"enableLoggingPressure",enable_logging_pressure , false);
	getSdfParam<bool>(_sdf,"enableLoggingBattery",enable_logging_battery , false);
	getSdfParam<bool>(_sdf,"enableLoggingPackets",enable_logging_packets , false);

	getSdfParam<int>(_sdf,"udpPort", port , port);
	getSdfParam<int>(_sdf,"nbQuads", nbQuads , nbQuads);

	// open socket in UDP mode
	if ((fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
    	gzerr << "create socket failed " << std::endl;
    	return;
  	}
  	memset((char *)&myaddr, 0, sizeof(myaddr));
  	myaddr.sin_family = AF_INET;
	myaddr.sin_addr.s_addr = htonl(INADDR_ANY);
	myaddr.sin_port = htons(port);

	// Bind the socket
  	if (bind(fd, (struct sockaddr *)&myaddr, sizeof(myaddr)) < 0){
    	gzerr << "Bind socket failed " << std::endl;
    	return;
  	}
	
	cfToInitialize =  moodycamel::ReaderWriterQueue<int>(MAX_QUADS);
	subPubToInitialize = moodycamel::ReaderWriterQueue<int>(MAX_QUADS);
	m_queueSend = moodycamel::BlockingReaderWriterQueue<SensorsData>(6);
	addrlen_rcv = sizeof(remaddr_rcv);
	for (uint8_t i = 0 ; i< nbQuads ; i++){
		addrlen[i] = sizeof(remaddr[i]);
		isInit[i] = false;
		socketInit[i] = false;
		m_queue[i] = moodycamel::BlockingReaderWriterQueue<Crazyradio::Ack>(100);
		m_motor_command_[i].m1 = 0;
		m_motor_command_[i].m2 = 0;
		m_motor_command_[i].m3 = 0;
		m_motor_command_[i].m4 = 0;
	}

	// Launch the ros:sender task and the receiver task
	receiverThread = std::thread(&GazeboCfHandler::recvThread, this);
	cfThread = std::thread(&GazeboCfHandler::cfROSThread , this);
	senderThread = std::thread(&GazeboCfHandler::sendThread , this);
}

void GazeboCfHandler::OnUpdate(const common::UpdateInfo& /*_info*/){
	initializeSubsAndPub();
	writeMotors(); // Should reduce this frequency of motor updates
}

void GazeboCfHandler::ImuCallback(ImuPtr& imu_msg , int index){
	struct imu_s m_imu_info = {
		.header = crtp(CRTP_PORT_SETPOINT_SIM, 0),
		.type = SENSOR_GYRO_ACC_SIM,
		.acc = { static_cast<int16_t>(imu_msg->linear_acceleration().x()  	/ 	SENSORS_G_PER_LSB_CFG 		/ GRAVITY_MAGNITUDE_CF),
				 static_cast<int16_t>(imu_msg->linear_acceleration().y()  	/ 	SENSORS_G_PER_LSB_CFG 		/ GRAVITY_MAGNITUDE_CF),
				 static_cast<int16_t>(imu_msg->linear_acceleration().z()  	/ 	SENSORS_G_PER_LSB_CFG 		/ GRAVITY_MAGNITUDE_CF)},
		.gyro = {static_cast<int16_t>(imu_msg->angular_velocity().x()	   	/ 	SENSORS_DEG_PER_LSB_CFG	 	/ DEG_TO_RAD_CF),
				 static_cast<int16_t>(imu_msg->angular_velocity().y()    	/ 	SENSORS_DEG_PER_LSB_CFG 	/ DEG_TO_RAD_CF),
				 static_cast<int16_t>(imu_msg->angular_velocity().z()    	/	SENSORS_DEG_PER_LSB_CFG		/ DEG_TO_RAD_CF)}};
	SensorsData msg;
	msg.index = index;
	memcpy(msg.data , (const uint8_t*) &m_imu_info , sizeof(m_imu_info));
	// gzmsg << " " << m_imu_info.acc.x << " ; "<< m_imu_info.acc.y << " ; " << m_imu_info.acc.z << std::endl;
	//cfROS_[index]->sendSensorsPacket((const uint8_t*) &m_imu_info , sizeof(m_imu_info));
	m_queueSend.enqueue(msg);
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

void GazeboCfHandler::FluidPressureCallback(FluidPressurePtr& press_msg , int index){
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

	SensorsData msg;
	msg.index = index;
	memcpy(msg.data , (const uint8_t*) &m_baro_info , sizeof(m_baro_info));
	// // cfROS_[index]->sendSensorsPacket((const uint8_t*) &m_baro_info , sizeof(m_baro_info));
	m_queueSend.enqueue(msg);
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

void GazeboCfHandler::MagneticFieldCallback(MagneticFieldPtr& mag_msg , int index){
	// Need to be convert in gauss
	struct mag_s m_mag_info = {
		.header = crtp(CRTP_PORT_SETPOINT_SIM, 0),
		.type = SENSOR_MAG_SIM,
		.mag = { static_cast<int16_t>(mag_msg->magnetic_field().x()	  * 10000.0 *	MAG_GAUSS_PER_LSB),
				 static_cast<int16_t>(mag_msg->magnetic_field().y()   * 10000.0 *	MAG_GAUSS_PER_LSB),
				 static_cast<int16_t>(mag_msg->magnetic_field().z()   *	10000.0 *	MAG_GAUSS_PER_LSB)}
	};

	SensorsData msg;
	msg.index = index;
	memcpy(msg.data , (const uint8_t*) &m_mag_info , sizeof(m_mag_info));
	m_queueSend.enqueue(msg);
	//cfROS_[index]->sendSensorsPacket((const uint8_t*) &m_mag_info , sizeof(m_mag_info));
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

void GazeboCfHandler::LpsCallback(LpsPtr& lps_msg , int index){
	// Just need to transfer to the fcu the external position
	// gzdbg << "size of x : " << sizeof(lps_msg->position().x()) << std::endl;
	/*cfROS_[index]->sendExternalPositionUpdate(
		static_cast<float>(lps_msg->position().x()),
		static_cast<float>(lps_msg->position().y()),
		static_cast<float>(lps_msg->position().z()));*/
	struct pos_s pos = {
		.header = crtp(CRTP_PORT_SETPOINT_SIM, 0),
		.type = SENSOR_POS_SIM,
		.pos = {static_cast<float>(lps_msg->position().x()) , static_cast<float>(lps_msg->position().y()), static_cast<float>(lps_msg->position().z())}
	}; 
	SensorsData msg;
	msg.index = index;
	memcpy(msg.data , (const uint8_t*) &pos , sizeof(pos));
	m_queueSend.enqueue(msg);
}

bool GazeboCfHandler::send(const uint8_t* data , uint32_t length , int index)
{	
	// std::cout << "header : " << (int) data[0] << std::endl; 
	int transferred = sendto(fd, data, length, 0, (struct sockaddr *) &(remaddr[index]), addrlen[index]);
	if (transferred <= 0)
		throw std::runtime_error(strerror(errno));
    return true;
}

void GazeboCfHandler::recvThread()
{
	uint8_t buf[35];
	//auto start = std::chrono::system_clock::now();
	while(isPluginOn)
	{
		int len = recvfrom(fd, buf, sizeof(buf), 0, (struct sockaddr *) &remaddr_rcv, &addrlen_rcv);
		if (len <= 0 )
			continue;
		//auto end = std::chrono::system_clock::now();
    	//std::chrono::duration<double> elapsedSeconds = end-start;
		//ROS_INFO("Recv call ! %f" , elapsedSeconds.count());
		//start = end;
		uint8_t cf_id = buf[0]-1;
		// handleMessage(&buf[1] , len-1 , cf_id);
		if (!socketInit[cf_id] && buf[1] == 0xF3 && len == 2){
			remaddr[cf_id] = remaddr_rcv;
			addrlen[cf_id] = addrlen_rcv;
			socketInit[cf_id] = true;
			// Send response to SITL instance
			uint8_t data[2] = {(uint8_t)(cf_id+1) , 0xF3};
			send(data , sizeof(data) , cf_id);
			cfToInitialize.enqueue(cf_id);
		}
		else if (socketInit[cf_id] && (crtp(buf[1]) == crtp(0x09,0)) && (len == 18)) // Motor command message
			handleMotorsMessage(&buf[1] , cf_id);
		else if (socketInit[cf_id])
			handleMessage(&buf[1] , len-1 , cf_id);
	}
}

void GazeboCfHandler::cfROSThread()
{
	ros::NodeHandle n;
	n.setCallbackQueue(&m_callback_queue);

	while(isPluginOn){
		initializeCf(n);
		for(uint8_t i = 0 ; i< nbQuads ; i++){
			if (isInit[i])
				cfROS_[i]->updateInformation();
		}
		m_callback_queue.callAvailable(ros::WallDuration(0.0));
		std::this_thread::sleep_for(std::chrono::milliseconds(5));
	}

}

void GazeboCfHandler::sendThread()
{
	SensorsData msg;
	struct pos_s *m_pos;
	while(isPluginOn){
		m_queueSend.wait_dequeue(msg);
		if(msg.data[1] == SENSOR_GYRO_ACC_SIM)
			send(msg.data , sizeof(struct imu_s) , msg.index);
		else if(msg.data[1] == SENSOR_MAG_SIM)
			send(msg.data, sizeof(struct mag_s) , msg.index);
		else if(msg.data[1] == SENSOR_BARO_SIM)
			send(msg.data , sizeof(struct baro_s) , msg.index);
		else if (msg.data[1] == SENSOR_POS_SIM){
			m_pos = (struct pos_s *) msg.data;
			cfROS_[msg.index]->sendExternalPositionUpdate(m_pos->pos.x , m_pos->pos.y , m_pos->pos.z);
		}
	}
}

/* This should be launched in the gazebo thread */
void GazeboCfHandler::initializeSubsAndPub()
{
	int index;
	bool succeeded = subPubToInitialize.try_dequeue(index);
	if (! succeeded)
		return;

	// Handle sensors callback function
	if (index == 0){
		imu_sub_[index] = node_handle_->Subscribe(cf_prefix + std::to_string(index+1) + "/" + imu_sub_topic_, &GazeboCfHandler::ImuCallback_1 , this);
		magnetic_field_sub_[index] = node_handle_->Subscribe(cf_prefix + std::to_string(index+1) + "/" + magnetic_field_sub_topic_, &GazeboCfHandler::MagneticFieldCallback_1, this );
		fluid_pressure_sub_[index] = node_handle_->Subscribe(cf_prefix + std::to_string(index+1) + "/" + fluid_pressure_sub_topic_, &GazeboCfHandler::FluidPressureCallback_1, this );
		lps_sub_[index] = node_handle_->Subscribe(cf_prefix + std::to_string(index+1) + "/" + lps_sub_topic_, &GazeboCfHandler::LpsCallback_1, this );
	}else if(index == 1){
		imu_sub_[index] = node_handle_->Subscribe(cf_prefix + std::to_string(index+1) + "/" + imu_sub_topic_, &GazeboCfHandler::ImuCallback_2, this);
		magnetic_field_sub_[index] = node_handle_->Subscribe(cf_prefix + std::to_string(index+1) + "/" + magnetic_field_sub_topic_, &GazeboCfHandler::MagneticFieldCallback_2, this );
		fluid_pressure_sub_[index] = node_handle_->Subscribe(cf_prefix + std::to_string(index+1) + "/" + fluid_pressure_sub_topic_, &GazeboCfHandler::FluidPressureCallback_2, this );
		lps_sub_[index] = node_handle_->Subscribe(cf_prefix + std::to_string(index+1) + "/" + lps_sub_topic_, &GazeboCfHandler::LpsCallback_2, this );
	}else if(index == 2){
		imu_sub_[index] = node_handle_->Subscribe(cf_prefix + std::to_string(index+1) + "/" + imu_sub_topic_, &GazeboCfHandler::ImuCallback_3 , this);
		magnetic_field_sub_[index] = node_handle_->Subscribe(cf_prefix + std::to_string(index+1) + "/" + magnetic_field_sub_topic_, &GazeboCfHandler::MagneticFieldCallback_3, this );
		fluid_pressure_sub_[index] = node_handle_->Subscribe(cf_prefix + std::to_string(index+1) + "/" + fluid_pressure_sub_topic_, &GazeboCfHandler::FluidPressureCallback_3, this );
		lps_sub_[index] = node_handle_->Subscribe(cf_prefix + std::to_string(index+1) + "/" + lps_sub_topic_, &GazeboCfHandler::LpsCallback_3, this );
	}else if (index == 3){
		imu_sub_[index] = node_handle_->Subscribe(cf_prefix + std::to_string(index+1) + "/" + imu_sub_topic_, &GazeboCfHandler::ImuCallback_4, this);
		magnetic_field_sub_[index] = node_handle_->Subscribe(cf_prefix + std::to_string(index+1) + "/" + magnetic_field_sub_topic_, &GazeboCfHandler::MagneticFieldCallback_4, this );
		fluid_pressure_sub_[index] = node_handle_->Subscribe(cf_prefix + std::to_string(index+1) + "/" + fluid_pressure_sub_topic_, &GazeboCfHandler::FluidPressureCallback_4, this );
		lps_sub_[index] = node_handle_->Subscribe(cf_prefix + std::to_string(index+1) + "/" + lps_sub_topic_, &GazeboCfHandler::LpsCallback_4, this );
	} else if(index == 4){
		imu_sub_[index] = node_handle_->Subscribe(cf_prefix + std::to_string(index+1) + "/" + imu_sub_topic_, &GazeboCfHandler::ImuCallback_5 , this);
		magnetic_field_sub_[index] = node_handle_->Subscribe(cf_prefix + std::to_string(index+1) + "/" + magnetic_field_sub_topic_, &GazeboCfHandler::MagneticFieldCallback_5, this);
		fluid_pressure_sub_[index] = node_handle_->Subscribe(cf_prefix + std::to_string(index+1) + "/" + fluid_pressure_sub_topic_, &GazeboCfHandler::FluidPressureCallback_5, this);
		lps_sub_[index] = node_handle_->Subscribe(cf_prefix + std::to_string(index+1) + "/" + lps_sub_topic_, &GazeboCfHandler::LpsCallback_5, this );
	} else if(index == 5){
		imu_sub_[index] = node_handle_->Subscribe(cf_prefix + std::to_string(index+1) + "/" + imu_sub_topic_, &GazeboCfHandler::ImuCallback_6 , this);
		magnetic_field_sub_[index] = node_handle_->Subscribe(cf_prefix + std::to_string(index+1) + "/" + magnetic_field_sub_topic_, &GazeboCfHandler::MagneticFieldCallback_6, this);
		fluid_pressure_sub_[index] = node_handle_->Subscribe(cf_prefix + std::to_string(index+1) + "/" + fluid_pressure_sub_topic_, &GazeboCfHandler::FluidPressureCallback_6, this);
		lps_sub_[index] = node_handle_->Subscribe(cf_prefix + std::to_string(index+1) + "/" + lps_sub_topic_, &GazeboCfHandler::LpsCallback_6, this );
	}
	motor_velocity_reference_pub_[index] = node_handle_->Advertise<gz_mav_msgs::CommandMotorSpeed>(cf_prefix + std::to_string(index+1) + motor_velocity_reference_pub_topic_, 1);
	
	isInit[index] = true;
	gzmsg << "Init subs and Pubs done !" << std::endl;
}

void GazeboCfHandler::initializeCf(ros::NodeHandle &n)
{
	int index;
	bool succeeded = cfToInitialize.try_dequeue(index);
	if (!succeeded)
		return;

	//std::this_thread::sleep_for(std::chrono::milliseconds(3000));

	// create recv and send function instance
	std::function<void(const uint8_t* , uint32_t)> sendDataFunc = boost::bind(&GazeboCfHandler::send , this , _1, _2, index);
	std::function<void(Crazyradio::Ack& , int64_t)> recvDataFunc = boost::bind(&GazeboCfHandler::recv , this , _1 ,_2, index);

	cfROS_[index] = new CrazyflieROS(
			cf_prefix + std::to_string(index+1),
			enable_logging,
			enable_parameters,
			use_ros_time,
			enable_logging_imu,
			enable_logging_temperature,
			enable_logging_magnetic_field,
			enable_logging_pressure,
			enable_logging_battery,
			enable_logging_packets,
			sendDataFunc,
			recvDataFunc);
	cfROS_[index]->initalizeRosRoutines(n);
	// Enqueue the index after intialization to wake up the gazebo main thread
	subPubToInitialize.enqueue(index);
}

void GazeboCfHandler::handleMotorsMessage(const uint8_t* data , uint8_t index)
{
	crtpMotorsDataResponse* motorsData = (crtpMotorsDataResponse *) data;
	{
		std::unique_lock<std::mutex> mlock(motors_mutex[index]);
		m_motor_command_[index].m1 = PWM2OMEGA(motorsData->m1);
		m_motor_command_[index].m2 = PWM2OMEGA(motorsData->m2);
		m_motor_command_[index].m3 = PWM2OMEGA(motorsData->m3);
		m_motor_command_[index].m4 = PWM2OMEGA(motorsData->m4);
	}

}

void GazeboCfHandler::handleMessage(uint8_t* data , int len, uint8_t index)
{
	Crazyradio::Ack ack;
	ack.ack = true;
	// Need to find a better work around in order to not copy twice the crtp data
	memcpy(&ack.data[0] , data , sizeof(uint8_t) * len);
	ack.size = len;
	bool succeeded  = m_queue[index].enqueue(ack);
	/*if (!succeeded)
		std::cout << "[" << cf_prefix << (index+1) << "] Queue full ! " << std::endl;*/
}

void GazeboCfHandler::recv(Crazyradio::Ack &ack , int64_t timeout_us , int index)
{
	// Poll the message and store it in ack
	bool succeeded = m_queue[index].wait_dequeue_timed(ack , timeout_us);
	ack.ack = succeeded;
}

void GazeboCfHandler::writeMotors()
{
	for (uint8_t i=0 ; i< nbQuads ; i++){
		if (!isInit[i])
			continue;
		m_motor_speed.clear_motor_speed();
		motors_mutex[i].lock();
		m_motor_speed.add_motor_speed(m_motor_command_[i].m1); // 0
		m_motor_speed.add_motor_speed(m_motor_command_[i].m2); // 1
		m_motor_speed.add_motor_speed(m_motor_command_[i].m3); // 2
		m_motor_speed.add_motor_speed(m_motor_command_[i].m4); // 3
		motors_mutex[i].unlock();
		motor_velocity_reference_pub_[i]->Publish(m_motor_speed);
	}
}

}