#include "crazyflie_ros/CrazyflieROS.h"
#include <tf/transform_datatypes.h>

constexpr double pi() { return 3.141592653589793238462643383279502884; }

double degToRad(double deg) {
  return deg / 180.0 * pi();
}

double radToDeg(double rad) {
  return rad * 180.0 / pi();
}

CrazyflieROS::CrazyflieROS(
  const std::string& cf_uri,
  const std::string& tf_prefix,
  float roll_trim,
  float pitch_trim,
  bool enable_logging,
  bool enable_parameters,
  std::vector<crazyflie_gazebo::LogBlock>& log_blocks,
  bool use_ros_time,
  bool enable_logging_imu,
  bool enable_logging_temperature,
  bool enable_logging_magnetic_field,
  bool enable_logging_pressure,
  bool enable_logging_battery,
  bool enable_logging_packets)
: m_cf(cf_uri,rosLogger)
, m_tf_prefix(tf_prefix)
, m_isEmergency(false)
, m_roll_trim(roll_trim)
, m_pitch_trim(pitch_trim)
, m_enableLogging(enable_logging)
, m_enableParameters(enable_parameters)
, m_logBlocks(log_blocks)
, m_use_ros_time(use_ros_time)
, m_enable_logging_imu(enable_logging_imu)
, m_enable_logging_temperature(enable_logging_temperature)
, m_enable_logging_magnetic_field(enable_logging_magnetic_field)
, m_enable_logging_pressure(enable_logging_pressure)
, m_enable_logging_battery(enable_logging_battery)
, m_enable_logging_packets(enable_logging_packets)
, m_serviceEmergency()
, m_serviceUpdateParams()
, m_serviceSetGroupMask()
, m_serviceTakeoff()
, m_serviceLand()
, m_serviceStop()
, m_serviceGoTo()
, m_serviceUploadTrajectory()
, m_serviceStartTrajectory()
, m_subscribeCmdVel()
, m_subscribeCmdFullState()
, m_subscribeCmdHover()
, m_subscribeCmdStop()
, m_subscribeCmdPosition()
, m_subscribeExternalPosition()
, m_pubImu()
, m_pubTemp()
, m_pubMag()
, m_pubPressure()
, m_pubBattery()
, m_pubRssi()
, m_sentSetpoint(false)
/*, m_sentExternalPosition(false)
, m_sentExternalImu(false)*/
, m_gyrobias_found(false)
, first_pos_sent(false)
{
  m_thread = std::thread(&CrazyflieROS::run, this);
}

CrazyflieROS::~CrazyflieROS(){
  m_cf.reboot();
  m_isEmergency = true;
  m_thread.join();
  ROS_INFO("Delete CrazyflieROS Instance");
}

void CrazyflieROS::setOnMotorsData(std::function<void(const crtpMotorsDataResponse*)> cb){
  m_cf.setMotorsCallback(cb);
}

void CrazyflieROS::sendSensorsPacket(const uint8_t* data, uint32_t length){
  m_cf.sendPacketNoAck(data , length);
  // m_sentExternalImu.store(true);
}

void CrazyflieROS::sendExternalPositionUpdate(float x , float y , float z){
  if ((!first_pos_sent) && m_gyrobias_found.load()){
    // declare kalman parameters
    std::string kalman_initX = "/" + m_tf_prefix + "/kalman/initialX";
    std::string kalman_initY = "/" + m_tf_prefix + "/kalman/initialY";
    std::string kalman_initZ = "/" + m_tf_prefix + "/kalman/initialZ";
    std::string reset_estim = "/" + m_tf_prefix + "/kalman/resetEstimation";
    
    // get id entry for the params
    auto entry_resetEstim = m_cf.getParamTocEntry("kalman" , "resetEstimation");
    auto entry_initX = m_cf.getParamTocEntry("kalman" , "initialX");
    auto entry_initY = m_cf.getParamTocEntry("kalman" , "initialY");
    auto entry_initZ = m_cf.getParamTocEntry("kalman" , "initialZ");

    ros::param::set(kalman_initX , x);
    ros::param::set(kalman_initY , y);
    ros::param::set(kalman_initZ , z);

    // update initial position
    updateParam<float , float>(entry_initX->id , kalman_initX);
    updateParam<float , float>(entry_initY->id , kalman_initY);
    updateParam<float , float>(entry_initZ->id , kalman_initZ);
    ros::Duration(1.0).sleep(); // lazy trick to wait enough for the params to init

    ros::param::set(reset_estim , 1);
    updateParam<uint8_t , int>(entry_resetEstim->id , reset_estim);
    ros::Duration(1.0).sleep(); // lazy trick to avoid restart of cf2 in hitl

    ROS_INFO("RESET OF KALMAN DONE !");
    first_pos_sent = true;
  }
  if (first_pos_sent){
    m_cf.sendExternalPositionUpdate(x,y,z);
    // m_sentExternalPosition.store(true);
  }
}

void CrazyflieROS::stop()
{
  ROS_INFO("Disconnecting ...");
  m_isEmergency = true;
  m_thread.join();
}

bool CrazyflieROS::sendPacket (
  crazyflie_gazebo::sendPacket::Request &req,
  crazyflie_gazebo::sendPacket::Response &res)
{
    /** Convert the message struct to the packet struct */
  crtpPacket_t packet;
  packet.size = req.packet.size;
  packet.header = req.packet.header;
  for (int i = 0; i < CRTP_MAX_DATA_SIZE; i++) {
    packet.data[i] = req.packet.data[i];
  }
  m_cf.queueOutgoingPacket(packet);
  return true;
}

void CrazyflieROS::publishPackets() {
  std::vector<Crazyradio::Ack> packets = m_cf.retrieveGenericPackets();
  if (!packets.empty())
  {
    std::vector<Crazyradio::Ack>::iterator it;
    for (it = packets.begin(); it != packets.end(); it++)
    {
      crazyflie_gazebo::crtpPacket packet;
      packet.size = it->size;
      packet.header = it->data[0];
      for(int i = 0; i < packet.size; i++)
      {
        packet.data[i] = it->data[i+1];
      }
      m_pubPackets.publish(packet);
    }
  }
}

bool CrazyflieROS::emergency(
  std_srvs::Empty::Request& req,
  std_srvs::Empty::Response& res)
{
  ROS_FATAL("Emergency requested!");
  m_isEmergency = true;

  return true;
}


void CrazyflieROS::cmdHoverSetpoint(
  const crazyflie_gazebo::Hover::ConstPtr& msg)
{
     //ROS_INFO("got a hover setpoint");
  if (!m_isEmergency) {
    float vx = msg->vx;
    float vy = msg->vy;
    float yawRate = msg->yawrate;
    float zDistance = msg->zDistance;

    m_cf.sendHoverSetpoint(vx, vy, yawRate, zDistance);
    m_sentSetpoint = true;
      //ROS_INFO("set a hover setpoint");
  }
}

void CrazyflieROS::cmdStop(
  const std_msgs::Empty::ConstPtr& msg)
{
     //ROS_INFO("got a stop setpoint");
  if (!m_isEmergency) {
    m_cf.sendStop();
    m_sentSetpoint = true;
      //ROS_INFO("set a stop setpoint");
  }
}

void CrazyflieROS::cmdPositionSetpoint(
  const crazyflie_gazebo::Position::ConstPtr& msg)
{
  if(!m_isEmergency) {
    float x = msg->x;
    float y = msg->y;
    float z = msg->z;
    float yaw = msg->yaw;

    m_cf.sendPositionSetpoint(x, y, z, yaw);
    m_sentSetpoint = true;
  }
}

bool CrazyflieROS::updateParams(
  crazyflie_gazebo::UpdateParams::Request& req,
  crazyflie_gazebo::UpdateParams::Response& res)
{
  ROS_INFO("Update parameters");
  for (auto&& p : req.params) {
    std::string ros_param = "/" + m_tf_prefix + "/" + p;
    size_t pos = p.find("/");
    std::string group(p.begin(), p.begin() + pos);
    std::string name(p.begin() + pos + 1, p.end());

    auto entry = m_cf.getParamTocEntry(group, name);
    if (entry)
    {
      switch (entry->type) {
        case Crazyflie::ParamTypeUint8:
        updateParam<uint8_t, int>(entry->id, ros_param);
        break;
        case Crazyflie::ParamTypeInt8:
        updateParam<int8_t, int>(entry->id, ros_param);
        break;
        case Crazyflie::ParamTypeUint16:
        updateParam<uint16_t, int>(entry->id, ros_param);
        break;
        case Crazyflie::ParamTypeInt16:
        updateParam<int16_t, int>(entry->id, ros_param);
        break;
        case Crazyflie::ParamTypeUint32:
        updateParam<uint32_t, int>(entry->id, ros_param);
        break;
        case Crazyflie::ParamTypeInt32:
        updateParam<int32_t, int>(entry->id, ros_param);
        break;
        case Crazyflie::ParamTypeFloat:
        updateParam<float, float>(entry->id, ros_param);
        break;
      }
    }
    else {
      ROS_ERROR("Could not find param %s/%s", group.c_str(), name.c_str());
    }
  }
  return true;
}

void CrazyflieROS::cmdVelChanged(
  const geometry_msgs::Twist::ConstPtr& msg)
{
  if (!m_isEmergency) {
    float roll = msg->linear.y + m_roll_trim;
    float pitch = - (msg->linear.x + m_pitch_trim);
    float yawrate = msg->angular.z;
    uint16_t thrust = std::min<uint16_t>(std::max<float>(msg->linear.z, 0.0), 60000);

    m_cf.sendSetpoint(roll, pitch, yawrate, thrust);
    m_sentSetpoint = true;
  }
}

void CrazyflieROS::cmdFullStateSetpoint(
  const crazyflie_gazebo::FullState::ConstPtr& msg)
{
    //ROS_INFO("got a full state setpoint");
  if (!m_isEmergency) {
    float x = msg->pose.position.x;
    float y = msg->pose.position.y;
    float z = msg->pose.position.z;
    float vx = msg->twist.linear.x;
    float vy = msg->twist.linear.y;
    float vz = msg->twist.linear.z;
    float ax = msg->acc.x;
    float ay = msg->acc.y;
    float az = msg->acc.z;

    float qx = msg->pose.orientation.x;
    float qy = msg->pose.orientation.y;
    float qz = msg->pose.orientation.z;
    float qw = msg->pose.orientation.w;
    float rollRate = msg->twist.angular.x;
    float pitchRate = msg->twist.angular.y;
    float yawRate = msg->twist.angular.z;

    m_cf.sendFullStateSetpoint(
      x, y, z,
      vx, vy, vz,
      ax, ay, az,
      qx, qy, qz, qw,
      rollRate, pitchRate, yawRate);
    m_sentSetpoint = true;
      //ROS_INFO("set a full state setpoint");
  }
}

void CrazyflieROS::positionMeasurementChanged(
  const geometry_msgs::PointStamped::ConstPtr& msg)
{
  sendExternalPositionUpdate(msg->point.x, msg->point.y, msg->point.z);
}

void CrazyflieROS::run()
{
  ros::NodeHandle n;
  n.setCallbackQueue(&m_callback_queue);

  // solve LIBUSB_ERROR_TIMEOUT always on the first call
  try {
    m_cf.requestParamToc();
  } catch (std::runtime_error& e){
    ROS_INFO("Runtime Error catched : %s" , e.what());
  }

  ROS_INFO("Creating CrazyflieROS services ");

  m_subscribeCmdVel = n.subscribe(m_tf_prefix + "/cmd_vel", 1, &CrazyflieROS::cmdVelChanged, this);
  m_subscribeCmdFullState = n.subscribe(m_tf_prefix + "/cmd_full_state", 1, &CrazyflieROS::cmdFullStateSetpoint, this);
  m_subscribeExternalPosition = n.subscribe(m_tf_prefix + "/external_position", 1, &CrazyflieROS::positionMeasurementChanged, this);
  m_serviceEmergency = n.advertiseService(m_tf_prefix + "/emergency", &CrazyflieROS::emergency, this);
  m_subscribeCmdHover = n.subscribe(m_tf_prefix + "/cmd_hover", 1, &CrazyflieROS::cmdHoverSetpoint, this);
  m_subscribeCmdStop = n.subscribe(m_tf_prefix + "/cmd_stop", 1, &CrazyflieROS::cmdStop, this);
  m_subscribeCmdPosition = n.subscribe(m_tf_prefix + "/cmd_position", 1, &CrazyflieROS::cmdPositionSetpoint, this);

  m_pubState = n.advertise<geometry_msgs::PoseStamped>(m_tf_prefix + "/pose",10);

  m_serviceSetGroupMask = n.advertiseService(m_tf_prefix + "/set_group_mask", &CrazyflieROS::setGroupMask, this);
  m_serviceTakeoff = n.advertiseService(m_tf_prefix + "/takeoff", &CrazyflieROS::takeoff, this);
  m_serviceLand = n.advertiseService(m_tf_prefix + "/land", &CrazyflieROS::land, this);
  m_serviceStop = n.advertiseService(m_tf_prefix + "/stop", &CrazyflieROS::stop, this);
  m_serviceGoTo = n.advertiseService(m_tf_prefix + "/go_to", &CrazyflieROS::goTo, this);
  m_serviceUploadTrajectory = n.advertiseService(m_tf_prefix + "/upload_trajectory", &CrazyflieROS::uploadTrajectory, this);
  m_serviceStartTrajectory = n.advertiseService(m_tf_prefix + "/start_trajectory", &CrazyflieROS::startTrajectory, this);

  ROS_INFO("Enable logging : %s", m_enableLogging ? "TRUE" : "FALSE");
  ROS_INFO("Enable logging imu : %s", m_enable_logging_imu ? "TRUE" : "FALSE");
  ROS_INFO("Enable logging Temperature : %s", m_enable_logging_temperature ? "TRUE" : "FALSE");
  ROS_INFO("Enable logging mag field : %s", m_enable_logging_magnetic_field ? "TRUE" : "FALSE");
  ROS_INFO("Enable logging pressure : %s", m_enable_logging_pressure ? "TRUE" : "FALSE");
  ROS_INFO("Enable logging battery : %s", m_enable_logging_battery ? "TRUE" : "FALSE");
  ROS_INFO("Enable logging packets : %s", m_enable_logging_packets ? "TRUE" : "FALSE");
  ROS_INFO("Enable parameters : %s", m_enableParameters ? "TRUE" : "FALSE");

  if (m_enable_logging_imu) {
    m_pubImu = n.advertise<sensor_msgs::Imu>(m_tf_prefix + "/imu", 10);
  }
  if (m_enable_logging_temperature) {
    m_pubTemp = n.advertise<sensor_msgs::Temperature>(m_tf_prefix + "/temperature", 10);
  }
  if (m_enable_logging_magnetic_field) {
    m_pubMag = n.advertise<sensor_msgs::MagneticField>(m_tf_prefix + "/magnetic_field", 10);
  }
  if (m_enable_logging_pressure) {
    m_pubPressure = n.advertise<std_msgs::Float32>(m_tf_prefix + "/pressure", 10);
  }
  if (m_enable_logging_battery) {
    m_pubBattery = n.advertise<std_msgs::Float32>(m_tf_prefix + "/battery", 10);
  }
  if (m_enable_logging_packets) {
    m_pubPackets = n.advertise<crazyflie_gazebo::crtpPacket>(m_tf_prefix + "/packets", 10);
  }

  m_pubRssi = n.advertise<std_msgs::Float32>(m_tf_prefix + "/rssi", 10);

  for (auto& logBlock : m_logBlocks)
  {
    m_pubLogDataGeneric.push_back(n.advertise<crazyflie_gazebo::GenericLogData>(m_tf_prefix + "/" + logBlock.topic_name, 10));
  }

  m_sendPacketServer = n.advertiseService(m_tf_prefix + "/send_packet"  , &CrazyflieROS::sendPacket, this);
  // m_cf.reboot();

  auto start = std::chrono::system_clock::now();

  std::function<void(const char*)> cb_console = std::bind(&CrazyflieROS::onConsole, this, std::placeholders::_1);
  m_cf.setConsoleCallback(cb_console);

  m_cf.logReset();

  std::function<void(float)> cb_lq = std::bind(&CrazyflieROS::onLinkQuality, this, std::placeholders::_1);
  m_cf.setLinkQualityCallback(cb_lq);

  std::function<void(const crtpImuSimDataResponse*)> cb_imu_res = std::bind(&CrazyflieROS::onImuSimDataResponse, this, std::placeholders::_1);
  m_cf.setImuSimResponseCallback(cb_imu_res);

  if (m_enableParameters)
  {
    ROS_INFO("Requesting parameters...");
    m_cf.requestParamToc();
    for (auto iter = m_cf.paramsBegin(); iter != m_cf.paramsEnd(); ++iter) {
      auto entry = *iter;
      std::string paramName = "/" + m_tf_prefix + "/" + entry.group + "/" + entry.name;
      switch (entry.type) {
        case Crazyflie::ParamTypeUint8:
        ros::param::set(paramName, m_cf.getParam<uint8_t>(entry.id));
        ROS_INFO("%s (uint8_t) : %d" , paramName.c_str() , (int) m_cf.getParam<uint8_t>(entry.id));
        break;
        case Crazyflie::ParamTypeInt8:
        ros::param::set(paramName, m_cf.getParam<int8_t>(entry.id));
        ROS_INFO("%s (int8_t) : %d" , paramName.c_str() , (int) m_cf.getParam<int8_t>(entry.id));
        break;
        case Crazyflie::ParamTypeUint16:
        ros::param::set(paramName, m_cf.getParam<uint16_t>(entry.id));
        ROS_INFO("%s (uint16_t) : %d" , paramName.c_str() , (int) m_cf.getParam<uint16_t>(entry.id));
        break;
        case Crazyflie::ParamTypeInt16:
        ros::param::set(paramName, m_cf.getParam<int16_t>(entry.id));
        ROS_INFO("%s (int16_t) : %d" , paramName.c_str() , (int) m_cf.getParam<int16_t>(entry.id));
        break;
        case Crazyflie::ParamTypeUint32:
        ros::param::set(paramName, (int)m_cf.getParam<uint32_t>(entry.id));
        ROS_INFO("%s (uint32_t) : %d" , paramName.c_str() , (int) m_cf.getParam<uint32_t>(entry.id));
        break;
        case Crazyflie::ParamTypeInt32:
        ros::param::set(paramName, m_cf.getParam<int32_t>(entry.id));
        ROS_INFO("%s (int32_t) : %d" , paramName.c_str() , (int) m_cf.getParam<int32_t>(entry.id));
        break;
        case Crazyflie::ParamTypeFloat:
        ros::param::set(paramName, m_cf.getParam<float>(entry.id));
        ROS_INFO("%s (float) : %f" , paramName.c_str() ,  m_cf.getParam<float>(entry.id));
        break;
      }
    }
    m_serviceUpdateParams = n.advertiseService(m_tf_prefix + "/update_params", &CrazyflieROS::updateParams, this);
  }

  std::unique_ptr<LogBlock<logImu> > logBlockImu;
  std::unique_ptr<LogBlock<log2> > logBlock2;
  std::unique_ptr<LogBlock<logStats> > logStatsData;
  std::unique_ptr<LogBlock<logState> > logStateData;

  std::vector<std::unique_ptr<LogBlockGeneric> > logBlocksGeneric(m_logBlocks.size());
  if (m_enableLogging) {

    std::function<void(const crtpPlatformRSSIAck*)> cb_ack = std::bind(&CrazyflieROS::onEmptyAck, this, std::placeholders::_1);
    m_cf.setEmptyAckCallback(cb_ack);

    ROS_INFO("Requesting Logging variables...");
    m_cf.requestLogToc();

    std::for_each(m_cf.logVariablesBegin(), m_cf.logVariablesEnd(),
      [](const Crazyflie::LogTocEntry& entry)
      {
        std::string logName = entry.group + "." + entry.name + " (";
        switch (entry.type) {
          case Crazyflie::LogTypeUint8:
          ROS_INFO("%s uint8)", logName.c_str());
          break;
          case Crazyflie::LogTypeInt8:
          ROS_INFO("%s int8)", logName.c_str());
          break;
          case Crazyflie::LogTypeUint16:
          ROS_INFO("%s uint16)", logName.c_str());
          break;
          case Crazyflie::LogTypeInt16:
          ROS_INFO("%s int16)", logName.c_str());
          break;
          case Crazyflie::LogTypeUint32:
          ROS_INFO("%s uint32)", logName.c_str());
          break;
          case Crazyflie::LogTypeInt32:
          ROS_INFO("%s int32)", logName.c_str());
          break;
          case Crazyflie::LogTypeFloat:
          ROS_INFO("%s float)", logName.c_str());
          break;
          case Crazyflie::LogTypeFP16:
          ROS_INFO("%s fp16)", logName.c_str());
          break;
        }
      }
      );

    if (m_enable_logging_imu) {
      std::function<void(uint32_t, logImu*)> cb = std::bind(&CrazyflieROS::onImuData, this, std::placeholders::_1, std::placeholders::_2);

      logBlockImu.reset(new LogBlock<logImu>(
        &m_cf,{
          {"acc", "x"},
          {"acc", "y"},
          {"acc", "z"},
          {"gyro", "x"},
          {"gyro", "y"},
          {"gyro", "z"},
        }, cb));
        logBlockImu->start(1); // 10ms
      }

      if (   m_enable_logging_temperature
        || m_enable_logging_magnetic_field
        || m_enable_logging_pressure
        || m_enable_logging_battery)
      {
        std::function<void(uint32_t, log2*)> cb2 = std::bind(&CrazyflieROS::onLog2Data, this, std::placeholders::_1, std::placeholders::_2);

        logBlock2.reset(new LogBlock<log2>(
          &m_cf,{
            {"mag", "x"},
            {"mag", "y"},
            {"mag", "z"},
            {"baro", "temp"},
            {"baro", "pressure"},
            {"pm", "vbat"},
          }, cb2));
        logBlock2->start(10); // 100ms
      }

      /* Print Receive - Drop - Transmit Rate */
      /*
      std::function<void(uint32_t, logStats*)> cbStats = std::bind(&CrazyflieROS::onLogStatData, this, std::placeholders::_1, std::placeholders::_2);
      logStatsData.reset(new LogBlock<logStats>(
        &m_cf,{
          {"crtp", "rxRate"},
          {"crtp", "rxDrpRte"},
          {"crtp", "txRate"},
        }, cbStats));
      logStatsData->start(200);*/

      std::function<void(uint32_t, logState*)> cbQuadState = std::bind(&CrazyflieROS::onLogStateData, this, std::placeholders::_1, std::placeholders::_2);
      logStateData.reset(new LogBlock<logState>(
        &m_cf,{
          {"stateEstimate", "x"},
          {"stateEstimate", "y"},
          {"stateEstimate", "z"},
          {"stabilizer", "roll"},
          {"stabilizer", "pitch"},
          {"stabilizer", "yaw"},
        }, cbQuadState));
      logStateData->start(1);

      // custom log blocks
      size_t i = 0;
      for (auto& logBlock : m_logBlocks)
      {
        std::function<void(uint32_t, std::vector<double>*, void* userData)> cb =
        std::bind(
          &CrazyflieROS::onLogCustom,
          this,
          std::placeholders::_1,
          std::placeholders::_2,
          std::placeholders::_3);

        logBlocksGeneric[i].reset(new LogBlockGeneric(
          &m_cf,
          logBlock.variables,
          (void*)&m_pubLogDataGeneric[i],
          cb));
        logBlocksGeneric[i]->start(logBlock.frequency / 10);
        ++i;
      }


    }

    ROS_INFO("Requesting memories...");
    m_cf.requestMemoryToc();

    ROS_INFO("Ready...");
    auto end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsedSeconds = end-start;
    ROS_INFO("Elapsed: %f s", elapsedSeconds.count());

    // Send 0 thrust initially for thrust-lock
    for (int i = 0; i < 100; ++i) {
     m_cf.sendSetpoint(0, 0, 0, 0);
   }

   Crazyradio::Ack ack;

   while(!m_isEmergency) {
    if (m_cf.isSITLsim() && m_enableLogging){
      m_cf.transmitPackets();
      m_cf.recvPacket(ack);
      if(m_enable_logging_packets) {
        this->publishPackets();
      }
    }else if (!m_cf.isSITLsim() && m_enableLogging  && !m_sentSetpoint){
      // make sure we ping often enough to stream data out
      m_cf.transmitPackets();
      m_cf.sendPing();
      if(m_enable_logging_packets) {
        this->publishPackets();
      }
      m_sentSetpoint = false;
    }

    // Execute any ROS related functions now
    m_callback_queue.callAvailable(ros::WallDuration(0.0));

    // In case of SITL, no need delay since the delay is done when doing recvPacket
    if (!m_cf.isSITLsim())
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

    // Make sure we turn the engines off
  for (int i = 0; i < 100; ++i) {
   m_cf.sendSetpoint(0, 0, 0, 0);
 }

}

void CrazyflieROS::onImuData(uint32_t time_in_ms, logImu* data) {
  if (m_enable_logging_imu) {
    sensor_msgs::Imu msg;
    if (m_use_ros_time) {
      msg.header.stamp = ros::Time::now();
    } else {
      msg.header.stamp = ros::Time(time_in_ms / 1000.0);
    }
    msg.header.frame_id = m_tf_prefix + "/base_link";
    msg.orientation_covariance[0] = -1;

      // measured in deg/s; need to convert to rad/s
    msg.angular_velocity.x = degToRad(data->gyro_x);
    msg.angular_velocity.y = degToRad(data->gyro_y);
    msg.angular_velocity.z = degToRad(data->gyro_z);

      // measured in mG; need to convert to m/s^2
    msg.linear_acceleration.x = data->acc_x * 9.81;
    msg.linear_acceleration.y = data->acc_y * 9.81;
    msg.linear_acceleration.z = data->acc_z * 9.81;

    m_pubImu.publish(msg);
  }
}

void CrazyflieROS::onLogStateData(uint32_t time_in_ms, logState* data) {
    geometry_msgs::PoseStamped msg;
    if (m_use_ros_time) {
      msg.header.stamp = ros::Time::now();
    } else {
      msg.header.stamp = ros::Time(time_in_ms / 1000.0);
    }
    msg.header.frame_id = m_tf_prefix + "/base_link";
    msg.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(degToRad(data->roll), degToRad(data->pitch), degToRad(data->yaw));
    msg.pose.position.x = data->x;
    msg.pose.position.y = data->y;
    msg.pose.position.z = data->z;
    m_pubState.publish(msg);
}

void CrazyflieROS::onLogStatData(uint32_t time_in_ms, logStats* data){
  ROS_WARN("rxRate = %d | rxDrpRte = %d | txRate = %d", (int) data->rxRate, (int) data->rxDrpRte, (int) data->txRate);
}

void CrazyflieROS::onLog2Data(uint32_t time_in_ms, log2* data) {

  if (m_enable_logging_temperature) {
    sensor_msgs::Temperature msg;
    if (m_use_ros_time) {
      msg.header.stamp = ros::Time::now();
    } else {
      msg.header.stamp = ros::Time(time_in_ms / 1000.0);
    }
    msg.header.frame_id = m_tf_prefix + "/base_link";
      // measured in degC
    msg.temperature = data->baro_temp;
    m_pubTemp.publish(msg);
  }

  if (m_enable_logging_magnetic_field) {
    sensor_msgs::MagneticField msg;
    if (m_use_ros_time) {
      msg.header.stamp = ros::Time::now();
    } else {
      msg.header.stamp = ros::Time(time_in_ms / 1000.0);
    }
    msg.header.frame_id = m_tf_prefix + "/base_link";

      // measured in Tesla
    msg.magnetic_field.x = data->mag_x;
    msg.magnetic_field.y = data->mag_y;
    msg.magnetic_field.z = data->mag_z;
    m_pubMag.publish(msg);
  }

  if (m_enable_logging_pressure) {
    std_msgs::Float32 msg;
      // hPa (=mbar)
    msg.data = data->baro_pressure;
    m_pubPressure.publish(msg);
  }

  if (m_enable_logging_battery) {
    std_msgs::Float32 msg;
      // V
    msg.data = data->pm_vbat;
    m_pubBattery.publish(msg);
  }
}

void CrazyflieROS::onLogCustom(uint32_t time_in_ms, std::vector<double>* values, void* userData) {

  ros::Publisher* pub = reinterpret_cast<ros::Publisher*>(userData);

  crazyflie_gazebo::GenericLogData msg;
  if (m_use_ros_time) {
    msg.header.stamp = ros::Time::now();
  } else {
    msg.header.stamp = ros::Time(time_in_ms / 1000.0);
  }
  msg.header.frame_id = m_tf_prefix + "/base_link";
  msg.values = *values;

  pub->publish(msg);
}

void CrazyflieROS::onEmptyAck(const crtpPlatformRSSIAck* data) {
  std_msgs::Float32 msg;
      // dB
  msg.data = data->rssi;
  m_pubRssi.publish(msg);
}

void CrazyflieROS::onLinkQuality(float linkQuality) {
  if (linkQuality < 0.7) {
    ROS_WARN("Link Quality low (%f)", linkQuality);
  }
}

void CrazyflieROS::onImuSimDataResponse(const crtpImuSimDataResponse* imuResp){
  m_gyrobias_found = imuResp->isGyroBiasFound;
}

void CrazyflieROS::onConsole(const char* msg) {
  ROS_INFO("CF Console: %s", msg);
}

bool CrazyflieROS::setGroupMask(
  crazyflie_gazebo::SetGroupMask::Request& req,
  crazyflie_gazebo::SetGroupMask::Response& res)
{
  ROS_INFO("SetGroupMask requested");
  m_cf.setGroupMask(req.groupMask);
  return true;
}

bool CrazyflieROS::takeoff(
  crazyflie_gazebo::Takeoff::Request& req,
  crazyflie_gazebo::Takeoff::Response& res)
{
  ROS_INFO("Takeoff requested");
  m_cf.takeoff(req.height, req.duration.toSec(), req.groupMask);
  return true;
}

bool CrazyflieROS::land(
  crazyflie_gazebo::Land::Request& req,
  crazyflie_gazebo::Land::Response& res)
{
  ROS_INFO("Land requested");
  m_cf.land(req.height, req.duration.toSec(), req.groupMask);
  return true;
}

bool CrazyflieROS::stop(
  crazyflie_gazebo::Stop::Request& req,
  crazyflie_gazebo::Stop::Response& res)
{
  ROS_INFO("Stop requested");
  m_cf.stop(req.groupMask);
  return true;
}

bool CrazyflieROS::goTo(
  crazyflie_gazebo::GoTo::Request& req,
  crazyflie_gazebo::GoTo::Response& res)
{
  ROS_INFO("GoTo requested");
  m_cf.goTo(req.goal.x, req.goal.y, req.goal.z, req.yaw, req.duration.toSec(), req.relative, req.groupMask);
  return true;
}

bool CrazyflieROS::uploadTrajectory(
  crazyflie_gazebo::UploadTrajectory::Request& req,
  crazyflie_gazebo::UploadTrajectory::Response& res)
{
  ROS_INFO("UploadTrajectory requested");

  std::vector<Crazyflie::poly4d> pieces(req.pieces.size());
  for (size_t i = 0; i < pieces.size(); ++i) {
    if (   req.pieces[i].poly_x.size() != 8
      || req.pieces[i].poly_y.size() != 8
      || req.pieces[i].poly_z.size() != 8
      || req.pieces[i].poly_yaw.size() != 8) {
      ROS_FATAL("Wrong number of pieces!");
    return false;
  }
  pieces[i].duration = req.pieces[i].duration.toSec();
  for (size_t j = 0; j < 8; ++j) {
    pieces[i].p[0][j] = req.pieces[i].poly_x[j];
    pieces[i].p[1][j] = req.pieces[i].poly_y[j];
    pieces[i].p[2][j] = req.pieces[i].poly_z[j];
    pieces[i].p[3][j] = req.pieces[i].poly_yaw[j];
  }
}
m_cf.uploadTrajectory(req.trajectoryId, req.pieceOffset, pieces);

ROS_INFO("Upload completed!");
return true;
}

bool CrazyflieROS::startTrajectory(
  crazyflie_gazebo::StartTrajectory::Request& req,
  crazyflie_gazebo::StartTrajectory::Response& res)
{
  ROS_INFO("StartTrajectory requested");
  m_cf.startTrajectory(req.trajectoryId, req.timescale, req.reversed, req.relative, req.groupMask);
  return true;
}

CrazyflieServer::CrazyflieServer()
{

}

void CrazyflieServer::run()
{
  ros::NodeHandle n;
  ros::CallbackQueue callback_queue;
  n.setCallbackQueue(&callback_queue);

  ros::ServiceServer serviceAdd = n.advertiseService("add_crazyflie", &CrazyflieServer::add_crazyflie, this);
  ros::ServiceServer serviceRemove = n.advertiseService("remove_crazyflie", &CrazyflieServer::remove_crazyflie, this);

  while(ros::ok()) {
      // Execute any ROS related functions now
    callback_queue.callAvailable(ros::WallDuration(0.0));
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

bool CrazyflieServer::add_crazyflie(
  crazyflie_gazebo::AddCrazyflie::Request  &req,
  crazyflie_gazebo::AddCrazyflie::Response &res)
{
  ROS_INFO("Adding %s as %s with trim(%f, %f). Logging: %d, Parameters: %d, Use ROS time: %d",
    req.uri.c_str(),
    req.tf_prefix.c_str(),
    req.roll_trim,
    req.pitch_trim,
    req.enable_parameters,
    req.enable_logging,
    req.use_ros_time);

    // Ignore if the uri is already in use
  if (m_crazyflies.find(req.uri) != m_crazyflies.end()) {
    ROS_ERROR("Cannot add %s, already added.", req.uri.c_str());
    return false;
  }

  CrazyflieROS* cf = new CrazyflieROS(
    req.uri,
    req.tf_prefix,
    req.roll_trim,
    req.pitch_trim,
    req.enable_logging,
    req.enable_parameters,
    req.log_blocks,
    req.use_ros_time,
    req.enable_logging_imu,
    req.enable_logging_temperature,
    req.enable_logging_magnetic_field,
    req.enable_logging_pressure,
    req.enable_logging_battery,
    req.enable_logging_packets);

  m_crazyflies[req.uri] = cf;

  return true;
}

CrazyflieServer::~CrazyflieServer(){
  std::map<std::string, CrazyflieROS*>::iterator it = m_crazyflies.begin();
  while( it != m_crazyflies.end()){
    (it->second)->stop();
    delete (it->second); 
  }
}

bool CrazyflieServer::remove_crazyflie(
  crazyflie_gazebo::RemoveCrazyflie::Request  &req,
  crazyflie_gazebo::RemoveCrazyflie::Response &res)
{

  if (m_crazyflies.find(req.uri) == m_crazyflies.end()) {
    ROS_ERROR("Cannot remove %s, not connected.", req.uri.c_str());
    return false;
  }

  ROS_INFO("Removing crazyflie at uri %s.", req.uri.c_str());

  m_crazyflies[req.uri]->stop();
  delete m_crazyflies[req.uri];
  m_crazyflies.erase(req.uri);

  ROS_INFO("Crazyflie %s removed.", req.uri.c_str());

  return true;
}