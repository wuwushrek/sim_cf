#include <iostream>
#include <stdio.h>

#include "crazyflie_cpp/Crazyradio.h"
#include "crazyflie_cpp/crtp.h"
#include "crazyflie_cpp/Crazyflie.h"

// #include "crazyflie_ros/CrazyflieROS.h"

#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>

#include <thread>
#include <ros/ros.h>
#include <ros/callback_queue.h>

//#define ROS_INFO(X) printf(X);

struct sockaddr_in myaddr;

struct sockaddr_in remaddr_rcv;
socklen_t addrlen_rcv;

int fd;

uint8_t buf[35];

struct logImu {
	float acc_x;
	float acc_y;
	float acc_z;
	float gyro_x;
	float gyro_y;
	float gyro_z;
} __attribute__((packed));

/*************** Additions/Modifications for Simulation ****************/
struct logStats {
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
struct logSim {
	float mag_x;
	float mag_y;
	float mag_z;
	float baro_temp;
	float baro_pressure;
	float pm_vbat;
} __attribute__((packed));


void onImuData(uint32_t time_in_ms, logImu* data)
{

}

void onLog2Data(uint32_t time_in_ms, logSim* data)
{

}

void onLogStateData(uint32_t time_in_ms, logState* data)
{

}

void sendData(const uint8_t* data , uint32_t length)
{	
	// std::cout << "header : " << (int) data[0] << std::endl; 
	int transferred = sendto(fd, data, length, 0, (struct sockaddr *) &(remaddr_rcv), addrlen_rcv);
	if (transferred <= 0)
		throw std::runtime_error(strerror(errno));
}

void recvData(Crazyradio::Ack &ack , int64_t timeout_us)
{
	int len = recvfrom(fd, buf, sizeof(buf), 0, (struct sockaddr *) &remaddr_rcv, &addrlen_rcv);
	if (len <= 0 ){
		ack.ack = false;
		return;
	}
	uint8_t cf_id = buf[0]-1;
	if (buf[1] == 0xF3 && len == 2){
		// Send response to SITL instance
		uint8_t data[2] = {(uint8_t)(cf_id+1) , 0xF3};
		sendData(data , sizeof(data));
	}else {
		ack.ack = true;
		memcpy(ack.data , &(buf[1]) , sizeof(uint8_t) * (len-1));
		ack.size = len-1;
	}

}

int main(int argc, char **argv)
{

	// open socket in UDP mode
	if ((fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
    	std::cout << "create socket failed " << std::endl;
    	return 0;
  	}

  	memset((char *)&myaddr, 0, sizeof(myaddr));
  	myaddr.sin_family = AF_INET;
	myaddr.sin_addr.s_addr = htonl(INADDR_ANY);
	myaddr.sin_port = htons(19950);
  	if (bind(fd, (struct sockaddr *)&myaddr, sizeof(myaddr)) < 0){
    	std::cout << "Bind socket failed " << std::endl;
    	return 0;
  	}

  	addrlen_rcv = sizeof(remaddr_rcv);

	std::function<void(const uint8_t* , uint32_t)> sendDataFunc = sendData; //boost::bind(send , _1 , _2);
	std::function<void(Crazyradio::Ack& , int64_t)> recvDataFunc = recvData;//boost::bind(recv , _1 , _2);
	
	Crazyflie m_cf(sendDataFunc , recvDataFunc);

	Crazyradio::Ack ack;
	recvData(ack, 1000000);

	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	try {
		m_cf.requestParamToc();
  	} catch (std::runtime_error& e){
    	std::cout << "Runtime Error catched : " << e.what() << std::endl;
  	}

  	m_cf.logReset();

  	std::string m_tf_prefix = "cf1";
  	ROS_INFO("Requesting parameters...");
  	m_cf.requestParamToc();
    for (auto iter = m_cf.paramsBegin(); iter != m_cf.paramsEnd(); ++iter) {
      auto entry = *iter;
      std::string paramName = "/" + m_tf_prefix + "/" + entry.group + "/" + entry.name;
      switch (entry.type) {
        case Crazyflie::ParamTypeUint8:
        ROS_INFO("%s (uint8_t) : %d" , paramName.c_str() , (int) m_cf.getParam<uint8_t>(entry.id));
        break;
        case Crazyflie::ParamTypeInt8:
        ROS_INFO("%s (int8_t) : %d" , paramName.c_str() , (int) m_cf.getParam<int8_t>(entry.id));
        break;
        case Crazyflie::ParamTypeUint16:
        ROS_INFO("%s (uint16_t) : %d" , paramName.c_str() , (int) m_cf.getParam<uint16_t>(entry.id));
        break;
        case Crazyflie::ParamTypeInt16:
        ROS_INFO("%s (int16_t) : %d" , paramName.c_str() , (int) m_cf.getParam<int16_t>(entry.id));
        break;
        case Crazyflie::ParamTypeUint32:
        ROS_INFO("%s (uint32_t) : %d" , paramName.c_str() , (int) m_cf.getParam<uint32_t>(entry.id));
        break;
        case Crazyflie::ParamTypeInt32:
        ROS_INFO("%s (int32_t) : %d" , paramName.c_str() , (int) m_cf.getParam<int32_t>(entry.id));
        break;
        case Crazyflie::ParamTypeFloat:
        ROS_INFO("%s (float) : %f" , paramName.c_str() ,  m_cf.getParam<float>(entry.id));
        break;
      }
    }

  std::unique_ptr<LogBlock<logImu> > logBlockImu;
  std::unique_ptr<LogBlock<logSim> > logBlock2;
  std::unique_ptr<LogBlock<logStats> > logStatsData;
  std::unique_ptr<LogBlock<logState> > logStateData;

  //std::vector<std::unique_ptr<LogBlockGeneric> > logBlocksGeneric(m_logBlocks.size());

  bool m_enableLogging = true;

  if (m_enableLogging) {

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

    if (true) {
      std::function<void(uint32_t, logImu*)> cb = onImuData; //std::bind(&CrazyflieROS::onImuData, this, std::placeholders::_1, std::placeholders::_2);

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

      // if (   false)
      // {
      //   std::function<void(uint32_t, logSim*)> cb2 = onLog2Data;// std::bind(&CrazyflieROS::onLog2Data, this, std::placeholders::_1, std::placeholders::_2);

      //   logBlock2.reset(new LogBlock<logSim>(
      //     &m_cf,{
      //       {"mag", "x"},
      //       {"mag", "y"},
      //       {"mag", "z"},
      //       {"baro", "temp"},
      //       {"baro", "pressure"},
      //       {"pm", "vbat"},
      //     }, cb2));
      //   logBlock2->start(10); // 100ms
      // }

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

      std::function<void(uint32_t, logState*)> cbQuadState = onLogStateData;//std::bind(&CrazyflieROS::onLogStateData, this, std::placeholders::_1, std::placeholders::_2);
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


    }

    ROS_INFO("Requesting memories...");
    // m_cf.requestMemoryToc();
    while(true);

  return 0;
}