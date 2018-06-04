#include <iostream>
#include <stdio.h>

#include <functional>

#include "crazyflie_cpp/Crazyradio.h"
#include "crazyflie_cpp/crtp.h"
#include "crazyflie_cpp/Crazyflie.h"

#include <fstream>

#include <thread>
#include <chrono>

#define TESTS_DURATION 14400.0 // 4 hours
#define FILEPATH "sensors_data.txt"

std::ofstream mData;

struct logStats {
	uint16_t rxRate;
	uint16_t rxDrpRte;
	uint16_t txRate;
} __attribute__((packed));

void onLogStatData(uint32_t time_in_ms, logStats* data){
  printf("rxRate = %d | rxDrpRte = %d | txRate = %d\n", (int) data->rxRate, (int) data->rxDrpRte, (int) data->txRate);
}

void onImuReceived(const crtpImuExpDataResponse* imu){
	static long number_data = 0;
	mData << (int) imu->ax << "," << (int) imu->ay << "," << (int) imu->az << ",";
	mData << (int) imu->gx << "," << (int) imu->gy << "," << (int) imu->gz << ",";
	mData << (int) imu->elapsed_time << std::endl;
	if (number_data % 100000 == 0){
		std::cout << "Data : " << number_data << std::endl; 
	}
	number_data++;

	// mData << (int) imu->headingx << "," << (int) imu->headingy << "," << (int) imu->headingz << ",";
	// mData << (int) imu->pressure << "," << (int) imu->elapsed_time << std::endl;
}


int main(int argc, char **argv)
{
	//std::string uri = "radio://0/80/2M";

	mData.open(FILEPATH , std::ios::out | std::ios::trunc);
	if(! mData.is_open()){
		std::cout << "Can't create and open file !" << std::endl;
		return 0;
	}

	std::string uri = "usb://0";
	Crazyflie m_cf(uri , EmptyLogger);

	try {
		m_cf.requestParamToc();
	} catch (std::runtime_error& e){
		printf("Runtime Error catched : %s\n" , e.what());
	}


	/*m_cf.logReset();
	m_cf.requestLogToc();

	std::unique_ptr<LogBlock<logStats> > logStatsData;
	std::function<void(uint32_t, logStats*)> cbStats = std::bind(&onLogStatData, std::placeholders::_1, std::placeholders::_2);
	logStatsData.reset(new LogBlock<logStats>(
		&m_cf,{
			{"crtp", "rxRate"},
			{"crtp", "rxDrpRte"},
			{"crtp", "txRate"},
		}, cbStats));
	logStatsData->start(200);*/

	std::function<void(const crtpImuExpDataResponse*)> cbImu = std::bind(&onImuReceived , std::placeholders::_1);
	m_cf.setImuExpDataCallback(cbImu);

	auto start = std::chrono::system_clock::now();
	while(((std::chrono::duration<double>) (std::chrono::system_clock::now() - start)).count() < TESTS_DURATION){
		m_cf.sendPing();
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
	}
	mData.close();
	m_cf.reboot();
}