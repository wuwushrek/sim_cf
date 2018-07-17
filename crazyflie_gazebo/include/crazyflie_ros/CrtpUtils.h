#ifndef __CRTPUTILS_H___
#define __CRTPUTILS_H___

#include <iostream>
#include <cstdint>
#include "crazyflie_cpp/crtp.h"

#define CRTP_PORT_SETPOINT_SIM 0x09

// Conversion coefficient for Crazyflie
#define MAG_GAUSS_PER_LSB                                 666.7
#define SENSORS_DEG_PER_LSB_CFG                           ((2 * 2000.0) / 65536.0)
#define SENSORS_G_PER_LSB_CFG                             ((2 * 16) / 65536.0)

#define PI_CF 3.14159265359
#define DEG_TO_RAD_CF (PI_CF/180.0)
#define RAD_TO_DEG_CF (180.0/PI_CF)

#define GRAVITY_MAGNITUDE_CF (9.81) // we use the magnitude such that the sign/direction is explicit in calculations

#define PWM2OMEGA(pwm) ((pwm) < (1000) ? (0) : ((0.04076521f*pwm) + 380.8359f))

// Sensor type (first byte of crtp packet)
enum SensorTypeSim_e {
  SENSOR_GYRO_ACC_SIM           = 0,
  SENSOR_MAG_SIM                = 1,
  SENSOR_BARO_SIM               = 2,
  SENSOR_POS_SIM				= 3,
};

union Axis3i16 {
	struct {
		int16_t x;
		int16_t y;
		int16_t z;
	};
	int16_t axis[3];
}__attribute__((packed));
CHECKSIZE_RESPONSE(Axis3i16)

union Axis3f {
	struct {
		float x;
		float y;
		float z;
	};
	float axis[3];
}__attribute__((packed));
CHECKSIZE_RESPONSE(Axis3f)

struct baro_s {
	const crtp header;
	uint8_t type;
	float pressure;           // mbar
	float temperature;        // degree Celcius
	float asl;                // m (ASL = altitude above sea level)
} __attribute__((packed));
CHECKSIZE_RESPONSE(baro_s)

struct imu_s {
	const crtp header;
	uint8_t type;
	union Axis3i16 acc;
	union Axis3i16 gyro;
} __attribute__((packed));
CHECKSIZE_RESPONSE(imu_s)

struct mag_s {
	const crtp header;
	uint8_t type;
	union Axis3i16 mag;
} __attribute__((packed));
CHECKSIZE_RESPONSE(mag_s)

struct pos_s {
	const crtp header;
	uint8_t type;
	union Axis3f pos;
} __attribute__((packed));
CHECKSIZE_RESPONSE(pos_s)
#endif //__CRTPUTILS_H___