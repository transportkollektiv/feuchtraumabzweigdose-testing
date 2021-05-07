#ifndef IMU_H
#define IMU_H

#include <HardwareSerial.h>
#include "config.h"


#if HAS_IMU == IMU_MPU9250
#include <MPU9250.h>
#elif HAS_IMU == IMU_LIS3DH
#include <SparkFunLIS3DH.h>
#else
#error "IMU not supported"
#endif


class imu
{
    public:
        void init ();
        void enableWakeup ();
        #if HAS_IMU == IMU_MPU9250
        MPU9250 IMU = MPU9250(SPI, IMU_NCS);
        #elif HAS_IMU == IMU_LIS3DH
        LIS3DH IMU = LIS3DH(SPI_MODE, IMU_NCS);
        #endif

    private:
        
};

#endif //IMU_H