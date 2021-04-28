#include "imu.h"

void imu::init()
{
    #if HAS_IMU == IMU_MPU9250
    int status = IMU.begin();

    #ifdef DEBUG
    if (status < 0)
    {
        Serial.println("IMU initialization unsuccessful");
        Serial.println("Check IMU wiring or try cycling power");
        Serial.print("Status: ");
        Serial.println(status);
        while (1);
    }
    #endif

    #elif HAS_IMU == IMU_LIS3DH
    IMU.settings.accelSampleRate = 50;  //Hz.  Can be: 0,1,10,25,50,100,200,400,1600,5000 Hz
    IMU.settings.accelRange = 2;      //Max G force readable.  Can be: 2, 4, 8, 16
    IMU.settings.adcEnabled = 0;
    IMU.settings.tempEnabled = 0;
    IMU.settings.xAccelEnabled = 1;
    IMU.settings.yAccelEnabled = 1;
    IMU.settings.zAccelEnabled = 1;

    status_t status = IMU.begin();

    #ifdef DEBUG
    switch (status) {
        case IMU_SUCCESS:
            Serial.println("IMU_SUCCESS");
            break;
	    case IMU_HW_ERROR:
            Serial.println("IMU_HW_ERROR");
            break;
        case IMU_NOT_SUPPORTED:
            Serial.println("IMU_NOT_SUPPORTED");
            break;
        case IMU_GENERIC_ERROR:
            Serial.println("IMU_GENERIC_ERROR");
            break;
        case IMU_OUT_OF_BOUNDS:
            Serial.println("IMU_OUT_OF_BOUNDS");
            break;
        case IMU_ALL_ONES_WARNING:
            Serial.println("IMU_ALL_ONES_WARNING");
            break;
    }
    #endif
    #endif
}

void imu::enableWakeup()
{
    #if HAS_IMU == IMU_MPU9250
    int imo_wom = IMU.enableWakeOnMotion(IMU_WAKEUP_FORCE, MPU9250::LP_ACCEL_ODR_15_63HZ);
    
    #ifdef DEBUG
    Serial.print("IMU: Wake on motion set: ");
    Serial.println(imo_wom);
    #endif

    #elif HAS_IMU == IMU_LIS3DH
    IMU.writeRegister(LIS3DH_INT1_CFG, 0x2a);
    IMU.writeRegister(LIS3DH_INT1_THS, 0x0a); 
    IMU.writeRegister(LIS3DH_INT1_DURATION, 0x01); 
    IMU.writeRegister(LIS3DH_CTRL_REG3, 0x20);
    #endif
}
