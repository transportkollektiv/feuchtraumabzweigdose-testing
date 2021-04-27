#include "imu.h"

void imu::init()
{
    status = IMU.begin();
    
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
}

void imu::enableWakeup()
{
    int imo_wom = IMU.enableWakeOnMotion(IMU_WAKEUP_FORCE, MPU9250::LP_ACCEL_ODR_15_63HZ);
    
    #ifdef DEBUG
    Serial.print("IMU: Wake on motion set: ");
    Serial.println(imo_wom);
    #endif
}
