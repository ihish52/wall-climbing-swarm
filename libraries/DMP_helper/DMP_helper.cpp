//Based on I2Cdev's MPU6050 using DMP by Jeff Rowberg

#include "DMP_helper.h"
#include "MPU6050_6Axis_MotionApps20.h"

MPU6050 mpu;

void DMP_helper::DMP_setup(){
	
	uint8_t devStatus;
	mpu.initialize();
	MPU_connection = mpu.testConnection();
	devStatus = mpu.dmpInitialize();
	
	//Offsets
	mpu.setXGyroOffset(489);
	mpu.setYGyroOffset(129);
	mpu.setZGyroOffset(9);
	mpu.setXAccelOffset(1964);
	mpu.setYAccelOffset(974);
	mpu.setZAccelOffset(1646);
	
	if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        mpu.setDMPEnabled(true);
  }
}

void DMP_helper::ypr_pitch_bound(float &yaw, float &pitch, float &roll){
	uint8_t fifoBuffer[64];
	Quaternion q;// [w, x, y, z]
	if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        yaw = atan2(2.0*(q.x*q.y + q.w*q.z), q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z)*180/M_PI;
        pitch = asin(2.0*(q.x*q.z - q.w*q.y))*180/M_PI;
		roll = atan2(2.0*(q.y*q.z + q.w*q.x), q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z)*180/M_PI;
    }
}

