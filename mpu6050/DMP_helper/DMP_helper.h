//Based on I2Cdev's MPU6050 using DMP by Jeff Rowberg

class DMP_helper{

	public:
		void DMP_setup();
		void ypr_pitch_bound(float &yaw, float &pitch, float &roll);
		
	private:
		bool MPU_connection = false;
		
};
