
void imu6050() {
  mpu6050.update();
  acc[0]=mpu6050.getAccX();
  acc[1]=mpu6050.getAccY();
  acc[2]=mpu6050.getAccZ();

  angle[0]=mpu6050.getAngleX();
  angle[1]=mpu6050.getAngleY();
  angle[2]=mpu6050.getAngleZ();

  acc_estimated[0]=KalmanFilter.updateEstimate(mpu6050.getAccX());
  acc_estimated[1]=KalmanFilter.updateEstimate(mpu6050.getAccY());
  acc_estimated[2]=KalmanFilter.updateEstimate(mpu6050.getAccZ());

  angle_estimated[0]=KalmanFilter.updateEstimate(mpu6050.getAngleX());
  angle_estimated[1]=KalmanFilter.updateEstimate(mpu6050.getAngleY());
  angle_estimated[2]=KalmanFilter.updateEstimate(mpu6050.getAngleZ());

  
}
