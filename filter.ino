#include <SparkFunMPU9250-DMP.h>
#include <Wire.h>
#define PI 3.1415926535897932384626433832795
#include <Servo.h>

// Declaring IMU and SERVO objects

MPU9250_DMP imu;
Servo servo;

// Declaring kalman filter variables


const float error_per=0.03; //increase it for bigger value difference
const int loop_dur=30;  //increase for better filtering | decrease for responsiveness
float est_error=0;
float est=0;
float pv=1e-4; //increase it for lag reducing
float kg=0;
float xp=0;
float zp=0;
float p=1;
static int i=0;


// Setup needed for SERVO and IMU objects

void setup() {
  
  Serial.begin(115200);
  
  if (imu.begin() != INV_SUCCESS)
  {
    while (1)
    {
        // Failed to initialize MPU-9250, loop forever
    }
    imu.setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);
    imu.setGyroFSR(2000); 
    imu.setAccelFSR(16);
  }

  servo.attach(11);
  servo.write(90);
  

}

// LOOP that is executed every clock pulse in the microcrontroller (Arduino UNO)

void loop() {

  // Reading the data from the IMU unit 
  
  imu.update(UPDATE_ACCEL | UPDATE_GYRO | UPDATE_COMPASS);

  float accelX = imu.calcAccel(imu.ax); // accelX is x-axis acceleration in g's
  float accelY = imu.calcAccel(imu.ay); // accelY is y-axis acceleration in g's
  float accelZ = imu.calcAccel(imu.az); // accelZ is z-axis acceleration in g's
  
  float gyroX = imu.calcGyro(imu.gx); // gyroX is x-axis rotation in dps
  float gyroY = imu.calcGyro(imu.gy); // gyroY is y-axis rotation in dps
  float gyroZ = imu.calcGyro(imu.gz); // gyroZ is z-axis rotation in dps
  
  float magX = imu.calcMag(imu.mx); // magX is x-axis magnetic field in uT
  float magY = imu.calcMag(imu.my); // magY is y-axis magnetic field in uT
  float magZ = imu.calcMag(imu.mz); // magZ is z-axis magnetic field in uT

  // filtering the data from the IMU unit

  float selected_mea = accelX;
  
  float accel_Y = filter(accelY);
  
  
  float estimate = filter(selected_mea);

  print_mea_and_est(selected_mea,estimate);

  // moving the servomotor according to the filtered data from the IMU unit
  

  //servo.write(90+90*accel_Y); // for top wing
  //servo.write(90-90*accel_Y); // for back wing
  //servo.write(90+90*estimate); // for right wing
  //servo.write(90-90*estimate); // for left wing
  
  //servo.write(90+3*gyroZ*PI/180);  //Anti-roll

}


float filter(float mea)
{
  float mea_error=abs(mea*error_per);

   for(int i=0; i<loop_dur; i++)
  {
      est_error=p+pv;
      kg=est_error/(est_error+mea_error);
      p=(1-kg)*est_error;
      xp=est;
      zp=xp;
      est=kg*(mea-zp)+xp;
      
  }

  return est;
}

void print_mea_and_est(float mea,float est)
{
  Serial.print("measurement: ");
  Serial.print(mea);
  Serial.print(" ");
  Serial.print("  est: ");
  Serial.println(est);
  //Serial.println("");
}
