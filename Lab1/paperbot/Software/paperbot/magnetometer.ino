#include <Wire.h>
#include <VL53L0X.h>

float m_bias[2] = {6.0, 81.0};
float m_scale[2] = {40.0, 41.0};

//sends magnetometer data to webpage
void sendMagnetometerData(uint8_t id) {
  // Request first magnetometer single measurement
  I2CwriteByte(MAG_ADDRESS, 0x0A, 0x01);

  uint8_t ST1;
  uint8_t ii = 0;
  do
  {
    I2Cread(MAG_ADDRESS, 0x02, 1, &ST1);
  }
  while (!(ST1 & 0x01));

  // Read magnetometer data
  uint8_t Mag[7];
  I2Cread(MAG_ADDRESS, 0x03, 7, Mag);

  // Create 16 bits values from 8 bits data
  // Magnetometer
  int16_t mx = (Mag[1] << 8 | Mag[0]);
  int16_t my = (Mag[3] << 8 | Mag[2]);

  float mag[2];
  mag[0] = (mx - m_bias[0])/m_scale[0];
  mag[1] = (my - m_bias[1])/m_scale[1];

  float heading = atan2(mag[0], mag[1]);

//  float declinationAngle = 0.0404;
//  heading += declinationAngle;

  // Correct for when signs are reversed.
  if (heading < 0)
    heading += 2 * PI;

  // Check for wrap due to addition of declination.
  if (heading > 2 * PI)
    heading -= 2 * PI;

  // Convert radians to degrees for readability.
  float headingDegrees = heading * 180 / PI;
  headingDegrees -= 198;
  if (headingDegrees < 0) headingDegrees += 360;
  if (headingDegrees > 360) headingDegrees -= 360;

  //Might need to adjust to lower size(?)
  char tx[80] = "blank";
  sprintf(tx, "Mx,My: (%d, %d). Deg: (%f)\0", mx, my, headingDegrees);
  Serial.write(tx);
  wsSend(id, tx);
}


// ---------------------------------------------------------------------------


//
////calibration for magnetometer
//void magcalMPU9250(float * dest1, float * dest2)
//{
// uint16_t ii = 0, sample_count = 0;
// int32_t mag_bias[3] = {0, 0, 0}, mag_scale[3] = {0, 0, 0};
// int16_t mag_max[3] = {-32767, -32767, -32767}, mag_min[3] = {32767, 32767, 32767}, mag_temp[3] = {0, 0, 0};
// float MPU9250mRes = 10.*4912./32760.;
//
// Serial.println("Mag Calibration: Wave device in a figure eight until done!");
// delay(4000);
//
// sample_count = 400;
//  uint8_t ST1;
//  uint8_t Mag[7];
// for(ii = 0; ii < sample_count; ii++) {
//  do
//  {
//    I2Cread(MAG_ADDRESS, 0x02, 1, &ST1);
//  }
//  while (!(ST1 & 0x01));
//  I2Cread(MAG_ADDRESS, 0x03, 7, Mag);
//
//  mag_temp[0] = (Mag[1] << 8 | Mag[0]);
//  mag_temp[1] = (Mag[3] << 8 | Mag[2]);
//  mag_temp[2] = (Mag[5] << 8 | Mag[4]);
//
//  for (int jj = 0; jj < 3; jj++) {
//    if(mag_temp[jj] > mag_max[jj]){
//      mag_max[jj] = mag_temp[jj];
//    }
//    if(mag_temp[jj] < mag_min[jj]){
//      mag_min[jj] = mag_temp[jj];
//    }
//  }
//  delay(10);
// }
//
//// Get hard iron correction
// mag_bias[0]  = (mag_max[0] + mag_min[0])/2;  // get average x mag bias in counts
// mag_bias[1]  = (mag_max[1] + mag_min[1])/2;  // get average y mag bias in counts
// mag_bias[2]  = (mag_max[2] + mag_min[2])/2;  // get average z mag bias in counts
//
// float MPU9250magCalibration[3];
// MPU9250magCalibration[0] = 1;
// MPU9250magCalibration[1] = 1;
// MPU9250magCalibration[2] = 1;
//
// dest1[0] = (float) mag_bias[0]//*MPU9250mRes*MPU9250magCalibration[0];  // save mag biases in G for main program
// dest1[1] = (float) mag_bias[1]//*MPU9250mRes*MPU9250magCalibration[1];
// dest1[2] = (float) mag_bias[2]//*MPU9250mRes*MPU9250magCalibration[2];
//
//// Get soft iron correction estimate
// mag_scale[0]  = (mag_max[0] - mag_min[0]+2)/2;  // get average x axis max chord length in counts
// mag_scale[1]  = (mag_max[1] - mag_min[1]+2)/2;  // get average y axis max chord length in counts
// mag_scale[2]  = (mag_max[2] - mag_min[2]+2)/2;  // get average z axis max chord length in counts
///*
// float avg_rad = (mag_scale[0] + mag_scale[1] + mag_scale[2]) / 3;
// Serial.println(avg_rad);
// Serial.println(mag_max[0]);
// Serial.println(mag_min[0]);
// */
// dest2[0] = ((float)mag_scale[0]);
// dest2[1] = ((float)mag_scale[1]);
// dest2[2] = ((float)mag_scale[2]);
//
// Serial.println("CALIBRATION VALUES!!!! (mx-d2[0])/d1[0] for accurate value, scale to unit sphere");
// char text[200];
// sprintf(text, "Bias: %f %f %f", dest1[0], dest1[1], dest1[2]);
// Serial.println(text);
// sprintf(text, "Scale: %f %f %f", dest2[0], dest2[1], dest2[2]);
// Serial.println(text);
//
// Serial.println("---------------------------------------------");
//}
//
