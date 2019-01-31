#include <Wire.h>
#include <VL53L0X.h>

float m_bias[2] = {-1, -58.500};
float m_scale[2] = {23.0, 24.5};

//Bias: -1.000000 58.500000
//Scale: 23.000000 24.500000


//sends magnetometer data to webpage
float sendMagnetometerData(uint8_t id) {
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
  //headingDegrees -= 198;
  if (headingDegrees < 0) headingDegrees += 360;
  if (headingDegrees > 360) headingDegrees -= 360;

  //Might need to adjust to lower size(?)
  char tx[80] = "blank";
  sprintf(tx, "Mx,My: (%d, %d). Deg: (%f)\0", mx, my, headingDegrees);
  Serial.write(tx);
  wsSend(id, tx);

  return headingDegrees;
}


// ---------------------------------------------------------------------------



//calibration for magnetometer
void magcalMPU9250()
{
 uint16_t ii = 0, sample_count = 0;
 int16_t mag_max[3] = {-10000, -10000}, mag_min[3] = {10000, 10000}, mag_temp[2] = {0, 0};

 Serial.println("Mag Calibration: Wave device in a figure eight until done!");

 sample_count = 400;
  uint8_t ST1;
  LED_ON;
 for(ii = 0; ii < sample_count; ii++) {
  I2CwriteByte(MAG_ADDRESS, 0x0A, 0x01);


  do
  {
    I2Cread(MAG_ADDRESS, 0x02, 1, &ST1);
  }
  while (!(ST1 & 0x01));
  uint8_t Mag[7];

  I2Cread(MAG_ADDRESS, 0x03, 7, Mag);

  mag_temp[0] = (Mag[1] << 8 | Mag[0]);
  mag_temp[1] = (Mag[3] << 8 | Mag[2]);

    if(mag_temp[0] > mag_max[0]){
      mag_max[0] = mag_temp[0];
    }
    if(mag_temp[0] < mag_min[0]){
      mag_min[0] = mag_temp[0];
    }    
    
    if(mag_temp[1] > mag_max[1]){
      mag_max[1] = mag_temp[1];
    }
    if(mag_temp[1] < mag_min[1]){
      mag_min[1] = mag_temp[1];
    }
    delay(25);
 }
 LED_OFF;

// Get hard iron correction
 m_bias[0]  = float(mag_max[0] + mag_min[0])/2.0;  // get average x mag bias in counts
 m_bias[1]  = float(mag_max[1] + mag_min[1])/2.0;  // get average y mag bias in counts

// Get soft iron correction estimate
 m_scale[0]  = float(mag_max[0] - mag_min[0])/2.0;  // get average x axis max chord length in counts
 m_scale[1]  = float(mag_max[1] - mag_min[1])/2.0;  // get average y axis max chord length in counts


 char text[200];
 sprintf(text, "Ms %d %d %d %d", mag_max[0], mag_min[0], mag_max[1], mag_min[1]);
 Serial.println(text);
 sprintf(text, "Bias: %f %f", m_bias[0], m_bias[1]);
 Serial.println(text);
 sprintf(text, "Scale: %f %f", m_scale[0], m_scale[1]);
 Serial.println(text);
 Serial.println("---------------------------------------------");
}
