#include <Wire.h>
#include <VL53L0X.h>


//sends lidar data to webpage
void sendLidarData(uint8_t id, float out[]) {
  char tx[100] = "blank";
  if (sensor.timeoutOccurred() || sensor2.timeoutOccurred()) {
    sprintf(tx, "TIMEOUT");
  }
  else {
    //Maybe these are floats? idk
    int dist[2];
    // Measure and unbias
    out[0] = float(sensor.readRangeSingleMillimeters() ); //0 right
    out[1] = float(sensor2.readRangeSingleMillimeters() ); //1 front
    
    sprintf(tx, "L1 (%d); L2(%d)", out[0], out[1]);
  }
  wsSend(id, tx);
  Serial.write(tx);
}
