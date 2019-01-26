#include <Wire.h>
#include <VL53L0X.h>


//sends lidar data to webpage
void sendLidarData(uint8_t id) {
  /*Serial.print("Lidar 1 range(mm): ");
    Serial.print(sensor.readRangeSingleMillimeters());
    if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }

    Serial.print("  Lidar 2 range(mm): ");
    Serial.println(sensor2.readRangeSingleMillimeters());
    if (sensor.timeoutOccurred()) { Serial.println(" TIMEOUT"); }*/
  char tx[100] = "blank";
  if (sensor.timeoutOccurred() || sensor2.timeoutOccurred()) {
    sprintf(tx, "TIMEOUT");
  }
  else {
    //Maybe these are floats? idk
    int dist[2];
    // Measure and unbias
    dist[0] = sensor.readRangeSingleMillimeters() - 22;
    dist[1] = sensor2.readRangeSingleMillimeters() - 47;
    
    sprintf(tx, "L1 (%d); L2(%d)", dist[0], dist[1]);
  }
  wsSend(id, tx);
  Serial.write(tx);
}
