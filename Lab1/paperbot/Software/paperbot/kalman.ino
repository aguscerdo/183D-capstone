

float X = 0;
float Y = 0;
float theta = 0;
char Ktext[200];

void kalman(float pwmL, float pwmR, float lidarF, float lidarR, float mag) {




  sprintf(Ktext, "Kalman: %f %f %f", X, Y, theta);
  Serial.println(Ktext)
}
