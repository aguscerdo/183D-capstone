char Ktext[200];

float X_est[3]; //X
float cov_est[9]; //P
float innovation[3]; //y 
float cov_innovation[9]; //S
float gain[9]; //K
float JF[9]; //Jacobian of f, xk = f(x_k-1, uk)
float JH[9]; //Jacobian of j, zk = h(xk)
float I[9]; //3x3 Identity
float Q[9]; // noise in f
float R[9]; // noise in h

float Box[4][2]; //TL TR BR BL

// TO DO: implement f, F, h, H. convert to matrices.

//all these functions part of h(.)
float distToLidarR(float dist){
  return dist; // should be system3 model
}
float distToLidarF(float dist){
  return dist; //should be system3 model
}
float angleToMagnetometer(float theta){
   return theta; // should be system3 model
}
float* zerosOfLine(float* p1, float* p2){
  // y = (x-p1.x) * (p2.y - p1.y)/(p2.x - p1.x) + p1.y
  // x = 0; y = 
  float y = (-p1[0]) * (p2[1] - p1[1])/(p2[0] - p1[0]) + p1[1]
  // x = (y-p1.y) * (p2.x - p1.x)/(p2.y - p1.y) + p1.x
  // y = 0; x = 
  float x = (-p1[1]) * (p2[0] - p1[0])/(p2[1] - p1[1]) + p1[0]
  //z[0] = x, z[1] = y
  float z[2];
  z[0] = x;
  z[1] = y;
  return z;
}

//Function f
float* f(float* X, float* U){
  return X;
}

//Function h
float* h(float* X){
  float newBox[4][2];
  float x = X[0];
  float y = X[1];
  float theta = X[2];
  // Translate all box points
  for (int i =  0; i < 4; i++){
    newBox[i][0] = Box[i][0] - x;
    newBox[i][1] = Box[i][1] - y;
  }
  // Rotate all points 
  // Rotation matrix R(-theta)
  float c = cos(theta);
  float s = cos(theta);
  float R[4];
  R[0] = c; R[1] = s;
  R[2] = -s; R[3] = c;
  for (int i =  0; i < 4; i++){
    newBox[i] = R*Box[i]
  }
  // Find zeros with y = 0 x > 0 and and x = 0  y > 0
  float z[2];
  float xDist = -1; //if they stay -1, something went wrong.
  float yDist = -1;
  for (int i = 0; i < 4; i++){
    z = zerosOfLine(newBox[i], newBox[(i+1) % 4] );
    if (z[1] > 0){
      yDist = z[1];
    }
    if (z[0] > 0){
      xDist = z[0];
    }
  }
  float newState[3];
  newState[0] = distToLidarR(xDist);
  newState[1] = distToLidarF(yDist);
  newState[2] = angleToMagnetometer(theta);
  
  return newState;
}

//Jacobian of f
float* F(float* X, float* U){
  return X;
}

//Jacobian of h
float* H(float* X){
  
  return X;
}

//
void kalman(float pwmL, float pwmR, float lidarF, float lidarR, float mag) {
  //create and set U, z, F
  float U[2];
  float z[3];
  U[0] = pwmL;
  U[1] = pwmR;
  z[0] = lidarF;
  z[1] = lidarR;
  z[2] = mag;
  //get JF
  JF = F(X_est, U);
  //predict
  X_est = f(X_est, U);
  cov_est = JF*cov_est*JF.T+Q;
  //get JH
  JH = H(X_est);
  //update
  innovation = z - h(X_est); //get innovation
  cov_innovatoin = H*cov_est*H.T+R; //get cov of innovation
  gain = cov_est*H.T*inv(S); //get gain
  //new estimates
  X_est = X_est+gain*innovation;
  cov_est = (I-gain*H)*cov_est;
  
  
  sprintf(Ktext, "Kalman: (x,y,theta)=(%f,%f,%f)", X_est[0], X_est[0], X_est[0]);
  Serial.println(Ktext)
}
