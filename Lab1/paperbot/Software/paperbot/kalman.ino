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

// TO DO: implement f, F, h, H. convert to matrices.

//Function f
float* f(float* X, float* U){
  return X;
}

//Function h
float* h(float* X){
  return X;
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
