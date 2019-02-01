float t = 0.1; //time; t = 1/10 -> 10 times every sec
char Ktext[200];
float z_est[3]; //z_est given X_est
//float X_est[3] = { 325, 250, 0}; //X_est
float cov_est[9]; //P
float innovation[3]; //y 
float cov_innovation[9]; //S
float gain[9]; //K
float JF[9]; //Jacobian of f, xk = f(x_k-1, uk)
float JH[9]; //Jacobian of j, zk = h(xk)
float I[9] = {1,0,0,0,1,0,0,0,1}; //3x3 Identity

float Q[9] = {0.25*t,0,0,0,0.25*t,0,0,0,0.89}; // noise in f
float R[9] = {3.28,0,0,0,1.86,0,0,0,0.8}; // noise in h

float tmp0[9]; // temp/intermediary
float tmp1[9]; // temp/intermediary
float tmp2[9]; // temp/intermediary
float tmp3[9]; // temp/intermediary
float pi = 3.1415;
MatrixMath M;

//float Box[4][2] = {{0, 500}, {750, 500}, {750, 0}, {0, 0}}; //TL TR BR BL
float pen = 145;
float Box[4][2] = {{0, 5.5*pen}, {2*pen, 5.5*pen}, {2*pen, 0}, {0, 0}}; //TL TR BR BL
float X_est[3] = { 1*pen, 2.25*pen, 0}; //X_est

float radToDeg(float rad) {
  rad = rad * 180 / pi;
  while (rad > 360.)
    rad -= 360.;
  while (rad < 0)
    rad += 360.;

  return rad;
}

float degToRad(float deg) {
  deg = deg * pi / 180;
  return deg;
}



// TO DO: implement f, F, h, H. convert to matrices.

//all these functions part of h(.)
float distToLidarR(float dist){
  return 0.951*dist+47.4; // should be system3 model
}
float distToLidarF(float dist){
  return 0.983*dist+22.4; //should be system3 model
}
float angleToMagnetometer(float theta){
  // TODO wtf is this?
  float tmp = (134*atan(degToRad(theta-200))+374);
    while (tmp > 360) {
    tmp -= 360;
  }
  while (tmp < 0) {
    tmp += 360; 
  }
   return  tmp; // should be system3 model
}
//takes in p1, p2, gives zeros (z1,0), (0,z2)
void zerosOfLine(float* p1, float* p2, float* z){
  // y = (x-p1.x) * (p2.y - p1.y)/(p2.x - p1.x) + p1.y
  // x = 0; y = 
  float y = (-p1[0]) * (p2[1] - p1[1])/(p2[0] - p1[0]) + p1[1];
  // x = (y-p1.y) * (p2.x - p1.x)/(p2.y - p1.y) + p1.x
  // y = 0; x = 
  float x = (-p1[1]) * (p2[0] - p1[0])/(p2[1] - p1[1]) + p1[0];
  //z[0] = x, z[1] = y
  z[0] = x;
  z[1] = y;
  return ;
}

//Function f, takes in X, U and updates X
void stateUpdate(float* X, float* U){
  X[2] = (X[2] + ((t/84) * ((11.05 * pow((U[1] - 90), 0.2) + 0.69) - (10.72 * pow((U[0] - 90), 0.2) - 1.49))) );
  while (X[2] > 360) {
    X[2] -= 360;
  }
  while (X[2] < 0) {
    X[2] += 360; 
  }
  X[0] = X[0] + ((t/2) * cos(degToRad(X[2])) * ((10.72 * pow((U[0] - 90), 0.2) - 1.49) + (11.05 * pow((U[1] - 90), 0.2) + 0.69)));
  X[1] = X[1] + ((t/2) * sin(degToRad(X[2])) * ((10.72 * pow((U[0] - 90), 0.2) - 1.49) + (11.05 * pow((U[1] - 90), 0.2) + 0.69)));

  return ;
}

//Function h, takes in state X and returns sensorData
//Also gives JH, jacobian of h.
void stateToOutput(float* X, float* sensorData, float* JacH){
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
  // Rotation M. R(-theta)
  float c = cos(degToRad(theta));
  float s = cos(degToRad(theta));
  float R[4];
  R[0] = c; R[1] = s;
  R[2] = -s; R[3] = c;
  for (int i =  0; i < 4; i++){
    newBox[i][0] = R[0]*Box[i][0] + R[1]*Box[i][1];
    newBox[i][1] = R[2]*Box[i][0] + R[3]*Box[i][1];
  }
  // Find zeros with y = 0 x > 0 and and x = 0  y > 0
  float z[2];
  float xDist = -1; //if they stay -1, something went wrong.
  float yDist = -1;
  // TODO is this eq right? Check units
  float dMdtheta = 134*180*pi/(pow(pi,2) * pow((theta-200),2) +32400);
  for (int i = 0; i < 4; i++){
    zerosOfLine(newBox[i], newBox[(i+1) % 4], z);
    if (z[1] > 0){
      yDist = z[1];
      //JacH[3] = dLF/dx = dy/dx for this line * derivative of distToLidarF;
      //this line: y = yDist + ax and fits point newBox[i]
      float a = (newBox[i][1] - yDist) / newBox[i][0];
      JacH[3] = a * 0.951;
      //JacH[5] = dLF/dtheta = 

      // TODO is this equation right
      JacH[5] =  -pi/pow(sin(pi*theta/180),2) / (180*pow(a+1/tan(degToRad(theta)),2)) ;// *dMdtheta;
    }
    if (z[0] > 0){
      xDist = z[0];
      //JacH[1] = dLR/dy = dx/dy for this line * derivative of distToLidarR; 
      //this line: x = xDist + by and fits point newBox[i]
      //b = (x1-xDist) / y1
      float b = (newBox[i][0] - xDist) / newBox[i][1];
      JacH[1] = b * 0.983;
       //JacH[2] = dLR/dtheta = 
      // TODO check nums
      JacH[2] =  -pi/pow(cos(degToRad(theta)),2) / (180*pow((b+tan(degToRad(theta))),2)) ;// *dMdtheta;
    }
  }
  sensorData[0] = distToLidarR(xDist);
  sensorData[1] = distToLidarF(yDist);
  sensorData[2] = angleToMagnetometer(theta);
// M = magnetometer, LF = lidar front, LR = lidar right
  JacH[6] = 0; //dM/dx
  JacH[7] = 0;  //dM/dy
  JacH[8] = dMdtheta; // dM/dtheta = derivative of angleToMagnetometer()

  // TODO are these meant to be hard coded
  JacH[0] = 0.983; // dLR/dx = derivative of distToLidarR()
  JacH[4] = 0.951; // dLF/dy = derivative of distToLidarF()
  return ;
}

//Jacobian of f, takes in X, U returns Jacobian JacF
void jacobianStateUpdate(float* X, float* U, float* JacF){
  JF[0] = 1; //dx/dx
  JF[1] = 0; //dx/dy
  JF[2] = 0; //dx/dtheta
  JF[3] = 0; //dy/dx
  JF[4] = 1; //dy/dy
  JF[5] = 0; //dy/dtheta

  // TODO check units
  JF[6] = (-t) * sin(degToRad(X[2])) * pi/180*((5.36 * pow((U[0] - 90), 0.2)) + (5.42 * pow((U[1] - 90), 0.2)) - 0.398); //dtheta/dx
  JF[7] = (t) * cos(degToRad(X[2])) * pi/180*((5.36 * pow((U[0] - 90), 0.2)) + (5.42 * pow((U[1] - 90), 0.2)) - 0.398); //dtheta/dy
  JF[8] = 1; //dtheta/dtheta
  return ;
}


//
void kalman(float pwmL, float pwmR, float lidarF, float lidarR, float mag, int id) {
  //create and set U, z, F
  float U[2];
  float z[3];
  U[0] = pwmL;
  U[1] = pwmR;
  z[0] = lidarR;
  z[1] = lidarF;
  z[2] = mag;
  //get JF, X
  jacobianStateUpdate(X_est, U, JF);
  stateUpdate(X_est, U);
  //predict
  //cov_est = JF*cov_est*JF.T+Q;
  M.Multiply(JF, cov_est, 3,3,3, tmp0);
  M.Transpose(JF,3,3, tmp1);
  M.Multiply(tmp0, tmp1, 3,3,3, tmp2);
  M.Add(tmp2, Q, 3,3, cov_est);
  //get JH, h   //H(X_est, JH);//h(X_est, z_est);
  stateToOutput(X_est, z_est, JH);
  //update
  //innovation = z - z_est; 
  M.Subtract(z, z_est, 3,1, innovation);
  //cov_innovation = JH*cov_est*JH.T+R; 
  M.Multiply(JH, cov_est, 3,3,3, tmp0);
  M.Transpose(JH,3,3, tmp1);
  M.Multiply(tmp0, tmp1, 3,3,3, tmp2);
  M.Add(tmp2, R, 3,3, cov_innovation);
  //gain = cov_est*JH.T*inv(cov_innovation); 
  M.Multiply(cov_est, tmp1, 3,3,3, tmp0);
  //Make sure inverse exists, if it fails try again but with small epsilon away
  int inv = 0;
  M.Copy(cov_innovation, 3,3, tmp1);
  inv = M.Invert(tmp1, 3);
  float epsilon = 0.001;
  while (inv == 0){
      M.Copy(cov_innovation, 3,3, tmp1);
      M.Copy(I, 3, 3, tmp2);
      M.Scale(tmp2, 3, 3, epsilon);
      M.Add(tmp1, tmp2, 3, 3, tmp3);
      M.Copy(tmp3, 3, 3, tmp1);
      inv = M.Invert(tmp1, 3);
      epsilon = epsilon/2;
  }
  M.Multiply(tmp0, tmp1, 3,3,3, gain);
  //new estimates
  //X_est = X_est+gain*innovation;
  M.Multiply(gain, innovation, 3,3,1, tmp0);
  M.Copy(X_est, 3,1, tmp1);
  M.Add(tmp1, tmp0, 3, 1, X_est);
  //cov_est = (I-gain*JH)*cov_est;
  M.Multiply(gain, JH, 3,3,3, tmp0);
  M.Subtract(I, tmp0, 3, 3, tmp1);
  M.Copy(cov_est, 3,3, tmp2);
  M.Multiply(tmp1, tmp2, 3,3,3, cov_est);
  
  sprintf(Ktext, "Kalman: (x,y,theta)=(%f,%f,%f), y=(Lr,Lf,M)=(%f,%f,%f)", X_est[0], X_est[1], X_est[2], innovation[0], innovation[1], innovation [2] );
  wsSend(id, Ktext);
}
