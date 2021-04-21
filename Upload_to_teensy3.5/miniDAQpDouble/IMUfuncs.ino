void gMap(void){
  //Threads::Scope scope(wire_lock);
  
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  // Possible vector values can be:
  // - VECTOR_ACCELEROMETER - m/s^2
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - rad/s
  // - VECTOR_EULER         - degrees
  // - VECTOR_LINEARACCEL   - m/s^2
  // - VECTOR_GRAVITY       - m/s^2
 
  gACC[0]= euler.x()/9.81;
  gACC[1]= euler.y()/9.81;    //acceleration in g in Y & X

  accN = euler.x()*sin(roll* PI / 180.0) + euler.y()*cos(roll* PI / 180.0);
  accE = euler.x()*cos(roll* PI / 180.0) - euler.y()*sin(roll* PI / 180.0);  //acceleration in N & E
  //Serial.println(euler.x()); //got out raw values to calculate covariance
  //Serial.println(accE);
}

void getOri(void){

  bno.getCalibration(&sys, &gyro, &accel, &mag);
  sensors_event_t event;
  bno.getEvent(&event);

  roll = (double)event.orientation.x;
  pitch = (double)event.orientation.y;
  yaw = (double)event.orientation.z;

  Nx = 99 + 29*sin(roll* PI / 180.0);
  Ny = 34 + 29*cos(roll* PI / 180.0);
  
}
