void GPSserial() {

  Serial.print("\n\nTime: ");
  Serial.print(lHour, DEC); Serial.print(':');
  Serial.print(lMinute, DEC); Serial.print(':');
  Serial.print(lSec, DEC); Serial.print('.');
  Serial.print("Date: ");
  Serial.print(GPSday, DEC); Serial.print('/');
  Serial.print(GPSmonth, DEC); Serial.print("/20");
  Serial.println(GPSyear, DEC);
  Serial.print("Fix: "); Serial.print((int)GPSfix);
  Serial.print(" quality: "); Serial.println((int)GPSquality); 
  
  Serial.println(usingInterrupt);

  if (GPSfix) {
    Serial.print("Location: ");
    Serial.print(GPSlatitude, 8); Serial.print(GPSlat);
    Serial.print(", "); 
    Serial.print(GPSlongitude, 8); Serial.println(GPSlon);
    
    Serial.print("Speed (ms-1): "); Serial.println(GPSspeed);
    Serial.print("Angle: "); Serial.println(GPSangle);
    Serial.print("Altitude: "); Serial.println(GPSaltitude);
    Serial.print("Satellites: "); Serial.println((int)GPSsatellites);Serial.println();
  }
}


void IMUserial(){
  //****************************Acc**********************************
  Serial.print("X: ");
  Serial.println(abs(gDot[0]));
  Serial.print("Y: ");
  Serial.println(abs(gDot[1]));

  //***************************ori************************************


  Serial.print(F("Orientation: "));
  Serial.print(roll);
  Serial.print(F(" "));
  Serial.print(pitch);
  Serial.print(F(" "));
  Serial.print(yaw);
  Serial.println(F(""));
}



