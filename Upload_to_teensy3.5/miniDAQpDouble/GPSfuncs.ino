void timeCorrection(){
  if ((GPS.minute + 30) > 60) {
    lMinute = GPS.minute + 30-60;
    lHour = GPS.hour+6;
    lSec = GPS.seconds;
  }
  else{
    lHour = GPS.hour+5;
    lMinute = GPS.minute + 30;
    lSec = GPS.seconds;
  }
  GPSday = GPS.day;
  GPSmonth = GPS.month;
  GPSyear = GPS.year;
  GPSfix = GPS.fix;
  GPSquality = GPS.fixquality;
}


boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy



#ifdef __AVR__
// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  c = GPS.read();
  // if you want to debug, this is a good time to do it!
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;  
    // writing direct to UDR0 is much much faster than Serial.print 
    // but only one character can be written at a time. 
  #endif
}

void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}
#endif //#ifdef__AVR__

uint32_t timer = millis();

double degreeToM(double LatorLan){
  return LatorLan*PI/180.0*earthRadiusKm*10.0;
}

double MtoDegree(double in){
  return in/PI*180.0/earthRadiusKm/10.0;
}


void readGPS(){
  
  // in case you are not using the interrupt above, you'll
  // need to 'hand query' the GPS, not suggested :(
  if (! usingInterrupt) {
    // read data from the GPS in the 'main loop'
    c = GPS.read();
    // if you want to debug, this is a good time to do it!
    if (GPSECHO)
      if (c) Serial.print(c);
  }
  
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences! 
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
  
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }

  // if millis() or timer wraps around, we'll just reset it
  if (timer > millis())  timer = millis();

  // approximately every 50ms or so, print out the current stats
  if (millis() - timer > 100) { 
    
    
    timeCorrection();
    
    if (GPSfix) {
      GPSlatitude = degreeToM(GPS.latitude); GPSlat = GPS.lat;
      //Serial.println(GPS.latitude);
      GPSlongitude = degreeToM(GPS.longitude); GPSlon = GPS.lon;
      GPSspeed = GPS.speed*0.514444;
      GPSangle = GPS.angle;
      GPSaltitude = GPS.altitude;
      GPSsatellites = GPS.satellites; 
      GPSspeedN = (GPSlatitude-GPSlatitude1)/(millis() - timer)*1000;
      GPSspeedE = (GPSlongitude-GPSlongitude1)/(millis() - timer)*1000;
      //Serial.println(GPSspeedE);
      GPSlatitude1=GPSlatitude;
      GPSlongitude1=GPSlongitude;
    } 
    timer = millis(); // reset the timer
  
  
  }
  //else if (millis() - timer > 10) getOri();
}





