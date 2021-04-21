void loop() {
    
    getOri();
    gMap();
    TimeNowSec = millis()/1000.0;
    //***********************Kalman*****************
    if (GPSfix) {
      kE.predict(accE,TimeNowSec);
      kN.predict(accN,TimeNowSec);
      

      // if millis() or timer wraps around, we'll just reset it
      if (timer2 > millis())  timer2 = millis();
      
      if (millis() - timer2 > 100){
        timer2 = millis();
        kE.updates(GPSlongitude,GPSspeedE);
        kN.updates(GPSlatitude,GPSspeedN);
        
        
       // Serial.println("*");
      }
      estEpos = kE.getPosition();
      estEvel = kE.getVelocity();
      //Matrix.Print((double*)kE.currentState, 2, 1, "CTE");
      
      estNpos = kN.getPosition();
      estNvel = kN.getVelocity();
      //Matrix.Print((double*)kN.currentState, 2, 1, "CTN");
   
      
    }
    LBSprev = logButtonState;
    logButton.update();
    logButtonState  = logButton.read();

    if (logButtonState > LBSprev) {
        getFileName();
        Serial.println(fileName);
        initKML();
    } else if (logButtonState < LBSprev) {
        Serial.println("down");
        closeKML();
    }
    if ( logButtonState == HIGH ) {
      logData();
    } 
    
    display.clearDisplay();
    GPSdisplay();
    AccDisplay();
    display.display();
         
    delay(10);
}

void IMUthread(){
  while(1){
    //GPSserial();
    //IMUserial();  
    //Serial.println("********************");
    threads.delay(10);
  }
}


void GPSthread(){

  while(1){
    
    readGPS(); 
    //threads.delay(100);
  }
}

