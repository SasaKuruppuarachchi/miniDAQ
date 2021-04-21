void introDisplay(){
  display.begin();
  display.fillScreen(BLACK);
  display.display();

  display.setCursor(0, 0);
  display.setTextColor(WHITE);
  display.setTextSize(2);
  display.println("TeamSHARK");
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.println("Racing");
  display.println();display.println("UOM");
  display.println("(C)SKcreations");
  display.display();

}

void GPSdisplay(){
  //gMap();
  
  timeCorrection();
  if ((int)GPSfix >=1){
    display.drawFastVLine(1,2,2, WHITE);
    display.drawFastVLine(2,2,2, WHITE);
    display.drawFastVLine(3,1,3, WHITE);
    display.drawFastVLine(4,1,3, WHITE);}
  if ((int)GPSfix >= 2){
    display.drawFastVLine(5,0,4, WHITE);
    display.drawFastVLine(6,0,4, WHITE);}

  if (sys>0)display.fillCircle(12, 2,2, WHITE); //Calibration status
  else display.drawCircle(12, 2,2, WHITE);
  
  if ( logButtonState == HIGH ) display.fillCircle(20, 2,2, WHITE); //Logging status
  else display.drawCircle(20, 2,2, WHITE);
  
  display.setCursor(80, 0);
  
  display.print("");
  display.print(lHour, DEC); display.print(':');
  display.print(lMinute, DEC); display.print(':');
  display.println(lSec, DEC);
  display.setCursor(0, 6);
  
  if (GPSfix) {
  //  display.print(estNpos/1000.0, 4); display.print("Km ");display.println(GPSlat);
  //  display.print(estEpos/1000.0, 4); display.print("Km ");display.println(GPSlon);
  
    display.print(MtoDegree(estNpos), 4);display.println(GPSlat);
    display.print(MtoDegree(estEpos), 4);display.println(GPSlon);
    
    display.print("Vn : "); display.println(estNvel);
    display.print("Ve : "); display.println(estEvel);
  }
}

void AccDisplay(){
  //***************************************************GMAP display************************************
        display.drawRect(70, 5, 58, 58, WHITE);
        //display.fillRect(70, 5, 58, 58, WHITE);
        display.drawFastVLine(99, 5, 58, WHITE );
        display.drawFastHLine(70, 34, 58, WHITE );

        
        display.setCursor(0, 40);
        display.setTextColor(BLACK, WHITE);
        display.println("      G-Map");
        display.setTextColor(WHITE);
        display.print("X: ");
        display.println(abs(accN));
        display.print("Y: ");
        display.print(abs(accE));
      
        
      
        gDot[0]= map(gACC[0], -0.9,0.9 ,70,128);
        gDot[1]= map(gACC[1], -0.9,0.9 ,63,5);

        
        
        display.fillCircle(gDot[0], gDot[1], 3, WHITE);

        //North comapss
        display.drawLine(99, 34, Nx, Ny, WHITE);
}

void OriDisplay(){
  //-------------------------------------------disply OLED------------------------
  
  
  
  display.setCursor(0, 0);
  display.setTextColor(WHITE);
  display.setTextSize(1.5);
  display.setTextColor(BLACK, WHITE);
  display.println(F("Orientation:"));
  display.setTextColor(WHITE);
  
  
  display.setTextSize(1);
  display.print(F("X - "));
  display.println(roll);
  display.print(F("Y - "));
  display.println(pitch);
  display.print(F("Z - "));
  display.println(yaw);
  display.println();
  
  display.setCursor(120, 0);
  display.print(sys, DEC);
}



void displaySensorDetails(void)
{
  sensor_t sensor;
  bno.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
  Serial.println("------------------------------------");
  Serial.println("");

  display.fillScreen(BLACK);
  display.display();

  display.setCursor(0, 0);
  display.println("---------------------");
  display.print  ("Sensor:       "); display.println(sensor.name);
  display.print  ("Driver Ver:   "); display.println(sensor.version);
  display.print  ("Unique ID:    "); display.println(sensor.sensor_id);
  display.print  ("Max Value:    "); display.print(sensor.max_value); display.println(" xx");
  display.print  ("Min Value:    "); display.print(sensor.min_value); display.println(" xx");
  display.print  ("Resolution:   "); display.print(sensor.resolution); display.println(" xx");
  display.println("----------------------");
  display.println("");
  display.display();
  
  delay(2000);
}

