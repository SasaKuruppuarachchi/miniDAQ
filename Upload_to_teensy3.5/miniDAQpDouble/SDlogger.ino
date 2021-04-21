void getFileName(){
 
  sprintf(fileName, "%02d_%02d_%02d.kml", lHour,lMinute,lSec);
  //sprintf(fileName, "%d-%d-%d_%02d:%02d:%02d.txt", GPSyear,GPSmonth,GPSday,lHour,lMinute,lSec);
  sprintf(fileNameG, "%02dG%02dG%02d.csv", lHour,lMinute,lSec);
}


void logData(){
  if(GPS.fix==1) { //Only save data if we have a fix
  //  mySensorData = SD.open("NMEA.txt", FILE_WRITE); //Open file on SD card for writing
  //  mySensorData.println(NMEA1); //Write first NMEA to SD card
  //  mySensorData.println(NMEA2); //Write Second NMEA to SD card
  //  mySensorData.close();  //Close the file

    mySensorData = SD.open(fileNameG, FILE_WRITE);
    mySensorData.print(gACC[0]);mySensorData.print(",");mySensorData.println(gACC[1]);
    mySensorData.close();

    
    mySensorData = SD.open(fileName, FILE_WRITE);
 
    
    //in .kml longitude comes first
    degWholel = double(int(MtoDegree(estEpos)/100.0));
    degDecl=(MtoDegree(estEpos)-degWholel*100.0)/60.0;
    degl = degWholel + degDecl;
    if (GPSlon=='W') {  //If you are in Western Hemisphere, longitude degrees should be negative
     degl= (-1)*degl;
    }
    mySensorData.print(degl,10); //writing decimal degree longitude value to SD card
    mySensorData.print(","); //write comma to SD card
  
    /*Serial.print(degl,10); //writing decimal degree longitude value to SD card
    Serial.print(",");*/
  
    //then latitude
    degWhole = double(int(MtoDegree(estNpos)/100.0));
    degDec=(MtoDegree(estNpos)-degWhole*100.0)/60.0;
    deg=degWhole + degDec;
    if (GPS.lat=='S') {  //If you are in Southern hemisphere latitude should be negative
      deg= (-1)*deg;
    }
    mySensorData.print(deg,10); //writing decimal degree longitude value to SD card
    mySensorData.print(","); //write comma to SD card
  
    /*Serial.print(deg,10); //writing decimal degree longitude value to SD card
    Serial.print(",");*/
  
    
   //then altitude
    mySensorData.print(GPS.altitude); //write altitude to file
    mySensorData.print(" ");  //format with one white space to delimit data sets
  
    /*Serial.print(GPS.altitude); //write altitude to file
    Serial.print(" ");  //format with one white space to delimit data sets*/
   
    mySensorData.close();
    
  //  mySensorData.print(MtoDegree(estNpos),4); //Write measured latitude to file
  //  mySensorData.print(GPSlat); //Which hemisphere N or S
  //  mySensorData.print(",");
  //  mySensorData.print(MtoDegree(estEpos), 4); //Write measured longitude to file
  //  mySensorData.print(GPSlon); //Which Hemisphere E or W
  //  mySensorData.print(",");
  //  mySensorData.println(GPSaltitude);
  //  mySensorData.close();
  }
}

void initKML(){
  mySensorData = SD.open(fileName, FILE_WRITE);
  mySensorData.println("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n<kml xmlns=\"http://www.opengis.net/kml/2.2\">\n<Document>\n<Style id=\"yellowPoly\">\n<LineStyle>\n<color>7f00ffff</color>\n<width>4</width>\n</LineStyle>\n<PolyStyle>\n<color>7f00ff00</color>\n</PolyStyle>\n</Style>\n<Placemark><styleUrl>#yellowPoly</styleUrl>\n<LineString>\n<extrude>1</extrude>\n<tesselate>1</tesselate>\n<altitudeMode>clampToGround</altitudeMode>\n<coordinates>");
  //Serial.println("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n<kml xmlns=\"http://www.opengis.net/kml/2.2\">\n<Document>\n<Style id=\"yellowPoly\">\n<LineStyle>\n<color>7f00ffff</color>\n<width>4</width>\n</LineStyle>\n<PolyStyle>\n<color>7f00ff00</color>\n</PolyStyle>\n</Style>\n<Placemark><styleUrl>#yellowPoly</styleUrl>\n<LineString>\n<extrude>1</extrude>\n<tesselate>1</tesselate>\n<altitudeMode>clampToGround</altitudeMode>\n<coordinates>");
  mySensorData.close();
}
void closeKML(){
  mySensorData = SD.open(fileName, FILE_WRITE);
  mySensorData.println("</coordinates>\n</LineString></Placemark>\n</Document></kml>");
  mySensorData.close();
}

