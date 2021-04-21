void setup() {

  GPSthreadID = threads.addThread(GPSthread);
  //IMUthreadID = threads.addThread(IMUthread);
  threads.setSliceMicros(10);
  
  Serial.begin(115200); //Turn on serial monitor
  introDisplay();
  setupGPS();
  setupSD();
  setupIMU();
  setupButton();
  
  getOri();
  
  readGPS();
  //setupKalman();
  
}

void setupButton(){
  // Setup the button with an internal pull-up :
  pinMode(BUTTON_PIN,INPUT_PULLUP);

  // After setting up the button, setup the Bounce instance :
  logButton.attach(BUTTON_PIN);
  logButton.interval(5); // interval in ms
}

void setupKalman(){
  
  Kalman kN(GPSlatitude,posCov,accCov,double(millis())/1000.0);
  Kalman kE(GPSlongitude,posCov,accCov,double(millis())/1000.0);
  
}

void setupGPS(){
  GPS.begin(9600); //Turn on GPS at 9600 baud
  mySerial.begin(9600);
  GPS.sendCommand("$PGCMD,33,0*6D");  //Turn off antenna update nuisance data
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); //Request RMC and GGA Sentences only
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ); //Set update rate to 10 hz
  GPS.sendCommand(PMTK_API_SET_FIX_CTL_5HZ);
  delay(1000); 
}


void setupSD(){
  pinMode(10, OUTPUT); //Must declare 10 an output and reserve it to keep SD card happy
  SD.begin(chipSelect); //Initialize the SD card reader
  
  if (SD.exists("NMEA.txt")) { //Delete old data files to start fresh
    SD.remove("NMEA.txt");
  }
  if (SD.exists(fileName)) { //Delete old data files to start fresh
    SD.remove(fileName);
  }
}


void setupIMU(){
  //Threads::Scope scope(wire_lock);
  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  /* Use external crystal for better accuracy */
  bno.setExtCrystalUse(true);
   
  /* Display some basic information on this sensor */
  displaySensorDetails();
}

