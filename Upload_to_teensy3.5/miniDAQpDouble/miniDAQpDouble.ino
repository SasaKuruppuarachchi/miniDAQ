//***********************************************SD*****************************
#include <SD.h> //Load SD card library
#include<SPI.h> //Load SPI Library
File mySensorData; //Data object you will write your sesnor data to

const int chipSelect = BUILTIN_SDCARD;

char fileName[] = "00_00_00.kml";

char fileNameG[] = "00G00G00.kml";

double degWhole,deg,degDec;
double degWholel,degl,degDecl;

//getFileName();
//***********************************************Button*************************
#include <Bounce2.h>

#define BUTTON_PIN 24
Bounce logButton = Bounce();
volatile int logButtonState = 0;

int LBSprev = 0;
 
//************************************************GPS****************************

#include <Adafruit_GPS.h>    //Install the adafruit GPS library
#define mySerial Serial3

Adafruit_GPS GPS(&mySerial); //Create the GPS Object

#define GPSECHO  false

String NMEA1; //Variable for first NMEA sentence
String NMEA2; //Variable for second NMEA sentence
volatile char c; //to read characters coming from the GPS

volatile int lHour,lMinute,lSec,GPSday,GPSmonth,GPSyear,GPSquality,GPSsatellites;
volatile bool GPSfix = 0;
volatile double GPSlatitude = 721226.3;
volatile double GPSlongitude = 8853627.2;  //home
volatile double GPSspeed,GPSangle,GPSaltitude,GPSspeedN,GPSspeedE;
volatile char GPSlat,GPSlon;
volatile double GPSlatitude1 = 721226.3;
volatile double GPSlongitude1 = 8853627.0; 

#define earthRadiusKm 6378.0

//************************************************IMU****************************
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (1)
Adafruit_BNO055 bno = Adafruit_BNO055(55);

/* Also send calibration data for each sensor. */
uint8_t sys, gyro, accel, mag = 0;
volatile double roll,pitch,yaw;
volatile double gDot[2];  //acceleration in g in Y & X
volatile double gACC[2];
volatile int Nx,Ny;
volatile double accN,accE; //acceleration in N & E

#define BNO055_SAMPLERATE_DELAY_MS (1)
//************************************************LCD****************************

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// If using software SPI, define CLK and MOSI
#define OLED_CLK 13
#define OLED_MOSI 11

// These are neede for both hardware & softare SPI
#define OLED_CS 5
#define OLED_RESET 6
#define OLED_DC 2


Adafruit_SSD1306 display(OLED_DC, OLED_RESET, OLED_CS);

//*************************************************threds*******************************
#include <TeensyThreads.h>
Threads::Mutex Serial_lock;

int GPSthreadID, IMUthreadID;
double timeNowSec;

//#include <MsTimer2.h> 


//**************************************************Kalman*********************************
#include <MatrixMath.h>

volatile double posCov = 1.1;
volatile double accCov = 0.08;  //GPS and ACC covariances
volatile double estNpos,estEpos,estNvel,estEvel;

volatile double timeStamp;
volatile int timer2;

class Kalman {
  public:

  //predict
  double currentState[2][1]; // state vector [ p  
                            //                v ]
  double u;                  //accelerometer reading
  double P[2][2];            //estimate error (initialize identity)
  double Q[2][2];            //Accelerometer noice covarience (ms-2)2
  double A[2][2];            //state transition metrix. Predicts next step with no external inputs
  double B[2][1];            //Control metrix. adds the influence of external influence
  double currentTimeStampSec;
  
  //double preState[2][1];
  
  //update
  double z[2][1];            //GPS reading for pos, vel
  double H[2][2];            //transform pred. in form of GPS reading (identity as degrees are converted to m in API for simplicity)
  double R[2][2];            //GPS error covarience +/- m
  double K[2][2];            //Kalman gain

  double u1[2][1];
  double zi[2][1];
  double AT[2][2];
  double s[2][2];
  int err;
  
  //double estState[2][1];

  public:

  Kalman(double initialPos,double posCov, double accCov, double currentTimeStamp) //constructor
  :currentTimeStampSec(currentTimeStamp){
    
    currentState[0][0] = initialPos;
    currentState[1][1] = 0.0;

    Q[0][0] = accCov;
    Q[1][1] = accCov;

    R[0][0] = posCov;
    R[1][1] = posCov;

    Serial.println(currentTimeStampSec);
    Matrix.Print((double*)currentState, 2, 1, "C");
 
  }
  
  void recreateA(double deltaSec){                             //update state transition metrix
    A[0][0] = 1.0; A[0][1] = deltaSec;
    A[1][0] = 0.0; A[1][1] = 1.0;
     
  }

  void recreateB(double deltaSec){                            //update control metrix
    B[0][0] = 0.5*deltaSec*deltaSec; 
    B[1][0] = deltaSec; 
 
  }

  void predict(double accThisAxis, double timeStampNow){        //predict step
    double deltaT = timeStampNow - currentTimeStampSec;
    recreateA(deltaT);
    recreateB(deltaT);
    u = accThisAxis;
    
    //preState = A*currentState + B*u
    
    Matrix.Multiply((double*)A, (double*)currentState, 2, 2, 1, (double*)u1);
    Matrix.Scale((double*) B, 2, 1, u);
    Matrix.Add((double*) u1, (double*) B, 2, 1, (double*) currentState);
    
    
    //P = A*P*AT + Q
    Matrix.Multiply((double*)A, (double*)P, 2, 2, 2, (double*)P);
    Matrix.Transpose((double*)A, 2, 2, (double*)AT);
    Matrix.Multiply((double*)P, (double*)AT, 2, 2, 2, (double*)P);
    Matrix.Add((double*) P, (double*) Q, 2, 2, (double*) P);
    
    currentTimeStampSec = timeStampNow;
  }

  void updates(double pos, double velocity){                  // update step
    z[0][0] = pos;
    z[1][0] = velocity;

    

    //kalman gain K = P*(P+R)-1
    
    Matrix.Add((double*) P, (double*) R, 2, 2, (double*) s);
    err = Matrix.Invert((double*) s, 2);
    if (err == 0)Serial.println("No inverse");
    Matrix.Multiply((double*)P, (double*)s, 2, 2, 2, (double*)K);

    //estState = preState + K( z - preState)
    Matrix.Subtract((double*) z, (double*) currentState, 2, 1, (double*) z);
    Matrix.Multiply((double*)K, (double*)z, 2, 2, 1, (double*)zi);
    Matrix.Add((double*) currentState, (double*) zi, 2, 1, (double*) currentState);

    // P = P - KP
    double Ps[2][2];
    Matrix.Multiply((double*)K, (double*)P, 2, 2, 2, (double*)Ps);
    Matrix.Subtract((double*) P, (double*) Ps, 2, 1, (double*) P);

    

    
  }

  double getPosition(){
    return currentState[0][0];
  }

  double getVelocity(){
    return currentState[1][0];
  }
};

Kalman kE(GPSlongitude,posCov,accCov,millis()/1000.0);
Kalman kN(GPSlatitude,posCov,accCov,millis()/1000.0);

double TimeNowSec;


