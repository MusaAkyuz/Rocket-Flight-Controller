/*
 * Musa Akyüz 
 * 20.02.2021
 * 12:00AM
 * For Control Rocket Flying And Learn Pressure,Tempature,Altitude
 * 
 */
#include <SFE_BMP180.h> //Pressure and temprature Sensor Library
#include <MPU6050.h>   //6-Axis acceleration and Gyro Sensor Library
#include <Servo.h>    //Servo Motor Library
#include <Wire.h>    //Protocol that allows Arduino to communicate with microprocessors on sensors (for BMP180 AND MPU6050)
#include <SPI.h>    //Other comminication protocol Library (for SD Card)
#include <SD.h>    //SD Card Library

double getPressure(); //Function protocol
void setup();
void loop();

MPU6050 MPU;  //MPU6050 called MPU
SFE_BMP180 BMP;  //BMP180 called BMP
Servo myservoY, myservoX;  //Servo called myservoY and myservoX

//BMP180 VARİABLE
double baseline;  //baseline --> Starting pressure
double P,a;  //P --> instant pressure   a --> instant altitude

//MPU6050 VARİABLE
int16_t ax, ay, az;
int16_t gx, gy, gz;
int valY;  //myservoY angle variable
int prevValY;  //myservoY previous angle variable
int valX;  //myservoX angle variable
int prevValX;  //myservoX previous angle variable

//SD CARD CREATE FİLE
File myFile;

void setup() 
{
  Wire.begin();  //Starting comminication 12C Protocol
  Serial.begin(9600);  //Opens serial port, sets data rate to 9600 bps
  Serial.println("------------");
  
  //BMP180 CONNECTİON CHECK
  Serial.print("Initializing BMP...");
  Serial.println(BMP.begin() ? "Connected" : "Connection Failed");
  baseline = getPressure();  //Starting pressure
  
  //SD CARD CONNECTİON CHECK
  Serial.print("Initializing SD CARD...");
  Serial.println(SD.begin(4) ? "Connected" : "Connection Failed");
  
  //MPU CARD CONNECTİON CHECK
  Serial.print("Initializing MPU...");
  MPU.initialize();
  Serial.println(MPU.testConnection() ? "Connected" : "Connection Failed");
  
  //SERVO DEFİNED PİN
  myservoY.attach(2);
  myservoX.attach(3);
  
}//end of setup function

void loop() 
{
  //BMP180 PROCESSİNG
  P = getPressure(); //Instant pressure 
  a = BMP.altitude(P,baseline); //Instant altitude based on starting pressure and instantaneous pressure

  //MPU6050 PROCESSİNG 
  MPU.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  /*
   * How to use map function in arduino default
   * Syntax
   * map(value, fromLow, fromHigh, toLow, toHigh)
   * 
   * Parameters
   * value: the number to map.
   * fromLow: the lower bound of the value’s current range.
   * fromHigh: the upper bound of the value’s current range.
   * toLow: the lower bound of the value’s target range.
   * toHigh: the upper bound of the value’s target range.
   */
  valY = map(ay, -17000, 17000, 0, 179);
  valX = map(ax, -17000, 17000, 0, 179);
  
  if (valY != prevValY)
  {
    myservoY.write(valY);
    prevValY = valY;
  }

  if (valX != prevValX)
  {
    myservoX.write(valX);
    prevValX = valX;
  }
  delay(10);

  Serial.print("  Altitude : ");
  Serial.print(a);
  Serial.print("  meter");
  Serial.print(" Pressuure : ");
  Serial.print(P);
  Serial.println(" mb");
  Serial.print("  X Value : ");
  Serial.print(ax);
  Serial.print("  Y Value : ");
  Serial.print(ay);
  Serial.print("  Z Value : ");
  Serial.println(az);
  Serial.println("-------------------");

  myFile = SD.open("test.txt", FILE_WRITE);

  if (myFile) {
    myFile.print("Altitude : ");
    myFile.print(a);
    myFile.print("meter");
    myFile.print(" Pressuure : ");
    myFile.print(P);
    myFile.println(" mb");
    myFile.print("  X DEĞERİ : ");
    myFile.print(ax);
    myFile.print("  Y DEĞERİ : ");
    myFile.print(ay);
    myFile.print("  Z DEĞERİ : ");
    myFile.println(az);
    Serial.println("-------------------");
    myFile.close();
  }
}

/*
 * LOOK BMP180 LIBRARY FİLE
 * C:\Program Files (x86)\Arduino\libraries\BMP180_Breakout_Arduino_Library-master\src
 */
double getPressure()//getPressure function
{
  char status;
  double T,P;
  
  /*
   * startTempature function in library info
   * Begin a temperature reading.
   * Will return delay in ms to wait, or 0 if I2C error
   */
  status = BMP.startTemperature();
  if (status != 0) 
  {
    delay(status); 

    /*
     * getTempature function in library info
     * Retrieve a previously-started temperature reading.
     * Requires begin() to be called once prior to retrieve calibration parameters.
     * Requires startTemperature() to have been called prior and sufficient time elapsed.
     * T: external variable to hold result.
     * Returns 1 if successful, 0 if I2C error.
     */
    status = BMP.getTemperature(T); 
    if (status != 0)
    {

      /*
       * startPressure function in library info
       * Begin a pressure reading.
       * Oversampling: 0 to 3, higher numbers are slower, higher-res outputs.
       * Will return delay in ms to wait, or 0 if I2C error.
       */
      status = BMP.startPressure(3);
      if (status != 0)
      { 
        delay(status);

        /*
         * getPressure function in library info
         * Retrieve a previously started pressure reading, calculate abolute pressure in mbars.
         * Requires begin() to be called once prior to retrieve calibration parameters.
         * Requires startPressure() to have been called prior and sufficient time elapsed.
         * Requires recent temperature reading to accurately calculate pressure.
         * P: external variable to hold pressure.
         * T: previously-calculated temperature.
         * Returns 1 for success, 0 for I2C error.
         * Note that calculated pressure value is absolute mbars, to compensate for altitude call sealevel().
         */
        status = BMP.getPressure(P,T);
        if (status != 0)
        {
          return(P);
        }
        else Serial.println("Error in pressure measurement received");
      }
      else Serial.println("Pressure measurement failed to start");
    }
    else Serial.println("Could not get temperature value");
  }
  else Serial.println("Temperature measurement failed to start ");
}//end of getPressure function
