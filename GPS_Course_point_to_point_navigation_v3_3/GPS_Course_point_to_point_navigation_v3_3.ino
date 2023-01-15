#include <Wire.h>
#include <HMC5883L.h>
 #include <SoftwareSerial.h>                                         // Software Serial for Serial Communications - not used
#include <TinyGPS++.h>
#include <SD.h>
#include <SPI.h>


#define RXPin 0
#define TXPin 1
#define GPSBaud 9600
#define ConsoleBaud 9600

// The serial connection to the GPS device
int Number_of_SATS;
int headingerror;
int headingerror_1;
////////////////////////////////////Macros/////////////////////////////////////
#define RIGHT_DIRECTION  21
#define LEFT_DIRECTION  22
#define STOP 0
#define SPEED_INCREMENT 20
#define PULSE_INCREMT 21

#define DC_FROWARDS  10
#define DC_BACKWARDS 11

#define UP 1
#define DOWN 2

#define STEP_ANGLE 1.8
#define STEPPING_MODE 16
#define STEPPER_UPPER_LIMIT 6400
#define STEPPER_LOWER_LIMIT -6400

#define COMPASS_ANGLE_DEVIATION_DEGREES 5
 int COMPASS_STEERING_INCRMENT = 100;


/////////////////////////////////////Pins/////////////////////////////////////
//const uint8_t IntPIN_right_dir = 2;
//const uint8_t IntPIN_left_dir = 3;

const uint8_t STEPPER_pulse_pin = 6;
const uint8_t STEPPER_dir_pin = 7;

const uint8_t DC_pin_1 = 4;
const uint8_t DC_pin_2 = 5;
const uint8_t DC_pin_pwm = 3;

int CS_PIN = 53;
/////////////////////////////////////Vars/////////////////////////////////////
uint8_t stepping_Steering_direction = STOP;
uint8_t G_motor_speed = 200;
uint32_t G_pulse_duration = 500;
char  control_char =0 ;
int G_stepper_posistion = 0;
int G_stepper_desitnation = 0;
uint8_t G_runnung_interrupt_flag=0;
float G_Heading_Degrees = 0;
int angle_diff=0;

uint16_t G_destination_angle = 0;
uint8_t G_steering_flag = 0;
int flag_stop=0;
int desired_heading;
int Heading_A;
int Heading_B;
int heading_error;
float Distance_To_Home;
// GPS Variables & Setup

String dataString;
String dataString2;
String dataString3;

String dataString4;

String dataString5;


//////////////////////////////////////functions //////////////////////////////
void Timer_1_INIT();
void Stepper_vRun(float angle, uint8_t Steering_direction);
void Compass_vUpdate_angle(void);
void DC_vMove(uint8_t Steering_direction, uint8_t speed);
void Stepper_vStop(void);
void Car_vTurn (float angle, uint8_t Steering_direction);
void Stepepr_vHome(void);
void getGPS(void);
//void setHeading(void);
//void gpsInfo(void);
//void Startup(void);
void SlowLeftTurn(void );
void SlowRightTurn(void );
void Startup();
//void goWaypoint(float Home_LATarray,float Home_LONarray);

//////////////////////////////////Objects/////////////////////////////////
SoftwareSerial ss(RXPin, TXPin);

// The TinyGPS++ object
TinyGPSPlus gps;
unsigned long lastUpdateTime = 0;

HMC5883L compass;


#define MAX_HEADING_ANGLE  180
#define MIN_HEADING_ANGLE  5
#define ANGLE_RANGE_DIV 0.25



#define des_LAT 29.957848
#define des_LNG 30.956845
/* This example shows a basic framework for how you might
   use course and distance to guide a person (or a drone)
   to a destination.  This destination is the Eiffel Tower.
   Change it as required.

   The easiest way to get the lat/long coordinate is to
   right-click the destination in Google Maps (maps.google.com),
   and choose "What's here?".  This puts the exact values in the
   search box.
*/

void setup()
{
   Serial.begin(ConsoleBaud);
  ss.begin(GPSBaud);

  pinMode(STEPPER_pulse_pin,OUTPUT);
  pinMode(STEPPER_dir_pin,OUTPUT);
  digitalWrite(STEPPER_dir_pin,HIGH);
  digitalWrite(STEPPER_pulse_pin,LOW);

  delay(1000);

Serial.println("Initializing SD card...");
  pinMode(CS_PIN, OUTPUT);
  if (SD.begin())
  {
    Serial.println("SD card is ready to use.");
  } else
  {
    Serial.println("SD card initialization failed");
    return;
  }

  Serial.println("Initialize HMC5883L");
  while (!compass.begin())
  {
    Serial.println("Could not find a valid HMC5883L sensor, check wiring!");
    delay(500);
  }

  // Set measurement range
  compass.setRange(HMC5883L_RANGE_1_3GA);

  // Set measurement mode
  compass.setMeasurementMode(HMC5883L_CONTINOUS);

  // Set data rate
  compass.setDataRate(HMC5883L_DATARATE_30HZ);

  // Set number of samples averaged
  compass.setSamples(HMC5883L_SAMPLES_8);

  // Set calibration offset. See HMC5883L_calibration.ino
  compass.setOffset(74, -124);
File dataFile = SD.open("GpsData7.txt", FILE_WRITE);
if (dataFile) {

    dataFile.print("%");
    dataFile.println();
    dataFile.close();
              }

  //Read the current headnign angle
  Compass_vUpdate_angle();

  Timer_1_INIT();


  Startup() ;
}

void loop()
{
 // If any characters have arrived from the GPS,
  // send them to the TinyGPS++ object
  while (ss.available() > 0)
    gps.encode(ss.read());

    // Establish our current status
  double  distanceToDestination = TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), des_LAT, des_LNG);
   double  courseToDestination = TinyGPSPlus::courseTo(gps.location.lat(), gps.location.lng(), des_LAT, des_LNG);
    const char *directionToDestination = TinyGPSPlus::cardinal(courseToDestination);
    int courseChangeNeeded = (int)(360 + courseToDestination - gps.course.deg()) % 360;

    // debug
 /*   Serial.print("DEBUG: Course2Dest: ");
    Serial.print(courseToDestination);
    Serial.print("  CurCourse: ");
    Serial.print(gps.course.deg());
    Serial.print("  Dir2Dest: ");
    Serial.print(directionToDestination);
    Serial.print("  RelCourse: ");
    Serial.print(courseChangeNeeded);
    Serial.print("  CurSpd: ");
    Serial.println(gps.speed.kmph());*/

    float l = gps.location.lat();
    float ln = gps.location.lng();

    Serial.println();
    Serial.print("Lat: "); Serial.print(l,6); Serial.print("  Lon: "); Serial.println(gps.location.lng());
    Serial.print("current Angle: "); Serial.println(atan2(gps.location.lat(), gps.location.lng())*180/M_PI);

   heading_error=courseChangeNeeded-G_Heading_Degrees;
   if (heading_error < -180)
    {
    heading_error = heading_error + 360;
    }
  if (heading_error > 180)
    {
    heading_error = heading_error - 360;
    }

    dataString = String(l, 6);
    dataString2 = String(ln, 6);
    dataString3 = String(distanceToDestination, 2);
    dataString4= String(heading_error);

  // Every 5 seconds, do an update.

  File dataFile = SD.open("GpsData8.txt", FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) {
    dataFile.print(dataString);
    dataFile.print(" , ");
    dataFile.print(dataString2);
    dataFile.print(" , ");
    dataFile.print(dataString3);
    dataFile.print(" , ");
    dataFile.print(dataString4);
    dataFile.println();
    dataFile.close();
              }
    lastUpdateTime = millis();

    // Within 20 meters of destination?  We're here!
   if (distanceToDestination <= 10.0)
    { Stepper_vStop();
      DC_vMove(STOP,STOP);
      Serial.println("CONGRATULATIONS: You've arrived!");
      exit(1);
    }

    Serial.print("DISTANCE: ");
    Serial.print(distanceToDestination);
    Serial.println(" meters to go.");
    Serial.print("INSTRUCTION: ");

    // Standing still? Just indicate which direction to go.
   /*if (gps.speed.kmph() < 2.0)
    {
      Serial.print("Head ");
      Serial.print(directionToDestination);
      Serial.println(".");
      //return;
    }
else
{*/
Serial.print("G_Heading_Degrees=  ");
Serial.println(G_Heading_Degrees);
Serial.print("heading_error =  ");
Serial.println(heading_error );
    if (heading_error > 10 && heading_error <= 180)
      {//Serial.println("Keep on right!");
      COMPASS_STEERING_INCRMENT=60 ;
      Car_vTurn (heading_error, RIGHT_DIRECTION);
      Stepepr_vHome();

      }
    else if (heading_error <-10 && heading_error >= -180)
     { //Serial.println("Veer slightly to the left.");
        COMPASS_STEERING_INCRMENT=60 ;
        Car_vTurn (heading_error, LEFT_DIRECTION);
        Stepepr_vHome();

     }
   else if (heading_error < 10 && heading_error >= -10)
   {
    DC_vMove(DC_FROWARDS,G_motor_speed);
    Serial.println("FORWARDDDD");
   }


    else
      { Serial.println("nothing");

    }

  }
