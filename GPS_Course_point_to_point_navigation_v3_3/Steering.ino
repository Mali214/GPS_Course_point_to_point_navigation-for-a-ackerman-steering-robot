void Compass_vUpdate_angle(void)
{
  Vector norm = compass.readNormalize();

  // Calculate heading
  float heading = atan2(norm.YAxis, norm.XAxis);

  // Set declination angle on your location and fix heading
  // You can find your declination on: http://magnetic-declination.com/
  // (+) Positive or (-) for negative
  // For Bytom / Poland declination angle is 4'26E (positive)
  // Formula: (deg + (min / 60.0)) / (180 / M_PI);
  float declinationAngle = (4.0 + (26.0 / 60.0)) / (180 / M_PI);
  heading += declinationAngle;

  // Correct for heading < 0deg and heading > 360deg
  if (heading < 0)
  {
    heading += 2 * PI;
  }

  if (heading > 2 * PI)
  {
    heading -= 2 * PI;
  }

  // Convert to degrees
  G_Heading_Degrees = heading * 180/M_PI;

  // Output
  //Serial.print(" Heading = ");
  //Serial.print(heading);
 // Serial.print(" Degress = ");
  //Serial.print(G_Heading_Degrees);
  //Serial.println();

  //delay(100);
}


void Car_vTurn (float angle, uint8_t Steering_direction)
{
  int angle_diffrence ;
  Compass_vUpdate_angle();
  getGPS();
  //Serial.print("->>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>= ");
      //Serial.println(G_Heading_Degrees);
  if (G_steering_flag == 0)
  {
    G_steering_flag = 1;
    if(Steering_direction == RIGHT_DIRECTION)
    {
      G_destination_angle = G_Heading_Degrees +angle ;
      if( G_destination_angle>360)
       {
        G_destination_angle=G_destination_angle-360;
       }
     }




    if(Steering_direction == LEFT_DIRECTION)
    {
      G_destination_angle = G_Heading_Degrees - angle  ;
      if( G_destination_angle<0)
       {
        G_destination_angle=G_destination_angle+360;
       }


    }
    Stepper_vRun(COMPASS_STEERING_INCRMENT, Steering_direction);
      delay(1000);



  }



  while (1)
  {
    angle_diff = G_destination_angle-G_Heading_Degrees;
    angle_diffrence =abs(angle_diff);
    Compass_vUpdate_angle();
    getGPS();
  
    //Serial.print("turining=="); Serial.println(Steering_direction);
    // Serial.println(angle_diffrence);
    if (angle_diffrence >= COMPASS_ANGLE_DEVIATION_DEGREES)
    {
        DC_vMove(DC_FROWARDS, G_motor_speed);
    }
    else if (angle_diffrence < COMPASS_ANGLE_DEVIATION_DEGREES)
    {

      break;
      }
    }
    Serial.print("angle_diffrence==");
    Serial.print(angle_diffrence);
            G_steering_flag = 0;


    }
    /*-------------------------------------------------------------------------------------------------------*/
   void DC_vMove(uint8_t Steering_direction, uint8_t speed)
{
  if(Steering_direction == DC_FROWARDS)
  {
    digitalWrite(DC_pin_1,HIGH);
    digitalWrite(DC_pin_2,LOW);
    analogWrite(DC_pin_pwm,speed);
  }
  else if(Steering_direction == DC_BACKWARDS)
  {
    digitalWrite(DC_pin_1,LOW);
    digitalWrite(DC_pin_2,HIGH);
    analogWrite(DC_pin_pwm,speed);
  }
  else if(Steering_direction == STOP)
  {
    digitalWrite(DC_pin_1,HIGH);
    digitalWrite(DC_pin_2,HIGH);
    analogWrite(DC_pin_pwm,255);
  }
  else
  {
    digitalWrite(DC_pin_1,LOW);
    digitalWrite(DC_pin_2,LOW);
    analogWrite(DC_pin_pwm,speed);
  }
}
void getGPS(void)                                                 // Get Latest GPS coordinates
{
    while (Serial.available() > 0)
    gps.encode(Serial.read());
}

void SlowLeftTurn(void)
{
 COMPASS_STEERING_INCRMENT=50 ;

Car_vTurn (10, LEFT_DIRECTION);

 }
 void SlowRightTurn(void)
{
 COMPASS_STEERING_INCRMENT=30;

Car_vTurn (10, RIGHT_DIRECTION);

 }
 void Startup()
{
                  
     for (int i=5; i >= 1; i--)                       // Count down for X seconds
      {         
        Serial.print("Pause for Startup... "); 
        Serial.print(i);
        delay(1000);                                   // Delay for X seconds
      }    
    
  Serial.println("Searching for Satellites "); 
  Serial.println("Searching for Satellites "); 
      
  while (Number_of_SATS <= 3)                         // Wait until x number of satellites are acquired before starting main loop
  {                                  
    getGPS();                                         // Update gps data
    Number_of_SATS = (int)(gps.satellites.value());   // Query Tiny GPS for the number of Satellites Acquired       
    
  }    
                                        // set intial waypoint to current location
 
  Serial.print(Number_of_SATS);
  Serial.print(" Satellites Acquired");    
}
