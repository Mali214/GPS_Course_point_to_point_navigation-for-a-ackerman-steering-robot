void Stepper_vRun(float angle, uint8_t Steering_direction)
{
  if (G_runnung_interrupt_flag == 0 )
  {
    double stepper_pulses=0;
    stepper_pulses = (angle/STEP_ANGLE)*(STEPPING_MODE);
    //////Serial.print("pulses="); //////Serial.println(stepper_pulses);
    if(Steering_direction == LEFT_DIRECTION)
    {
      stepping_Steering_direction = LEFT_DIRECTION;
      digitalWrite(STEPPER_dir_pin,LOW);
      if((G_stepper_posistion-stepper_pulses)<=STEPPER_LOWER_LIMIT)
      {
        stepper_pulses = STEPPER_LOWER_LIMIT+G_stepper_posistion-stepper_pulses+1;
      }
    }
    else if (Steering_direction == RIGHT_DIRECTION)
    {
      stepping_Steering_direction = RIGHT_DIRECTION;
      digitalWrite(STEPPER_dir_pin,HIGH);
      if((G_stepper_posistion+stepper_pulses)>=STEPPER_UPPER_LIMIT)
      {
        stepper_pulses = STEPPER_UPPER_LIMIT-G_stepper_posistion+stepper_pulses-1;
      }
    }
    //each interrupt sets the stepping pin either high or low
    // so the interrupt must happen "stepper_pulses*2" times in order to do the required stepper_pulses
    //Serial.print("pulses="); //Serial.println(stepper_pulses);
    G_stepper_desitnation = stepper_pulses * 2;
    digitalWrite(STEPPER_pulse_pin,LOW);
    TCNT1  = 0; //clear counter register
    TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt
  }
}

void Timer_1_INIT()
{
   TCCR1A = 0;// set entire TCCR1A register to 0
   TCCR1B = 0;// same for TCCR1B
   TCNT1  = 0;//initialize counter value to 0
   // set compare match register for 1hz increments
   OCR1A = 150;// = (16*10^6) / (1*1024) - 1 (must be <65536)
   // turn on CTC mode
   TCCR1B |= (1 << WGM12);
   // Set CS12 and CS10 bits for 1024 prescaler
   TCCR1B |= (1 << CS11) | (1 << CS10);
}


ISR(TIMER1_COMPA_vect) //STEPPER driving function
{
  static uint8_t pulse_state=LOW;
  G_runnung_interrupt_flag = 1;

  pulse_state ^= 1;
  digitalWrite(STEPPER_pulse_pin,pulse_state);

  if(stepping_Steering_direction==RIGHT_DIRECTION)
  {
    G_stepper_posistion ++;
     // Serial.println(" rigt");
  }
  if(stepping_Steering_direction==LEFT_DIRECTION)
  {
    G_stepper_posistion --;
    //Serial.println(" left");
  }
  G_stepper_desitnation--;
  //Serial.print("POS="); Serial.println(G_stepper_posistion);

  if(G_stepper_desitnation<=0)
  {
    TIMSK1 &=~(1<<OCIE1A);
    pulse_state = LOW;
    G_runnung_interrupt_flag = 0;
  }

}


void Stepepr_vHome(void)
{
  Serial.print("going home");
  if (G_runnung_interrupt_flag == 0 )
  {
  if(G_stepper_posistion >= 0)
  {
      G_stepper_desitnation = G_stepper_posistion;
      stepping_Steering_direction = LEFT_DIRECTION;
      digitalWrite(STEPPER_dir_pin,LOW);
      Serial.print(" left to ");
  }
  else if(G_stepper_posistion < 0)
  {
    G_stepper_desitnation = G_stepper_posistion*(-1);
    stepping_Steering_direction = RIGHT_DIRECTION;
    digitalWrite(STEPPER_dir_pin,HIGH);
    Serial.print(" right  to ");
  }

  digitalWrite(STEPPER_pulse_pin,LOW);
  Serial.print("  ");  Serial.println(G_stepper_desitnation);
  Serial.print(" pos =  ");  Serial.println(G_stepper_posistion);
  TCNT1  = 0; //clear counter register
  TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt
  delay(2000);

  while(G_runnung_interrupt_flag== 1 )
  {
    Serial.println("h");
  }
  }
}
void Stepper_vStop(void)
{
  TIMSK1 &=~(1<<OCIE1A);
  G_runnung_interrupt_flag = 0;
  digitalWrite(STEPPER_pulse_pin,LOW);
}
