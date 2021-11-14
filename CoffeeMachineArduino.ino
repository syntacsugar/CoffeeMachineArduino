/*
  ArdDIO SSR
  2   CH7 pump steam
  3   CH8 pump brew pwm
  4   CH6 3 way valve
  5   CH5 grinder
  6   CH4 pid power
  7   CH3 gh heater
  8   CH2 brew led?
  9   CH8 light
*/

#include <math.h>

//Pins
//analogue
int steam_fill = A1;
int steam_ntctherm = A0;
int steam_pid = A2; //Steam Pid output
int brew_from_PID = A3;

// messages
int steam_fill_msg_sent = 0;
int brew_pid_msg_sent = 0;
int steam_pid_msg_sent = 0;
int steam_fill_msg = 0;
int brew_pid_msg = 0;
int steam_pid_msg = 0;

//digitals
int pump_brew = 3;// pwm
int pump_steam = 2; // note: switched with brew
int brew_valve = 4; //
int pid_power = 5;
int grinder = 6; 
int gh_heater = 7;
int tank_leds = 8;
int light = 9;

//Variables
double ThermValue = 0; // Somewhere to stick the raw Analogue Pin Value.
float temperature_read = 0.0;
//float set_temperature = 130;
float set_temperature = 30; //for testing
float PID_error = 0;
float previous_error = 0;
float elapsedTime, Time, timePrev;
int PID_value = 0;


//PID constants
int kp = 9.1;
int ki = 0.3;
int kd = 1.8;

int PID_p = 0;
int PID_i = 0;
int PID_d = 0;

void setup()
{
  Serial.begin(9600);
  Serial.println(" ");
  Serial.println(" ");
  Serial.println(" ");
  Serial.println(" ");
  Serial.println(" ");

  Serial.println(" ==================================================================");
  Serial.println(" ");
  Serial.println("Coffee Control starting!");

  pinMode(pump_brew, OUTPUT);
  pinMode(pump_steam, OUTPUT);
  pinMode(brew_valve, OUTPUT);
  pinMode(grinder, OUTPUT);
  pinMode(pid_power, OUTPUT);
  pinMode(gh_heater, OUTPUT);
  pinMode(tank_leds, OUTPUT);
  pinMode(light, OUTPUT);
  pinMode(steam_pid, OUTPUT);

  pinMode(steam_fill, INPUT_PULLUP);
  pinMode(brew_from_PID, INPUT_PULLUP);
  pinMode(steam_ntctherm, INPUT);

  digitalWrite(pump_brew, LOW);
  digitalWrite(pump_steam, LOW);
  digitalWrite(brew_valve, LOW);
  digitalWrite(grinder, LOW);
  //digitalWrite(pid_power, HIGH);
  digitalWrite(gh_heater, LOW);
  digitalWrite(tank_leds, LOW);
  digitalWrite(light, LOW);
  digitalWrite(steam_pid, LOW); 
  //Serial.println("Brew PID ON");

  Time = millis();

}

int control_BREW()
{
  if ( digitalRead(brew_from_PID) == 0 )
  { // pour that coffee
    digitalWrite(pump_brew, HIGH);
    digitalWrite(brew_valve, HIGH);
    Serial.println("Coffee pouring!");
  } else {
    digitalWrite(pump_brew, LOW);
    digitalWrite(brew_valve, LOW);
    Serial.println("Coffee pouring stopped");
  }
  return 1;
}

int control_STEAM_FILL_TANK()
{
  if ( digitalRead(steam_fill) == 0 )
  { //fill that steam tank
    digitalWrite(pump_steam, HIGH);
    steam_fill_msg = 1;
  } else {
    digitalWrite(pump_steam, LOW);
    steam_fill_msg = 2;
  }
  return 1;
}

float STEAM_TEMP_CONTROL( float PID_error)
{
  // determine current temp of steam tank
  ThermValue = analogRead(steam_ntctherm); //raw value from Voltage Divider (0-5)
  temperature_read = (1 / ((1 / 298.00) + (1 / 4100.00) * log(1024.00 / ThermValue - 1.00))) - 273.00;

  //calculate the error between the setpoint and the real value
  PID_error = set_temperature - temperature_read;
  //Calculate the P value
  PID_p = kp * PID_error;
  //Calculate the I value in a range on +-3
  if (-3 < PID_error < 3)
  {
    PID_i = PID_i + (ki * PID_error);
  }

  //For derivative we need real time to calculate speed change rate
  timePrev = Time; // the previous time is stored before the actual time read
  Time = millis(); // actual time read
  elapsedTime = (Time - timePrev) / 1000;
  //Now we can calculate the D calue
  PID_d = kd * ((PID_error - previous_error) / elapsedTime);
  //Final total PID value is the sum of P + I + D
  PID_value = PID_p + PID_i + PID_d;

  //We define PWM range between 0 and 255
  if (PID_value < 0)
  {
    PID_value = 0;
  }
  if (PID_value > 255)
  {
    PID_value = 255;
  }
  //Now we can write the PWM signal to the mosfet on digital pin D3
  //analogWrite(steam_pid,255-PID_value); //disabledPWM

  Serial.print (set_temperature);
  Serial.print (" ");
  Serial.println (temperature_read);

  if (PID_value > 0)
  {
    digitalWrite(steam_pid, HIGH); // sets the digital steam_pid on
    Serial.println("steam heating!");
  } else {
    digitalWrite(steam_pid, LOW);
    Serial.println("steam not heating");
  }
  previous_error = PID_error; //Remember to store the previous error for next loop.
  return previous_error;
}

void loop()
{
  //control_BREW();
  control_STEAM_FILL_TANK();
  //PID_error = STEAM_TEMP_CONTROL(PID_error);
  delay(2300);
  Serial.println(" ");
  Serial.println(" =========================");
  Serial.println(" ");
  //digitalWrite(pid_power, HIGH); 
  digitalWrite(pump_brew, HIGH);
  digitalWrite(brew_valve, HIGH);
  //digitalWrite(pump_steam, HIGH);
  
  int steam_fill_msg_sent = 0;
int brew_pid_msg_sent = 0;
int steam_pid_msg_sent = 0;
  if (steam_fill_msg != 0){
    if (steam_fill_msg == 1){
      Serial.println("steam tank filling!");
      steam_fill_msg = 0;
    }
    if (steam_fill_msg == 2){
      Serial.println("steam tank stopped filling!");
      steam_fill_msg = 0;
    }
  }
      
  brew_pid_msg = 0;
  steam_pid_msg = 0;
}
