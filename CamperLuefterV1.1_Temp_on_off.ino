#include "Arduino.h"

//********DS18B20**********
#include <OneWire.h>
#include <DallasTemperature.h>

// Pin connected to the DS18B20 sensor
#define ONE_WIRE_BUS 2

// Setup a OneWire instance to communicate with any OneWire devices
OneWire oneWire(ONE_WIRE_BUS);

// Pass the OneWire reference to DallasTemperature
DallasTemperature sensors(&oneWire);

//*********PWM**********
#include <TimerOne.h>

// Pin-Definitionen
#define PWM_PIN_1 9       // PWM-Ausgang 1 für den Motor
#define PWM_PIN_2 10      // PWM-Ausgang 2 für den Motor
#define POT_PIN   A0      // Potentiometer-Eingang
#define LED_PIN       3       //Blinks once when Temp mode is off and twice when Temp mode is on

//**********ATTACHMENTS**********
//Pressing it changes the current Direction
#define BUTTON_DIRECTION 7
#define BUTTON_TEMP 6
#define TEMP_THRESHOLD 20

//*****STRUCTS******
typedef struct
{
  int   button_last_state;
  int   button_pin;
}button_info_t;

  
void setup() {
  //Serial
  Serial.begin(9600);
  Serial.println("Starting up..."); // <- add this
  
  //DS18B20
  sensors.begin();

  //PINS
  pinMode(POT_PIN, INPUT); // Configure button pin as input with pull-up
  pinMode(PWM_PIN_1, OUTPUT); // Configure button pin as input with pull-up
  pinMode(PWM_PIN_2, OUTPUT); // Configure button pin as input with pull-up  
  pinMode(BUTTON_DIRECTION, INPUT_PULLUP);
  pinMode(BUTTON_TEMP, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);

  // Timer1 auf 20 kHz einstellen
  Timer1.initialize(50);      // 50 µs = 20 kHz
  Timer1.pwm(PWM_PIN_1, 0);      // this function has to be used once to configure pin
  Timer1.pwm(PWM_PIN_2, 0);      // this function has to be used once to configure pin 
}

void loop() 
{
  //button structs
  static button_info_t button_temp = {1, BUTTON_TEMP};
  static button_info_t button_direction = {1, BUTTON_DIRECTION};

  //is used to controll the 2 different PWM PINS on the H-Bridge
  static bool current_direction = 0;
  //temperature mode
  static bool temp_mode = 0;

  //****DS18B20
  //affects the temperature mode
  static int current_temp = 0;
  static unsigned long old_millis_temp_sensor = 0;
  if(millis_passed(10000, &old_millis_temp_sensor))
    temp_read(&current_temp);

  //****DUTY CYCLE
  // read potentiometer 
  int potValue = analogRead(POT_PIN);
  //speed of the fan
  int dutyCycle = 0;
  //map potentiometer value to duty cycle
  if(temp_mode == 0 || (temp_mode == 1 && current_temp >= TEMP_THRESHOLD))
    dutyCycle = map(potValue, 0, 1023, 0, 1023);

/*
  //****DEBUG****
  static unsigned long old_millis_debug = 0;
  if(millis_passed(1000, &old_millis_debug))
  {
    Serial.println("DEBUG");
    Serial.print("current direction:");
    Serial.println(current_direction);
    Serial.print("duty cycle:");
    Serial.println(dutyCycle);
    Serial.print("temp_mode:");
    Serial.println(temp_mode);
    Serial.print("button state:");
    Serial.println(digitalRead(BUTTON_TEMP));
  }
*/

  //****TEMPRETURE MODE
  if(button_check(&button_temp))
  {
    temp_mode = !temp_mode;
    if(temp_mode == 0)
      led_blink(LED_PIN, 1, 200);
    else if(temp_mode == 1)
      led_blink(LED_PIN, 2, 200);
  }

  //****DIRECTION
  if(button_check(&button_direction))
    direction_change(&current_direction);
  
  //****update pwm signal depending on the direction
  if(current_direction == 0)
    Timer1.setPwmDuty(PWM_PIN_1, dutyCycle);
  else if(current_direction == 1)
    Timer1.setPwmDuty(PWM_PIN_2, dutyCycle);

  //short delay for stabilising
  delay(10);
}

void temp_read(int *current_temp)
{
  sensors.requestTemperatures();
  *current_temp = sensors.getTempCByIndex(0);
}

void led_blink(int LED, int n_blinks, int length_delay)
{
  for(int i = 0; i < n_blinks; i++)
  {
    //turn led on
    digitalWrite(LED, HIGH);
    //delay time x
    delay(length_delay);
    //turn led off
    digitalWrite(LED,LOW);
    delay(length_delay);
  }  
}

void direction_change(bool *current_direction)
{
      //reverse the direciton
    *current_direction = !*current_direction;

    ///set current pin duty cycle to 0 to stop the fan
    if(*current_direction == 0)
      Timer1.setPwmDuty(PWM_PIN_2, 0);
    else if(*current_direction == 1)
      Timer1.setPwmDuty(PWM_PIN_1, 0);

    //add delay to wait for fan to come to a stop -> depending on the value of the potentionmeter
    delay(4000);
}

//checks if the interval has passed
bool millis_passed(int time_span_millis, unsigned long *old_millis) 
{
  unsigned long new_millis = millis();

  // Overflow-safe comparison
  if ((new_millis - *old_millis) >= time_span_millis) {
    *old_millis = new_millis; // Update old_millismaxMenu
    return 1;                // Indicate success
  }
  return 0; // Indicate that the time span has not yet elapsed
}

bool button_check(button_info_t *button_info)
{
  //if button is not pressed = 1
  bool button_current_state = digitalRead(button_info->button_pin);

  //now we press the button = 0
  if(button_current_state == 0 && button_info->button_last_state == 1)
  {
    //if button is held it stays on 0 but button_last_state is now also 0
    //button = 0 && button_last_state = 0
    button_info->button_last_state = button_current_state;
    return 1;
  }
  //if i now release the button it goes to 1
  //button_last_state is still 0
  else if(button_current_state == 1 && button_info->button_last_state == 0)
  {
    //button not pressed = 1;
    button_info->button_last_state = button_current_state;
  }
  //button_last_state is now 1 and button current state = 1

  return 0;
}