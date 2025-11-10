#include <ECE3.h>
String dummy;


//----------------------------------------------------------------------------------
//Calibration
uint16_t sensorValues[8];
//Lab
// uint16_t maxVal[8] = {2500,	2297,	2500,	2500,	2248,	2500,	2500,	2500};
// uint16_t minVal[8] = {619,	573,	641,	688,	619,	688,	757,	828};
//Apartment
// uint16_t maxVal[8] = {2478,	2318,	2219,	2148,	1952,	2338,	2338,	2500};
// uint16_t minVal[8] = {811,	626,	723,	751,	605,	668,	791,	817};
//PCC
uint16_t maxVal[8] = {2059, 1935, 2013, 2050, 1914, 1871, 1986, 2391};
uint16_t minVal[8] = {750, 683, 723, 775, 694, 702, 771,	823};

//int16_t weightVal[8] = {-8, -4, -2, -1, 1, 2, 4, 8};
int16_t weightVal[8] = {-15, -14, -12, -8, 8, 12, 14, 15};

const int number_samples = 5;
bool print_directions = true;

//S1 button
const int right_btn_pin = 73;
const int red_led_pin = 75;
bool calibrate = true;
//----------------------------------------------------------------------------------



//----------------------------------------------------------------------------------
//Wheel Proportional Control
const int left_nslp_pin = 31;
const int left_dir_pin = 29;
const int left_pwm_pin = 40;

const int right_nslp_pin = 11;
const int right_dir_pin = 30;
const int right_pwm_pin = 39;

int left_pwm_state = 0;
int right_pwm_state = 0;
int base_pwm_speed = 50;

//true == right
//false == left
bool car_dir = true;
float avg_error = 0;
//----------------------------------------------------------------------------------

//----------------------------------------------------------------------------------
//Wheel Derivative Control
unsigned long delta_time = 0;
unsigned long last_delta_time = 0;
float delta_error = 0;
float last_error = 0;






void setup()
{
  ECE3_Init();
  Serial.begin(9600); // set the data rate in bits per second for serial data transmission
  delay(1000);

  pinMode(left_nslp_pin, OUTPUT);
  pinMode(left_dir_pin, OUTPUT);
  pinMode(left_pwm_pin, OUTPUT);

  pinMode(right_nslp_pin, OUTPUT);
  pinMode(right_dir_pin, OUTPUT);
  pinMode(right_pwm_pin, OUTPUT);

  pinMode(right_btn_pin, INPUT_PULLUP);
  pinMode(red_led_pin, OUTPUT);

  digitalWrite(left_nslp_pin,HIGH);
  digitalWrite(right_nslp_pin,HIGH);

  digitalWrite(left_dir_pin, LOW);
  digitalWrite(right_dir_pin, LOW);

  digitalWrite(red_led_pin,HIGH);
  delay(1000);
  digitalWrite(red_led_pin, LOW);

}


void loop()
{

  //-------------------------------------------------------------------------------------------------------------
  //Setup
  if (print_directions)
  {
    print_directions = false;
    last_delta_time = micros();
  }

  //wait for user input
  // while (Serial.available() == 0) {}
  // dummy = Serial.readString();

  if(digitalRead(right_btn_pin) == LOW){
    if(!calibration_function()){
      //blink 5 times if calibration was aborted midway
      for (unsigned char j = 0; j < 5; j++){
        digitalWrite(red_led_pin, HIGH);
        delay(600);
        digitalWrite(red_led_pin, LOW);
        delay(600);
      }
    }
  }
  //-------------------------------------------------------------------------------------------------------------



  //-------------------------------------------------------------------------------------------------------------
  //Sampling
  int summed_values[8] = {0};
  int weighted_sum = 0;

  // Take the average of 5 consecutive values for each sensor
  //the time between each value depend on excution time
  for (int j = 0; j < number_samples; j++){
    // Read all 8 raw sensor values into array
    ECE3_read_IR(sensorValues);

    // Add each current sensor values to their respective index
    // in the summed values array using a for loop
    for (unsigned char i = 0; i < 8; i++)
    {
      summed_values[i] += sensorValues[i];
    }
  }

  // Print current position at start of each column
  //Serial.print(current_position);
  //Serial.print("\t");
  // Increment current position by the increment value
  //current_position += increment_position;

  // Print average values (average value = summed_values / number_samples
  for (unsigned char i = 0; i < 8; i++) {
    //the last operation should always be division, because value less than 1 can be cast to 0 in int, if not handled correctly
    //the divide by 8 is for weighting purpose
    weighted_sum += (weightVal[i] * ((summed_values[i] / number_samples) - minVal[i]) * 1000) / (8 * (maxVal[i] - minVal[i]));
    // Serial.print(weightVal[i] * ((summed_values[i] / number_samples) - minVal[i]) * 1000 /(8 * (maxVal[i] - minVal[i])));
    // Serial.print('\t'); 
  }
  //Serial.println();
  //-------------------------------------------------------------------------------------------------------------
  


  //-------------------------------------------------------------------------------------------------------------
  //Calculation
  //take average error of the 8 sensors
  avg_error = weighted_sum / 8 ;

  //true == right
  //false == left
  car_dir = (avg_error < 0);
  int _p = abs(avg_error * 60 / 350);

  delta_time = micros() - last_delta_time;
  delta_error = avg_error - last_error;
  float _d = abs(delta_error * 8000 / delta_time);

  last_delta_time = micros();
  last_error = avg_error;
  
  // Serial.print("_d: ");
  // Serial.print(_d);
  // Serial.print('\t');
  // Serial.print(delta_error);
  // Serial.print('\t');
  // Serial.print(delta_time/10000);
  // Serial.print('\t');
  // Serial.println();

  if(car_dir){
    left_pwm_state = base_pwm_speed + _p - _d;
    right_pwm_state = base_pwm_speed - _p + _d;
  }
  else{
    left_pwm_state = base_pwm_speed - _p + _d;
    right_pwm_state = base_pwm_speed + _p - _d;
  }
  //-------------------------------------------------------------------------------------------------------------



  //-------------------------------------------------------------------------------------------------------------
  //Execution

  analogWrite(left_pwm_pin, left_pwm_state);
  analogWrite(right_pwm_pin, right_pwm_state);


}


bool calibration_function(){
  uint16_t _maxVal[8] = {0};        //temp array for new max and min
  uint16_t _minVal[8] = {0};
  int summed_values[8] = {0};
  for (int i = 0; i < 8; i++) sensorValues[i] = 0;
  int current_position = -40;
  int increment_position = 4;
  unsigned long press_start = micros();
  unsigned long press_duration = 0;
  digitalWrite(red_led_pin, HIGH);  //led indicator is on during calibration
  analogWrite(left_pwm_pin, 0);
  analogWrite(right_pwm_pin, 0);

  for (unsigned char i = 0; i < 21; i++) {

    //the btn cooldown time is 500 ms
    while ((digitalRead(right_btn_pin) == HIGH) || ((micros() - press_start) < 500000)) {
      //check if the button was released
      if(press_duration != 0 && digitalRead(right_btn_pin) == HIGH){
        press_duration = 0;
      }
      if(Serial.available() != 0){
        dummy = Serial.readString();
        press_duration = 0;
        break;
      }
    }

    if(press_duration != 0){
      press_duration = micros() - press_start;
      //abort calibration when button is hold for 5 sec
      if(press_duration > 5000000) return false;
    }
    else{
      press_start = micros();
      press_duration = 1;
    }

    for (int i = 0; i < 8; i++) summed_values[i] = 0;

    // Take the average of 5 consecutive values for each sensor
    //the time between each value depend on excution time
    for (int j = 0; j < number_samples; j++){
      // Read all 8 raw sensor values into array
      ECE3_read_IR(sensorValues);

      // Add each current sensor values to their respective index
      // in the summed values array using a for loop
      for (unsigned char k = 0; k < 8; k++)
      {
        summed_values[k] += sensorValues[k];
      }
    }

    //Print current position at start of each column
    Serial.print(current_position);
    Serial.print("\t");
    // Increment current position by the increment value
    current_position += increment_position;

    // Print average values (average value = summed_values / number_samples
    for (unsigned char j = 0; j < 8; j++) {
      int mean_value = summed_values[j] / number_samples;
      if(_maxVal[j] == 0 || mean_value > _maxVal[j]) _maxVal[j] = mean_value;
      if(_minVal[j] == 0 || mean_value < _minVal[j]) _minVal[j] = mean_value;
      //weighted_sum += (weightVal[i] * ((summed_values[j] / number_samples) - minVal[i]) * 1000) / (8 * (maxVal[i] - minVal[i]));
      Serial.print(summed_values[j] / number_samples);
      Serial.print('\t'); 
    }
    Serial.println();
  }

  Serial.print("Max: "); 
  for (unsigned char i = 0; i < 8; i++) {
    Serial.print(_maxVal[i]);
    Serial.print('\t'); 
  }
  Serial.println();

  Serial.print("Min: "); 
  for (unsigned char i = 0; i < 8; i++) {
    Serial.print(_minVal[i]);
    Serial.print('\t'); 
  }
  Serial.println();

  //abort calibration when button is hold for 5 sec
  while (press_duration != 0 && digitalRead(right_btn_pin) == LOW){
    press_duration = micros() - press_start;
    if(press_duration > 5000000) return false;
  }

  //copy the new calibration max and min to maxVal and minVal
  memcpy(maxVal, _maxVal, sizeof(maxVal));
  memcpy(minVal, _minVal, sizeof(minVal));
  //clean sensor value array
  for (int i = 0; i < 8; i++) sensorValues[i] = 0;
  digitalWrite(red_led_pin, LOW);
  return true;
}


