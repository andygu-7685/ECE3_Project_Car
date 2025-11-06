#include <ECE3.h>
String dummy;

//----------------------------------------------------------------------------------
//Calibration
uint16_t sensorValues[8];
uint16_t maxVal[8] = {0};
uint16_t minVal[8] = {0};
//int16_t weightVal[8] = {-8, -4, -2, -1, 1, 2, 4, 8};
int16_t weightVal[8] = {-15, -14, -12, -8, 8, 12, 14, 15};

int number_samples = 5;
bool print_directions = true;

//S1 button
int right_btn_pin = 73;
int btn_press_time = 0;
bool calibrate = false;
//----------------------------------------------------------------------------------



//----------------------------------------------------------------------------------
//Wheel
const int left_nslp_pin = 31;
const int left_dir_pin = 29;
const int left_pwm_pin = 40;

const int right_nslp_pin = 11;
const int right_dir_pin = 30;
const int right_pwm_pin = 39;

int left_pwm_state = 0;
int right_pwm_state = 0;
int base_pwm_speed = 10;

//true == right
//false == left
bool car_dir = true;
//----------------------------------------------------------------------------------

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

  digitalWrite(left_nslp_pin,HIGH);
  digitalWrite(right_nslp_pin,HIGH);

  digitalWrite(left_dir_pin, LOW);
  digitalWrite(right_dir_pin, LOW);

  if(calibrate){
    calibration_function();
  }
  else{
    uint16_t _maxVal[8] = {2500,	2297,	2500,	2500,	2248,	2500,	2500,	2500};
    uint16_t _minVal[8] = {619,	573,	641,	688,	619,	688,	757,	828};
    memcpy(maxVal, _maxVal, sizeof(maxVal));
    memcpy(minVal, _minVal, sizeof(minVal));
  }

}


void loop()
{

  //-------------------------------------------------------------------------------------------------------------
  //Setup
  if (print_directions)
  {
    for (int i = 8; i > 0; i--) {
      Serial.print("loading: ");
      Serial.println(i);
      delay(1000);
    }
    Serial.println("--------------");
    Serial.println("This is the IR sensor calibration script");
    Serial.println("To use, line up the car/track at the desired position (error value)");
    Serial.println("Then, press ENTER into the serial monitor input text field.");
    Serial.println("The leftmost column is the error; the other 8 columns are the 8 sensor values");
    print_directions = false;
  }

  //wait for user input
  while (Serial.available() == 0) {}
  dummy = Serial.readString();
  //-------------------------------------------------------------------------------------------------------------



  //-------------------------------------------------------------------------------------------------------------
  //Sampling
  int summed_values[8] = {0};
  int weighted_sum = 0;
  int avg_error = 0;

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
    Serial.print(weightVal[i] * ((summed_values[i] / number_samples) - minVal[i]) * 1000 /(8 * (maxVal[i] - minVal[i])));
    Serial.print('\t'); 
  }
  Serial.println();
  //-------------------------------------------------------------------------------------------------------------
  


  //-------------------------------------------------------------------------------------------------------------
  //Calculation
  //take average error of the 8 sensors
  avg_error = weighted_sum / 8 ;

  //true == right
  //false == left
  car_dir = (avg_error < 0);
  int _p = abs(avg_error * 40 / 350);

  left_pwm_state = base_pwm_speed;
  right_pwm_state = base_pwm_speed;

  if(car_dir){
    left_pwm_state += _p;
  }
  else{
    right_pwm_state += _p;
  }
  //-------------------------------------------------------------------------------------------------------------



  //-------------------------------------------------------------------------------------------------------------
  //Execution

  //analogWrite(left_pwm_pin, left_pwm_state);
  //analogWrite(right_pwm_pin, right_pwm_state);


}


bool calibration_function(){
  int current_position = -40;
  int increment_position = 4;
  int summed_values[8] = {0};
  for (int i = 0; i < 8; i++) sensorValues[i] = 0;

  for (unsigned char i = 0; i < 20; i++) {

    //the btn cooldown time is 500 ms
    while ((digitalRead(right_btn_pin) == HIGH) || ((micros() - btn_press_time) < 500000)) {}
    btn_press_time = micros();
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
      if(maxVal[j] == 0 || mean_value > maxVal[j]) maxVal[j] = mean_value;
      if(minVal[j] == 0 || mean_value < minVal[j]) minVal[j] = mean_value;
      Serial.print(summed_values[j] / number_samples);
      Serial.print('\t'); 
    }
    Serial.println();
  }

  Serial.print("Max: "); 
  for (unsigned char i = 0; i < 8; i++) {
    Serial.print(maxVal[i]);
    Serial.print('\t'); 
  }
  Serial.println();

  Serial.print("Min: "); 
  for (unsigned char i = 0; i < 8; i++) {
    Serial.print(minVal[i]);
    Serial.print('\t'); 
  }
  Serial.println();

  for (int i = 0; i < 8; i++) sensorValues[i] = 0;
}


