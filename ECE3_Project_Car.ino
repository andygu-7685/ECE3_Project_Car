#include <ECE3.h>
String dummy;


//----------------------------------------------------------------------------------
//Calibration
uint16_t sensorValues[8];
//Lab
// uint16_t maxVal[8] = {2500,	2297,	2500,	2500,	2248,	2500,	2500,	2500};
// uint16_t minVal[8] = {619,	573,	641,	688,	619,	688,	757,	828};
// uint16_t maxVal[8] = {2321,	1835,	1891,	2083,	1770,	1839,	2041,	2329};
// uint16_t minVal[8] = {721,	630,	723,	747,	653,	698,	749,	821};
//Apartment
uint16_t maxVal[8] = {2478,	2318,	2219,	2148,	1952,	2338,	2338,	2500};
uint16_t minVal[8] = {811,	626,	723,	751,	605,	668,	791,	817};
int16_t max_avg_error = 248;
//PCC
//uint16_t maxVal[8] = {2059, 1935, 2013, 2050, 1914, 1871, 1986, 2391};
//uint16_t minVal[8] = {750, 683, 723, 775, 694, 702, 771,	823};

//int16_t weightVal[8] = {-8, -4, -2, -1, 1, 2, 4, 8};
int16_t weightVal_left[8] = {0, 0, -1, -1, -1, 20, 30, 20};       //favor left branch
int16_t weightVal_right[8] = {-20, -30, -20, 1, 1, 1, 0, 0};      //favor right branch
int16_t weightVal_original[8] = {-15, -14, -12, -8, 8, 12, 14, 15};
int16_t weightVal[8] = {-15, -14, -12, -8, 8, 12, 14, 15};

const int number_samples = 5;

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
int base_pwm_speed = 30;

//true == right
//false == left
bool car_dir = true;
float avg_error = 0;
float K_p = 0.9 * base_pwm_speed / max_avg_error;
float K_d = 6.3 * K_p;
//----------------------------------------------------------------------------------

//----------------------------------------------------------------------------------
//Wheel Derivative Control
unsigned long current_time = 0;
unsigned long delta_time = 0;
unsigned long last_delta_time = 0;
float delta_error = 0;
float last_error = 0;
//----------------------------------------------------------------------------------

//----------------------------------------------------------------------------------
//Encoder
uint16_t avg_enc_cnt = 0;
unsigned long last_enc_time = 0;
const unsigned long enc_bin_len = 50e3;





uint16_t obstacle_ctr = 0;
uint16_t crosspiece = 0;

//0 = not detect peak
//1 = detect left peak
//2 = detect right peak
//3 = detect branch
uint16_t detect_peak = 0;




void setup()
{
  ECE3_Init();
  Serial.begin(9600); // set the data rate in bits per second for serial data transmission
  resetEncoderCount_left();
  resetEncoderCount_right();
  avg_enc_cnt = 0;
  last_enc_time = micros();
  last_delta_time = micros();
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
  uint16_t summed_values[8] = {0};
  int16_t non_weighted_sum = 0;
  uint16_t non_norm_sum = 0;
  uint16_t peak_value = 0;
  int16_t weighted_sum = 0;

  // Take the average of 5 consecutive values for each sensor
  //the time between each value depend on excution time
  for (unsigned char j = 0; j < number_samples; j++){
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


  //detect donut track
  //detect right
  if(detect_peak == 2){
    peak_value = max(((summed_values[0] / number_samples) - minVal[0]) * 1000 / (maxVal[0] - minVal[0])
                  , ((summed_values[1] / number_samples) - minVal[1]) * 1000 / (maxVal[1] - minVal[1]));
  }
  //detect left
  else if(detect_peak == 1){
    peak_value = max(((summed_values[6] / number_samples) - minVal[6]) * 1000 / (maxVal[6] - minVal[6])
                  , ((summed_values[7] / number_samples) - minVal[7]) * 1000 / (maxVal[7] - minVal[7]));
  }
  else if(detect_peak == 3){
    if( (((summed_values[2] / number_samples) - minVal[2]) * 1000 / (maxVal[2] - minVal[2])) + 
        ((summed_values[5] / number_samples) - minVal[5]) * 1000 / (maxVal[5] - minVal[5]) >= 2 * 800)
        peak_value = 1000;
  }



  // Print average values (average value = summed_values / number_samples
  for (unsigned char i = 0; i < 8; i++) {
    //must divide last to avoid precision loss
    //the divide by 8 is for weighting purpose
    weighted_sum += (weightVal[i] * (summed_values[i] / number_samples) - minVal[i]) * 1000 / (8 * (maxVal[i] - minVal[i]));
    non_weighted_sum += ((summed_values[i] / number_samples) - minVal[i]) * 1000 / (maxVal[i] - minVal[i]);
    non_norm_sum += (summed_values[i] / number_samples);
    // Serial.print(weightVal[i] * ((summed_values[i] / number_samples) - minVal[i]) * 1000 /(8 * (maxVal[i] - minVal[i])));
    // Serial.print('\t'); 
  }
  //Serial.println();
  //-------------------------------------------------------------------------------------------------------------

  //-------------------------------------------------------------------------------------------------------------
  //Calculation
  
  current_time = micros();

  // Encoder Speed Calculation
  if (current_time - last_enc_time > enc_bin_len) {  
    avg_enc_cnt = (getEncoderCount_left() + getEncoderCount_right()) / 2;       
    resetEncoderCount_left();
    resetEncoderCount_right();
    last_enc_time = current_time;
  }

  //detect crosspiece period
  if(non_norm_sum >= 8 * 2500)
    (crosspiece == 2) ? crosspiece = 0 : crosspiece = 2;
  else
    (crosspiece == 2) ? crosspiece = 1 : crosspiece = 0;

  //take average error of the 8 sensors
  avg_error = weighted_sum / 8 ;

  //true == right
  //false == left
  car_dir = (avg_error < 0);
  int16_t _p = abs(avg_error * K_p);

  delta_time = current_time - last_delta_time;
  delta_error = avg_error - last_error;
  //delta time is around 5ms, delta error is around 25 to 50
  //multiply by 1000 covert to delta time to ms
  //multiply by 10 for scaling factor
  float _d = abs(delta_error * 10 * 1000 * K_d / delta_time);
  //set _d to zero if current or prev error is a cross piece
  if(crosspiece != 0) _d = 0; 

  last_delta_time = current_time;
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
  //Obstacle Handling

  //detect black signal band
  if( (non_weighted_sum >= 8 * 800 && non_norm_sum < 8 * 2450) ||
      (peak_value >= 800 && detect_peak != 0 && non_norm_sum < 8 * 2450) ){
    switch(obstacle_ctr){
      // case 0:         //225 degree band
      //   rotation(20, 225);
      //   memcpy(weightVal, weightVal_left, sizeof(weightVal));
      // break;
      // case 1:         //first timed band
      //   memcpy(weightVal, weightVal_right, sizeof(weightVal));
      //   detect_peak = 1;
      // break;
      // case 2:         //first intersection with donut
      //   memcpy(weightVal, weightVal_left, sizeof(weightVal));
      //   detect_peak = 3;
      // break;
      // case 3:         //second intersection with donut
      //   memcpy(weightVal, weightVal_left, sizeof(weightVal));
      //   detect_peak = 2;
      // break;
      // case 4:         //first return intersection with donut
      //   memcpy(weightVal, weightVal_right, sizeof(weightVal));
      //   detect_peak = 1;
      // break;
      // case 5:         //second return intersection with donut
      //   memcpy(weightVal, weightVal_original, sizeof(weightVal));
      //   detect_peak = 0;
      // break;
      // case 6:         //second timed band
      //   memcpy(weightVal, weightVal_left, sizeof(weightVal));
      // break;

      case 0:         
        memcpy(weightVal, weightVal_right, sizeof(weightVal));
        detect_peak = 1;
        if(!calibration_function()){
          //blink 5 times if calibration was aborted midway
          for (unsigned char j = 0; j < 5; j++){
            digitalWrite(red_led_pin, HIGH);
            delay(600);
            digitalWrite(red_led_pin, LOW);
            delay(600);
          }
        }
        obstacle_ctr++;
      break;
      case 1:         //first timed band
        memcpy(weightVal, weightVal_right, sizeof(weightVal));
        detect_peak = 0;
        if(!calibration_function()){
          //blink 5 times if calibration was aborted midway
          for (unsigned char j = 0; j < 5; j++){
            digitalWrite(red_led_pin, HIGH);
            delay(600);
            digitalWrite(red_led_pin, LOW);
            delay(600);
          }
        }
        obstacle_ctr++;
      break;
      default:
        // if(!calibration_function()){
        //   //blink 5 times if calibration was aborted midway
        //   for (unsigned char j = 0; j < 5; j++){
        //     digitalWrite(red_led_pin, HIGH);
        //     delay(600);
        //     digitalWrite(red_led_pin, LOW);
        //     delay(600);
        //   }
        // }
      break;
    }
  }

  //-------------------------------------------------------------------------------------------------------------

  //-------------------------------------------------------------------------------------------------------------
  //Execution

  analogWrite(left_pwm_pin, left_pwm_state);
  analogWrite(right_pwm_pin, right_pwm_state);
  //-------------------------------------------------------------------------------------------------------------


}



//+angle = right rotation
//-angle = left rotation
//spd = pwm spd
void rotation(uint16_t spd, int16_t angle){
  resetEncoderCount_left();
  resetEncoderCount_right();
  int total_enc_cnt = 0;

  if(angle >= 0){
    digitalWrite(left_dir_pin, LOW);
    digitalWrite(right_dir_pin, HIGH);
  }
  else{
    digitalWrite(left_dir_pin, HIGH);
    digitalWrite(right_dir_pin, LOW);
  }

  analogWrite(left_pwm_pin, spd);
  analogWrite(right_pwm_pin, spd);

  while(total_enc_cnt <= abs(angle)){
    total_enc_cnt = abs(getEncoderCount_right());
    //Serial.print(total_enc_cnt);
    //Serial.println();
    delay(50);
  }

  digitalWrite(left_dir_pin, LOW);
  digitalWrite(right_dir_pin, LOW);

  resetEncoderCount_left();
  resetEncoderCount_right();
}


















bool calibration_function(){
  uint16_t _maxVal[8] = {0};        //temp array for new max and min
  uint16_t _minVal[8] = {0};
  uint16_t summed_values[8] = {0};
  for (unsigned char i = 0; i < 8; i++) sensorValues[i] = 0;
  int16_t current_position = -40;
  int16_t increment_position = 4;
  unsigned long press_start = micros();
  unsigned long press_duration = 0;
  uint16_t mean_matrix[21][8] = {0};
  int16_t _max_avg_error = 0;
  int16_t current_error = 0;

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

    for (unsigned char j = 0; j < 8; j++) summed_values[j] = 0;

    // Take the average of 5 consecutive values for each sensor
    //the time between each value depend on excution time
    for (unsigned char j = 0; j < number_samples; j++){
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
      mean_matrix[i][j] = mean_value;
      if(_maxVal[j] == 0 || mean_value > _maxVal[j]) _maxVal[j] = mean_value;
      if(_minVal[j] == 0 || mean_value < _minVal[j]) _minVal[j] = mean_value;
      Serial.print(mean_value);
      Serial.print('_');
      //test for black band
      Serial.print((mean_value - minVal[j]) * 1000 / (maxVal[j] - minVal[j]));     
      Serial.print("   ");
    }
    Serial.println();
  }

  //Max avg error calculation
  for (unsigned char i = 0; i < 21; i++) {
    current_error = 0;
    for (unsigned char j = 0; j < 8; j++) 
      current_error += (weightVal[j] * (mean_matrix[i][j] - _minVal[j]) * 1000) / (8 * 8 * (_maxVal[j] - _minVal[j]));
    if(_max_avg_error <= current_error) _max_avg_error = current_error;
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

  Serial.print("Max Error: "); 
  Serial.print(_max_avg_error);
  Serial.println();

  //abort calibration when button is hold for 5 sec
  while (press_duration != 0 && digitalRead(right_btn_pin) == LOW){
    press_duration = micros() - press_start;
    if(press_duration > 5000000) return false;
  }

  //copy the new calibration max and min to maxVal and minVal
  //copy the new max avg error
  memcpy(maxVal, _maxVal, sizeof(maxVal));
  memcpy(minVal, _minVal, sizeof(minVal));
  max_avg_error = _max_avg_error;
  K_p = 0.9 * base_pwm_speed / max_avg_error;
  K_d = 6.3 * K_p;
  //clean sensor value array
  for (int i = 0; i < 8; i++) sensorValues[i] = 0;
  digitalWrite(red_led_pin, LOW);
  return true;
}








