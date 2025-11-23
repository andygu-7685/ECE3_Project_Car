#include <ECE3.h>
String dummy;


//----------------------------------------------------------------------------------
//Calibration
uint16_t sensorValues[8];
//Lab
// uint16_t maxVal[8] = {2321,	1835,	1891,	2083,	1770,	1839,	2041,	2329};
// uint16_t minVal[8] = {721,	630,	723,	747,	653,	698,	749,	821};
uint16_t maxVal[8] = {1600,	1701,	1827,	1936,	1524,	1875,	1843,	1724	};
uint16_t minVal[8] = {366,	435,	435,	498,	366,	435,	457,	616	};
int16_t max_avg_error = 255;
//Apartment
// uint16_t maxVal[8] = {2478,	2318,	2219,	2148,	1952,	2338,	2338,	2500};
// uint16_t minVal[8] = {811,	626,	723,	751,	605,	668,	791,	817};
// int16_t max_avg_error = 248;
//PCC
//uint16_t maxVal[8] = {2059, 1935, 2013, 2050, 1914, 1871, 1986, 2391};
//uint16_t minVal[8] = {750, 683, 723, 775, 694, 702, 771,	823};
//Powell
// uint16_t maxVal[8] = {1258,	1512,	1294,	2052,	1957,	1959,	1737,	1457};
// uint16_t minVal[8] = {470,	403,	558,	556,	557,	554,	436,	652};
// int16_t max_avg_error = 212;
//Panda
// uint16_t maxVal[8] = {2262,	2168,	2050,	2252, 2237,	2308,	2426,	2500};
// uint16_t minVal[8] = {707,	571,	662,	685,	548,	685,	798,	868};
// int16_t max_avg_error = 201;
//Home
// uint16_t maxVal[8] = {2251,	2257,	2093,	2208,	2230,	2203,	2178,	2498};
// uint16_t minVal[8] = {606,	559,	668,	697,	647,  679,	765,	828};
// int16_t max_avg_error = 411;
// uint16_t maxVal[8] = {1813,	1795,	1723,	1794,	1723,	1723,	1837,	2077};
// uint16_t minVal[8] = {685,	616,	640,	685,	603,	616,	662,	755};
// int16_t max_avg_error = 300;

//int16_t weightVal[8] = {-8, -4, -2, -1, 1, 2, 4, 8};
int16_t weightVal_left[8] = {0, 0, 0, -20, -30, 20, 30, 20};       //favor left branch
int16_t weightVal_right[8] = {-20, -30, -20, 30, 20, 0, 0, 0};       //favor right branch
int16_t weightVal_left_cent[8] = {0, 0, 0, -5, -10, -20, 20, 15};      //center at left of car
int16_t weightVal_right_cent[8] = {-15, -20, 20, 10, 5, 0, 0, 0};     //center at right of car
int16_t weightVal_left_rot[8] = {0, 0, -10, -8, 8, 12, 14, 15};      //rotation left
int16_t weightVal_right_rot[8] = {-15, -14, -12, -8, 8, 10, 0, 0};     //rotation right
int16_t weightVal_concave[8] = {-15, -14, -12, -8, 8, 12, 14, 15};
int16_t weightVal_flat[8] = {-15, -15, -15, -15, 15, 15, 15, 15};
int16_t weightVal_blank[8] = {0, 0, 0, 0, 0, 0, 0, 0};
int16_t weightVal_convex[8] = {-8, -12, -14, -15, 15, 14, 12, 8};
//Official Track
int16_t weightVal_start[8] = {-25, -25, -25, 10, 25, 25, 25, 0};
// int16_t weightVal_start[8] = {0, -25, -30, 30, 30, 25, 0, 0};
int16_t weightVal[8] = {-25, -25, -25, 10, 25, 25, 25, 0};

const int number_samples = 5;

//S1 button
const int right_btn_pin = 73;
const int red_led_pin = 75;
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
int base_pwm_speed = 25;

//true == right
//false == left
bool car_dir = true;
float avg_error = 0;
float K_p = K_scale * 0.9 * base_pwm_speed / max_avg_error;
//----------------------------------------------------------------------------------


//----------------------------------------------------------------------------------
//Wheel Derivative Control
//delta_time = current_time - last_delta_time
unsigned long current_time = 0;
unsigned long last_delta_time = 0;
unsigned long delta_time = 0;
//delta_error = avg_error - last_error
float delta_error = 0;
float last_error = 0;
//correction pwm = delta_error * K_d / delta_time
float K_d = K_scale * 5 * K_p;
//----------------------------------------------------------------------------------


//----------------------------------------------------------------------------------
//Encoder
//avg_enc_cnt = number of encoder count every enc_bin_len us
uint16_t avg_enc_cnt = 0;
unsigned long last_enc_time = 0;
const unsigned long enc_bin_len = 50e3;
//cumulative encoder count that gets reset to 0 depending on the obstacle handling logic
uint16_t total_enc_cnt = 0;
//----------------------------------------------------------------------------------

//----------------------------------------------------------------------------------
//Path return
float off_path_error = 0;
unsigned long last_path_time = 0;
const unsigned long path_bin_len = 500e3;
//----------------------------------------------------------------------------------

//count how many obstacle the car had passed
uint16_t obstacle_ctr = 0;
//0 = no crosspiece
//1 = the first sample after a crosspiece
//2 = cross piece detected
uint16_t crosspiece = 0;
//prevent black band repetition in obstacle handling
unsigned long last_milestone_time = 0;
float map_factor = 1.1;
float K_scale = 1.0;
enum Color {
    Start,
    Bar1,
    AtDiscont,
    AfterDiscont,
    Branch,
    AfterBranch,
    Bar2,
    StartDonut,
    Midway1,
    Midway2,
    EndDonut,
    //Bar3,
    StartDouble,
    EndDouble,
    End,
    Origin
};


//set speed of the car
void setSpd(int spd, bool _change = true);



void setup()
{
  ECE3_Init();
  Serial.begin(9600); // set the data rate in bits per second for serial data transmission
  resetEncoderCount_left();
  resetEncoderCount_right();
  memcpy(weightVal, weightVal_flat, sizeof(weightVal));
  avg_enc_cnt = 0;
  last_enc_time = micros();
  last_delta_time = micros();
  last_path_time = micros();
  setSpd(25);
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

  //calibration method
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
  int16_t weighted_sum = 0;

  // Take the average of 5 consecutive values for each sensor
  // the time between each value depend on excution time
  for (unsigned char j = 0; j < number_samples; j++){
    // Read all 8 raw sensor values into array
    ECE3_read_IR(sensorValues);

    // Add each current sensor values to their respective index
    // in the summed values array using a for loop
    for (unsigned char i = 0; i < 8; i++)
      summed_values[i] += sensorValues[i];
  }

  // Print average values (average value = summed_values / number_samples
  for (unsigned char i = 0; i < 8; i++) {
    //must divide last to avoid precision loss, the divide by 8 is for weighting purpose
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
    avg_enc_cnt = (abs(getEncoderCount_left()) + abs(getEncoderCount_right())) / 2;
    total_enc_cnt += avg_enc_cnt;         //cumulative encoder count       
    resetEncoderCount_left();
    resetEncoderCount_right();
    last_enc_time = current_time;
  }

  //detect crosspiece period, if there's a crosspiece, 
  //the current smaple and next sample will not be used for derivative control calculation
  if(non_norm_sum >= 8 * 2500)
    (crosspiece == 2) ? crosspiece = 0 : crosspiece = 2;
  else
    (crosspiece == 2) ? crosspiece = 1 : crosspiece = 0;


  //PID Calculation

  //Proportional Control:
  //take average error of the 8 sensors
  avg_error = weighted_sum / 8 ;
  //false == left, true == right
  car_dir = (avg_error < 0);
  int16_t _p = abs(avg_error * K_p);

  //Derivative Control:
  delta_time = current_time - last_delta_time;
  delta_error = avg_error - last_error;
  //delta time is around 5ms, delta error is around 25 to 50
  //multiply by 1000 covert to delta time to ms
  //multiply by 10 for scaling factor
  float _d = abs(delta_error * 10 * 1000 * K_d / delta_time);
  //set _d to zero if current or prev avg_error is a cross piece
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


  //path return calculation
  if(current_time - last_path_time > path_bin_len && 
     non_weighted_sum >= 8 * 200 && non_norm_sum > 8 * 1) {
    off_path_error = avg_error;
    last_path_time = current_time;
  }
  //-------------------------------------------------------------------------------------------------------------


  //-------------------------------------------------------------------------------------------------------------
  //Execution
  if(left_pwm_state < 0){ left_pwm_state = 0; }
    //digitalWrite(left_dir_pin, HIGH);
  else{}
    //digitalWrite(left_dir_pin, LOW);

  if(right_pwm_state < 0){ right_pwm_state = 0; }
    //digitalWrite(right_dir_pin, HIGH);
  else{}
    //digitalWrite(right_dir_pin, LOW);

  analogWrite(left_pwm_pin, left_pwm_state);
  analogWrite(right_pwm_pin, right_pwm_state);
  //-------------------------------------------------------------------------------------------------------------


  //-------------------------------------------------------------------------------------------------------------
  //Obstacle Handling

  //detect black signal band
  if( (non_weighted_sum >= 8 * 800 &&                     //detect black band 
       non_norm_sum < 8 * 2450 &&                         //make sure it is not a crosspiece
       current_time - last_milestone_time > 1000000 &&    //make sure the black band was not already processed by the obstacle handling algotithm 
       (obstacle_ctr == Bar1 || obstacle_ctr == Bar2 || obstacle_ctr == End))   ||     //make sure the algorithm only check black band when certain obstacle is passed

       //(non_weighted_sum <= 8 * 200 && non_norm_sum > 8 * 1) ||                      //for path return algorithm

       (total_enc_cnt >= 90 * map_factor && obstacle_ctr == Start) ||
       (total_enc_cnt >= 100 * map_factor && obstacle_ctr == AtDiscont) ||
       (total_enc_cnt >= 100 * map_factor && obstacle_ctr == AfterDiscont) ||
       (total_enc_cnt >= 500 * map_factor && obstacle_ctr == Branch) ||     //after the 225 degree band
       (total_enc_cnt >= 90 * map_factor && obstacle_ctr == AfterBranch) ||     //shortly after branch off
       (total_enc_cnt >= 115 * map_factor && obstacle_ctr == StartDonut) ||     //after the first timed band
       (total_enc_cnt >= 82*2 * map_factor && obstacle_ctr == Midway1)   ||     //place hold
       (total_enc_cnt >= 332*2 * map_factor && obstacle_ctr == Midway2)   ||     //place hold      official: 332      home: 400
       (total_enc_cnt >= 90*2 * map_factor && obstacle_ctr == EndDonut) ||
       (total_enc_cnt >= 500 * map_factor&& obstacle_ctr == StartDouble)  ||
       (total_enc_cnt >= 400 * map_factor&& obstacle_ctr == EndDouble)  ||
       (total_enc_cnt >= 300 * map_factor&& obstacle_ctr == Origin)  ){     //after the donut and right turn
    
    ////for path return algorithm
    // if(non_weighted_sum <= 8 * 200 && non_norm_sum > 8 * 1){
    //   if(!cahlibration_function()){
    //     //blink 5 times if calibration was aborted midway
    //     for (unsigned char j = 0; j < 5; j++){
    //       digitalWrite(red_led_pin, HIGH);
    //       delay(600);
    //       digitalWrite(red_led_pin, LOW);
    //       delay(600);
    //     }
    //   }
    // }
    
    switch(obstacle_ctr){
      case Start:
        memcpy(weightVal, weightVal_start, sizeof(weightVal));
        setSpd(25);
        total_enc_cnt = 0;
        obstacle_ctr++;
      break;
      case Bar1:         //225 degree band
        //rotate 60 degree, put on favor right weighting mask
        //reset total encoder count to 0, to get the distance
        //car traveled from the 225 degree band
        //reset milestone time to prevent repetition
        rotation(base_pwm_speed, -265, true);             //official: -265    home: -275
        memcpy(weightVal, weightVal_right_cent, sizeof(weightVal));
        setSpd(20);
        total_enc_cnt = 0;
        last_milestone_time = current_time;
        obstacle_ctr++;
      break;
      case AtDiscont:
        memcpy(weightVal, weightVal_left, sizeof(weightVal));
        setSpd(20);
        total_enc_cnt = 0;
        obstacle_ctr++;
      break;
      case AfterDiscont:
        memcpy(weightVal, weightVal_right_cent, sizeof(weightVal));
        setSpd(25);
        total_enc_cnt = 0;
        obstacle_ctr++;
      break;
      case Branch:         //after car pass discont., right when the car is at the split off point
        //put on favor left weighting mask, such the car follow the left path
        memcpy(weightVal, weightVal_left_rot, sizeof(weightVal));
        setSpd(25);
        total_enc_cnt = 0;
        obstacle_ctr++;
      break;
      case AfterBranch:         
        memcpy(weightVal, weightVal_convex, sizeof(weightVal));
        setSpd(25);
        total_enc_cnt = 0;
        obstacle_ctr++;
      break;
      case Bar2:         //first timed band
        //put on regular weighting to center the car
        //reset total encoder count to 0, to get the distance
        //car traveled from the first timed band
        memcpy(weightVal, weightVal_concave, sizeof(weightVal));
        setSpd(25);
        total_enc_cnt = 0;
        last_milestone_time = current_time;
        obstacle_ctr++;
      break;
      case StartDonut:         //quarter way: a short distance after first timed band
        //start rotation 90 degree to the right
        //rotation(base_pwm_speed, 90, false);
        memcpy(weightVal, weightVal_right_rot, sizeof(weightVal));
        setSpd(15, false);
        total_enc_cnt = 0;
        obstacle_ctr++;
      break;
      case Midway1:         //first midway: when the car start the donut circle
        //start 360 donut rotation to the left
        //rotation(base_pwm_speed, -360, false);
        memcpy(weightVal, weightVal_left_rot, sizeof(weightVal));
        setSpd(15, false);
        total_enc_cnt = 0;
        obstacle_ctr++;
      break;
      case Midway2:       //second midway: when the car end the donut circle
        //start 90 degree rotation to the right
        //rotation(base_pwm_speed, 90, false);
        memcpy(weightVal, weightVal_right_rot, sizeof(weightVal));
        setSpd(15, false);
        total_enc_cnt = 0;
        obstacle_ctr++;
      break;
      case EndDonut:       //after donut and right turn
        //switch back to regular weighting mask
        memcpy(weightVal, weightVal_concave, sizeof(weightVal));
        setSpd(25);
        total_enc_cnt = 0;
        obstacle_ctr++;
      break;
      case StartDouble:
        memcpy(weightVal, weightVal_right_cent, sizeof(weightVal));
        setSpd(25);
        total_enc_cnt = 0;
        obstacle_ctr++;
      break;
      case EndDouble:
        memcpy(weightVal, weightVal_start, sizeof(weightVal));
        setSpd(25);
        total_enc_cnt = 0;
        obstacle_ctr++;
      break;
      case End:
        rotation(base_pwm_speed, 60, true);
        memcpy(weightVal, weightVal_blank, sizeof(weightVal));
        setSpd(25);
        total_enc_cnt = 0;
        // if(!calibration_function()){
        //   //blink 5 times if calibration was aborted midway
        //   for (unsigned char j = 0; j < 5; j++){
        //     digitalWrite(red_led_pin, HIGH);
        //     delay(600);
        //     digitalWrite(red_led_pin, LOW);
        //     delay(600);
        //   }
        // }
        obstacle_ctr++;
      break;
      case Origin:
        rotation(base_pwm_speed, 135, true);
        memcpy(weightVal, weightVal_start, sizeof(weightVal));
        setSpd(25);
        total_enc_cnt = 0;
        obstacle_ctr = 0;
      break;
      default:
        if(!calibration_function()){
          //blink 5 times if calibration was aborted midway
          for (unsigned char j = 0; j < 5; j++){
            digitalWrite(red_led_pin, HIGH);
            delay(600);
            digitalWrite(red_led_pin, LOW);
            delay(600);
          }
        }
      break;
    }

    //for debug purpose
    if(last_milestone_time != current_time){
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
    
  }
  //-------------------------------------------------------------------------------------------------------------


}



void setSpd(int spd, bool _change){
  base_pwm_speed = K_scale * spd;
  if(_change){
    K_p = K_scale * 0.9 * base_pwm_speed / max_avg_error;
    K_d = K_scale * 5 * K_p;
  }
}





//+angle = right rotation
//-angle = left rotation
//
//spd = pwm spd
//
//twoWheel:
//true = both wheel
//false = one wheel
void rotation(uint16_t spd, int16_t angle, bool twoWheel){
  resetEncoderCount_left();
  resetEncoderCount_right();
  int _enc_cnt = 0;
  int wheelAngle = angle * 2;

  if(twoWheel){
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
  }
  else{
    if(angle >= 0){
      analogWrite(left_pwm_pin, spd);
      analogWrite(right_pwm_pin, 0);
    }
    else{
      analogWrite(left_pwm_pin, 0);
      analogWrite(right_pwm_pin, spd);
    }
  }

  while(_enc_cnt <= abs(wheelAngle)){
    _enc_cnt = (abs(getEncoderCount_right()) + abs(getEncoderCount_left())) / 2;
    //Serial.print(_enc_cnt);
    //Serial.println();
    delay(50);
  }

  analogWrite(left_pwm_pin, 0);
  analogWrite(right_pwm_pin, 0);

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
      if(press_duration > 2000000) return false;
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
  setSpd(25);
  //clean sensor value array
  for (int i = 0; i < 8; i++) sensorValues[i] = 0;
  digitalWrite(red_led_pin, LOW);
  return true;
}








