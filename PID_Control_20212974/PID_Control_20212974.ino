#include <Servo.h> // [2972] 서보 헤더파일 포함

/////////////////////////////
// Configurable parameters //
/////////////////////////////

// Arduino pin assignment
#define PIN_LED 9
#define PIN_SERVO 10
#define PIN_IR A0

// Framework setting
#define _DIST_TARGET 255
#define _DIST_MIN 100
#define _DIST_MAX 410

// Distance sensor
#include "medianfilter.h" // 윤여민 학우님의 인터럽트 중위수 필터를 적용하였습니다.

// Servo range
#define _DUTY_MIN 1180
#define _DUTY_NEU 1412
#define _DUTY_MAX 1550  

// Servo speed control
#define _SERVO_ANGLE 30 
#define _SERVO_SPEED 600

// Event periods
#define _INTERVAL_SERVO 20 
#define _INTERVAL_SERIAL 100 

// iterm range
#define _ITERM_MAX 30

// PID parameters
#define _KP 1
#define _KD 50
#define _KI 2


//////////////////////
// global variables //
//////////////////////

// Servo instance
Servo myservo;

// Distance sensor
float dist_target; // location to send the ball
float dist_raw;
float dist_ema = 0;
float alpha = 0.7;    // [2959] ema의 알파값을 저장할 변수

// Event periods
unsigned long last_sampling_time_servo, last_sampling_time_serial; 
bool event_servo, event_serial;

// Servo speed control
int duty_chg_per_interval; 
int duty_target, duty_curr;

// PID variables
float error_curr, error_prev, control, pterm, dterm, iterm;

// 적외선 센서 값 보정을 위한 상수
float a = 625.0;
float b = 422.0;
float c = 336.0;
float d = 306.0;
float e = 271.0;
float f = 244.0;
float g = 233.0;

// 적외선 센서 거리 보정
float ir_distance_filtered(short raw_dist){ // return value unit: mm
  float dist_cali;

  // 5cm 간격으로 적외선 센서 보정
  if (raw_dist > b){
    dist_cali = 100 + 50.0 / (b - a) * (raw_dist - a);
  }
  else if (raw_dist <= b && raw_dist > c) {
    dist_cali = 150 + 50.0 / (c - b) * (raw_dist - b);
  }
  else if (raw_dist <= c && raw_dist > d) {
    dist_cali = 200 + 50.0 / (d - c) * (raw_dist - c);
  }
  else if (raw_dist <= d && raw_dist > e) {
    dist_cali = 250 + 50.0 / (e - d) * (raw_dist - d);
  }
  else if (raw_dist <= e && raw_dist > f) {
    dist_cali = 300 + 50.0 / (f - e) * (raw_dist - e);
  }
  else if (raw_dist <= f) {
    dist_cali = 350 + 50.0 / (g - f) * (raw_dist - f);
  }
  
  // ema 구현
  if (dist_ema == 0) {
    dist_ema = dist_cali;
  }
  else {
    dist_ema = alpha * dist_cali + (1-alpha) * dist_ema;
  }
  
  return dist_ema;
}

MedianFilter<ir_distance_filtered> filter; // IR 필터링

void setup() {
// initialize GPIO pins for LED and attach servo 
  pinMode(PIN_LED, OUTPUT); // [2952] LED를 GPIO 9번 포트에 연결
  myservo.attach(PIN_SERVO); // [2952] 서보 모터를 GPIO 10번 포트에 연결

// initialize global variables
  dist_target = _DIST_TARGET;
  iterm = 0;
  control = 0;

// move servo to neutral position
  myservo.writeMicroseconds(_DUTY_NEU); // [2952] 서보 모터를 중간 위치에 지정

// initialize serial port
  Serial.begin(57600); // [2952] 시리얼 포트를 115200의 속도로 연결

// convert angle speed into duty change per interval.
  duty_chg_per_interval = (_DUTY_MAX - _DUTY_MIN) * (_SERVO_SPEED / 180.0) * (_INTERVAL_SERVO / 1000.0);

// 이벤트 변수 초기화
  last_sampling_time_servo = 0;
  last_sampling_time_serial = 0;
  event_servo = event_serial = false;

  filter.init(); // IR 필터링
}
  

void loop() {
/////////////////////
// Event generator //
/////////////////////

// wait until next sampling time. 
// millis() returns the number of milliseconds since the program started. Will overflow after 50 days.
  unsigned long time_curr = millis(); // 각 주기 측정을 위한 변수 time_curr 선언
  if(time_curr >= last_sampling_time_servo + _INTERVAL_SERVO) {
        last_sampling_time_servo += _INTERVAL_SERVO;
        event_servo = true; // 서보 업데이트 주기가 되었으므로 서보 이벤트 발생
  }
  if(time_curr >= last_sampling_time_serial + _INTERVAL_SERIAL) {
        last_sampling_time_serial += _INTERVAL_SERIAL;
        event_serial = true; // 출력 주기가 되었으므로 출력 이벤트 발생
  }


////////////////////
// Event handlers //
////////////////////


  if(filter.ready()) { // IR 필터링 준비가 되면
      dist_raw = filter.read(); // 센서 값 받아오기

  // PID control logic
      error_curr = _DIST_TARGET - dist_raw;
      pterm = _KP * error_curr;
      dterm = _KD * (error_curr - error_prev);
      iterm += _KI * error_curr;
      control = pterm + dterm + iterm;
      duty_target = _DUTY_NEU + control;

  // duty_target = f(duty_neutral, control)
      if(error_curr < 0) {
        duty_target = _DUTY_NEU + 1.2 * control;
      }
      else {
        duty_target = _DUTY_NEU + control;
      }
      
  // keep duty_target value within the range of [_DUTY_MIN, _DUTY_MAX]
      if (duty_target > _DUTY_MAX) {duty_target = _DUTY_MAX;}
      else if (duty_target < _DUTY_MIN) {duty_target = _DUTY_MIN;}

      if (abs(iterm) > _ITERM_MAX) iterm = 0;

// update error_prev
      error_prev = error_curr;
  }
  
  if(event_servo) {
      event_servo = false; // 서보 이벤트 실행 후 이벤트 종료
    // adjust duty_curr toward duty_target by duty_chg_per_interval
      if(duty_target > duty_curr) { // [2951] 현재 duty값이 목표값보다 작다면?
        duty_curr += duty_chg_per_interval; // [2951] duty_chg_per_interval 만큼 더함
      }
      else{ // [2951] 현재 duty값이 목표값보다 크다면?
        duty_curr -= duty_chg_per_interval; // [2951] duty_chg_per_interval 만큼 뺌
      }
    // update servo position
      myservo.writeMicroseconds(duty_curr); // [2951] 현재 duty값 만큼 servo를 이동

  }
  
  if(event_serial) {
    event_serial = false;
    Serial.print("IR:");
    Serial.print(dist_raw);
    Serial.print(",T:");
    Serial.print(dist_target);
    Serial.print(",P:");
    Serial.print(map(pterm,-1000,1000,510,610));
    Serial.print(",D:");
    Serial.print(map(dterm,-1000,1000,510,610));
    Serial.print(",I:");
    Serial.print(map(iterm,-1000,1000,510,610));
    Serial.print(",DTT:");
    Serial.print(map(duty_target,1000,2000,410,510));
    Serial.print(",DTC:");
    Serial.print(map(duty_curr,1000,2000,410,510));
    Serial.println(",-G:245,+G:265,m:0,M:800");
    
  }

  
}
