#include <Servo.h> // [2972] 서보 헤더파일 포함

/////////////////////////////
// Configurable parameters //
/////////////////////////////

// Arduino pin assignment
#define PIN_LED 9 // [1234] LED를 아두이노 GPIO 9번 핀에 연결
                  // [2345] ...하면 개선
#define PIN_SERVO 10
#define PIN_IR A0

// Framework setting
#define _DIST_TARGET 255
#define _DIST_MIN 100
#define _DIST_MAX 410

// Distance sensor
#define _DIST_ALPHA 0.5   // [2959] ema 필터에 적용할 알파값

// Servo range
#define _DUTY_MIN 1200
#define _DUTY_NEU 1419 
#define _DUTY_MAX 1590 

// Servo speed control
#define _SERVO_ANGLE 30 
#define _SERVO_SPEED 155

// Event periods
#define _INTERVAL_DIST 20 
#define _INTERVAL_SERVO 20 
#define _INTERVAL_SERIAL 100 

// PID parameters
#define _KP 0.8


//////////////////////
// global variables //
//////////////////////

// Servo instance
Servo myservo;

// Distance sensor
float dist_target; // location to send the ball
float dist_raw, dist_ema;
float alpha;    // [2959] ema의 알파값을 저장할 변수

// Event periods
unsigned long last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial; 
bool event_dist, event_servo, event_serial;

// Servo speed control
int duty_chg_per_interval; 
int duty_target, duty_curr;

// PID variables
float error_curr, error_prev, control, pterm, dterm, iterm;

float a, b, c, d, e, f, g; // 적외선 센서 보정을 위한 상수


void setup() {
// initialize GPIO pins for LED and attach servo 
pinMode(PIN_LED, OUTPUT); // [2952] LED를 GPIO 9번 포트에 연결
myservo.attach(PIN_SERVO); // [2952] 서보 모터를 GPIO 10번 포트에 연결

// initialize global variables
alpha = _DIST_ALPHA;   // [2959] ema의 알파값 초기화
dist_ema = 0;          // [2959] dist_ema 초기화
duty_curr = ir_distance_filtered(); // 현재 탁구공 위치 초기화

// move servo to neutral position
myservo.writeMicroseconds(_DUTY_NEU); // [2952] 서보 모터를 중간 위치에 지정

// initialize serial port
Serial.begin(57600); // [2952] 시리얼 포트를 115200의 속도로 연결

// convert angle speed into duty change per interval.
  duty_chg_per_interval = (_DUTY_MAX - _DUTY_MIN) * (_SERVO_SPEED / 180.0) * (_INTERVAL_SERVO / 1000.0);

// 이벤트 변수 초기화
  last_sampling_time_dist = 0;
  last_sampling_time_servo = 0;
  last_sampling_time_serial = 0;
  event_dist = event_servo = event_serial = false;

// 적외선 센서 보정 상수 초기화
  a = 69.77;
  b = 124.13;
  c = 167.42;
  d = 188.45;
  e = 207.69;
  f = 230.48;
  g = 247.74;
}
  

void loop() {
/////////////////////
// Event generator //
/////////////////////

// wait until next sampling time. 
// millis() returns the number of milliseconds since the program started. Will overflow after 50 days.
  unsigned long time_curr = millis(); // 각 주기 측정을 위한 변수 time_curr 선언
  if(time_curr >= last_sampling_time_dist + _INTERVAL_DIST) {
        last_sampling_time_dist += _INTERVAL_DIST;
        event_dist = true; // 거리측정 주기가 되었으므로 거리 이벤트 발생
  }
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


  if(event_dist) {
      event_dist = false; // 거리 이벤트 실행하며 이벤트 종료
  // get a distance reading from the distance sensor
      dist_raw = ir_distance_filtered();   // [2959] ??에 필터링된 측정값 저장
      if (dist_ema == 0){            // [2959] 맨 처음
        dist_ema = dist_raw;               // [2959] 맨 처음 ema값 = 필터링된 측정값
      }
      else{
        dist_ema = alpha * dist_raw + (1-alpha) * dist_ema;   // [2959] ema 구현
      }
  // PID control logic
      error_curr = _DIST_TARGET - dist_ema;
      pterm = error_curr;
      control = _KP * pterm;

  // duty_target = f(duty_neutral, control)
      if (error_curr >= 0) {duty_target = 1.752 * control + 1024.8;}
      else {duty_target = 1.14 * control + 1105.5;}

  // keep duty_target value within the range of [_DUTY_MIN, _DUTY_MAX]
      if (duty_target > _DUTY_MAX) {duty_target = _DUTY_MAX;}
      }
      else if (duty_target < _DUTY_MIN) {duty_target = _DUTY_MIN;}
  
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
    Serial.print("Min:0,Low:200,dist:");
    Serial.print(dist_raw);
    Serial.print(",pterm:");
    Serial.print(pterm);
    Serial.print(",duty_target:");
    Serial.print(duty_target);
    Serial.print(",duty_curr:");
    Serial.print(duty_curr);
    Serial.println(",High:310,Max:2000");
  }
}
float ir_distance(void){ // return value unit: mm
                         // [2959] 센서가 측정한 거리를 리턴해주는 함수
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;            // [2959] 거리측정 구현
}

float ir_distance_filtered(void){ // return value unit: mm
  float raw_dist = ir_distance();
  float dist_cali;

  // 5cm 간격으로 적외선 센서 보정
  if (raw_dist < 124.13){
    dist_cali = 100 + 50.0 / (b - a) * (raw_dist - a);
  }
  else if (raw_dist >= 124.13 && raw_dist < 167.42) {
    dist_cali = 150 + 50.0 / (c - b) * (raw_dist - b);
  }
  else if (raw_dist >= 167.42 && raw_dist < 188.45) {
    dist_cali = 200 + 50.0 / (d - c) * (raw_dist - c);
  }
  else if (raw_dist >= 188.45 && raw_dist < 207.69) {
    dist_cali = 250 + 50.0 / (e - d) * (raw_dist - d);
  }
  else if (raw_dist >= 207.69 && raw_dist < 230.48) {
    dist_cali = 300 + 50.0 / (f - e) * (raw_dist - e);
  }
  else if (raw_dist >= 230.48) {
    dist_cali = 350 + 50.0 / (g - f) * (raw_dist - f);
  }
  return dist_cali;
}
