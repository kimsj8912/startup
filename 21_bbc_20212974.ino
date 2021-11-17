#include <Servo.h>

#define PIN_SERVO 10
#define PIN_IR A0
#define _DIST_ALPHA 0.5 // EMA weight of new sample (range: 0 to 1).

float a, b, c, d, e, f, g; // 적외선 센서 보정을 위한 상수

// ema 컨트롤을 위한 상수
float alpha;
float dist_ema;

Servo myservo;

void setup() {
  myservo.attach(PIN_SERVO);
  myservo.writeMicroseconds(1419);

  a = 69.77;
  b = 124.13;
  c = 167.42;
  d = 188.45;
  e = 207.69;
  f = 230.48;
  g = 247.74;

  alpha = _DIST_ALPHA;
  dist_ema = 0;

}

float ir_distance(void){ // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;
}

void loop() {
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

  dist_ema = dist_cali * alpha + (1 - alpha) * dist_ema; // EMA 보정 수식

  // Bang-Bang Control
  if(dist_ema >= 270) myservo.writeMicroseconds(1200);
  else myservo.writeMicroseconds(1590);
  
  delay(20);

}
