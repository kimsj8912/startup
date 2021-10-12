unsigned int i;

void setup() {
  pinMode(7, OUTPUT);
  Serial.begin(115200); //Initialize serial port
  while (!Serial) {
    ; // wait for serial port to connect.
  }
}

void loop() {
  digitalWrite(7, 0); // 1초동안 LED를 켬
  delay(1000);
  for (i = 1; i <= 10; i++) { // LED를 5번 깜빡이기 위해 10회 반복
    digitalWrite(7, i % 2); // i % 2의 결과값은 0과 1이 반복적으로 등장
    delay(100); // 꺼진 상태 또는 켜진 상태를 100ms 동안 유지
  }
  while(1) {
    digitalWrite(7, 1); // LED가 꺼진 상태
  }
}
