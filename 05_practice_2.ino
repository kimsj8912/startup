unsigned int i;

void setup() {
  pinMode(7, OUTPUT);
  Serial.begin(115200); //Initialize serial port
  while (!Serial) {
    ; // wait for serial port to connect.
  }
}

void loop() {
  digitalWrite(7, 0);
  delay(1000);
  for (i = 1; i <= 10; i++) {
    digitalWrite(7, i % 2);
    delay(100);
  }
  while(1) {
    digitalWrite(7, 1);
  }
}
