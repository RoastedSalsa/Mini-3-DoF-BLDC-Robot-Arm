// Explicit PA5 test for NUCLEO-G474RE
void setup() {
  pinMode(PA5, OUTPUT);      // explicit port
  digitalWrite(PA5, LOW);    // known start
  delay(200);
}

void loop() {
  digitalWrite(PA5, HIGH);
  delay(2000);
  digitalWrite(PA5, LOW);
  delay(2000);
}
