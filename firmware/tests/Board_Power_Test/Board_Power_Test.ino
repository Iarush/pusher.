// Blinks the onboard LED. Run this to confirm the Pico is powered and running
// once it is seated on the driver board.
void setup() { pinMode(LED_BUILTIN, OUTPUT); }

void loop() {
  digitalWrite(LED_BUILTIN, HIGH);
  delay(500);
  digitalWrite(LED_BUILTIN, LOW);
  delay(500);
}
