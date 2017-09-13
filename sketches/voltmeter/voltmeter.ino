// Measures voltage on analog pins and prints CSV results to the 
// serial port to be plotted in Arduino Serial Plotter.

// Which analog in pins to look at.
int input_pins[] = {A0, A1, /* A2, */ /* A3, */ /* A4, */ /* A5 */};
// The size of input_pin[] above, computed in setup().
int num_input_pins = 0;

void setup() {
  num_input_pins = sizeof(input_pins) / sizeof(input_pins[0]);
  for (int pin_idx = 0; pin_idx < num_input_pins; ++pin_idx) {
    pinMode(input_pins[pin_idx], INPUT);
  }

  Serial.begin(115200);
}

void loop() {
  for (int pin_idx = 0; pin_idx < num_input_pins; ++pin_idx) {
    Serial.print(analogRead(input_pins[pin_idx]));
    Serial.print(",");
    // Wait for voltage measurement circuitry to stabilize after usage.
    delay(2);
  }
  Serial.println("");
  delay(10);
}
