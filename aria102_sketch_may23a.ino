/**
 * RPM: 33.(3) (0.555 RPS)
 * CYCLE: 1801 ms
 * 
 * 1801 ms
 * 2pi / 10801 * time
 */

float DPI = 2.0 * PI;
float period = 1801.0 / 2.0;

void setup() {
  pinMode(3, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);

  TCCR2B = TCCR2B & B11111000 | B00000001; // for PWM frequency of 31372.55 Hz
  TCCR1B = TCCR1B & B11111000 | B00000001; // for PWM frequency of 31372.55 Hz
}

void loop() {
  float position = DPI / period * (float) millis();
  float phase1 = sin(position);
  float phase2 = cos(position);

  analogWrite(11, 255 * max(0, phase1));
  analogWrite(10, 255 * max(0, phase2));
  analogWrite(9, 255 * -min(0, phase1));
  analogWrite(3, 255 * -min(0, phase2));
}
