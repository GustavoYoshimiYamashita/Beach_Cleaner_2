const uint8_t trig_pin1NORTE = 7;
const uint8_t echo_pin1NORTE = 6;

void setup() {
  Serial.begin(9600);
  pinMode(trig_pin1NORTE, OUTPUT);
  pinMode(echo_pin1NORTE, INPUT);
  digitalWrite(trig_pin1NORTE, LOW);

}

void loop() {
  
  digitalWrite(trig_pin1NORTE, HIGH);
  delayMicroseconds(11600);
  digitalWrite(trig_pin1NORTE, LOW);
  uint32_t pulse_time1 = pulseIn(echo_pin1NORTE, HIGH, 23320);

  double distanceNORTE = pulse_time1/58;
  if(distanceNORTE <= 0) distanceNORTE = 400;

  Serial.print("distancia: ");
  Serial.println(distanceNORTE);
  //delay(500);

}
