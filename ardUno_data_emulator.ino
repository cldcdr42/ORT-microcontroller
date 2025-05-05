long timestamp_us = millis();
int max = 100;

float angle;
float velocity;
float target;


void setup() {
  Serial.begin(115200);

  // if analog input pin 0 is unconnected, random analog
  // noise will cause the call to randomSeed() to generate
  // different seed numbers each time the sketch runs.
  // randomSeed() will then shuffle the random function.
  randomSeed(analogRead(0));
}


void loop() {
  // print a random number from 0 to max - 1
  target = (float)random(max + 1) / max;
  angle = random(max) - 50;
  velocity = random(max) / 10. - 5;
  
  Serial.print("D");
  Serial.print(target);
  Serial.print(";");
  Serial.print(angle);
  Serial.print(";");
  Serial.print(velocity);
  Serial.print("\r\n");

  delay(10);
}
