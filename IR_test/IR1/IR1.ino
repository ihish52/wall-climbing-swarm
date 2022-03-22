double reading_a;
int reading;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(39, INPUT);

}

void loop() {
  // put your main code here, to run repeatedly:
  //reading_a = analogRead(39);
  reading = digitalRead(39);

  //Serial.println(reading_a);
  Serial.println(reading);
  Serial.println();
  delay(500);

}
