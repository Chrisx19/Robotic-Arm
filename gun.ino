const int buttonOne = 12; //actuator

void setup() {
pinMode(10, OUTPUT); //trigger
pinMode(9, OUTPUT); //safety
pinMode(buttonOne, INPUT_PULLUP);
}

void loop() {
  bool button1 = digitalRead(buttonOne);

  if(button1==false)
  {
    //bool button1 = digitalRead(buttonOne);
    digitalWrite(9,HIGH); //activate safety
    //delay(500);
    digitalWrite(10, HIGH); //activate trigger
    delay(1);

    digitalWrite(9, LOW); //disable safety
    digitalWrite(10, LOW);  //disable trigger
  }
}
