
#define fanSpdPin 3// was5  //PWM out to control fan speed (blue wire on 4 pin fans) 

#define hallsensor 2 //The pin location of the fan PWM sensor (green wire on 4 pin fans)


unsigned char fanSpeed = 0;

void setup() {
  pinMode(fanSpdPin, OUTPUT);
}

void loop() {
  fanSpeed = fanSpeed + 10;
  //setSpeed(fanSpeed);
  Serial.println("Current speed:");
  //Serial.println(fanSpeed);
  //delay(2000);
  setSpeed(0);
  delay(4000);
  setSpeed(255);
  delay(4000);
}

void setSpeed(int fspeed) {
  analogWrite(fanSpdPin, fspeed);
}

