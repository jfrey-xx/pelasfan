
#define fanSpdPin 3// was5  //PWM out to control fan speed (blue wire on 4 pin fans) 

#define hallsensor 2 //The pin location of the fan PWM sensor (green wire on 4 pin fans)

// reduce fan speed
// TODO: modulate according to actual RPM, especially when powered by more than 12v
#define maxFanSpeed 200

int fanSpeed = 0;

void setup() {
  pinMode(fanSpdPin, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  fanSpeed = fanSpeed + 10;
  if (fanSpeed > 255) {
    fanSpeed = 0;
  }
  //setSpeed(fanSpeed);
  Serial.println("Current speed:");
  //Serial.println(fanSpeed);
  //delay(2000);
  setSpeed(0);
  delay(4000);
  setSpeed(maxFanSpeed);
  delay(4000);
}

void setSpeed(int fspeed) {
  analogWrite(fanSpdPin, fspeed);
}

