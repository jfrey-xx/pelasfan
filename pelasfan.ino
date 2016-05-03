
// bigup to http://www.plastibots.com/index.php/2016/02/27/lulzbot-mini-arduino-temp-monitor-fan-led-controller/

#define fanSpdPin 3// was5  //PWM out to control fan speed (blue wire on 4 pin fans) 

#define hallsensor 2 //The pin location of the fan PWM sensor (green wire on 4 pin fans)

#define ledPin 13 // default LED

// reduce fan speed
// TODO: modulate according to actual RPM, especially when powered by more than 12v
#define maxFanSpeed 200

int fanSpeed = 0;

// switch state
unsigned long previousMillis = 0;
const long interval = 4000;

bool fanOn = true;

void setup() {
  pinMode(fanSpdPin, OUTPUT);

  // for debug
  pinMode(fanSpdPin, OUTPUT);
  Serial.begin(9600);
}

void loop() {

  unsigned long currentMillis = millis();


  if (currentMillis - previousMillis >= interval)
  {
    Serial.println("interval");
    // save the last time state was switched
    previousMillis = currentMillis;
    fanOn = !fanOn;
    if (fanOn) {
      setSpeed(maxFanSpeed);
      digitalWrite(13, HIGH);
    }
    else {
      setSpeed(0);
      digitalWrite(13, LOW);
    }
  }

}

void setSpeed(int fspeed) {
  analogWrite(fanSpdPin, fspeed);
}

