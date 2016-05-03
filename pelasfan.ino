
// bigup to http://www.plastibots.com/index.php/2016/02/27/lulzbot-mini-arduino-temp-monitor-fan-led-controller/

#define fanSpdPin 3// was5  //PWM out to control fan speed (blue wire on 4 pin fans) 

#define ledPin 13 // default LED

#define buttonPin 2 // for manual trigger of fan

// reduce fan speed
// TODO: modulate according to actual RPM, especially when powered by more than 12v
#define maxFanSpeed 200

int fanSpeed = 0;

// switch state
unsigned long previousMillis = 0;
const long interval = 4000;
bool fanOn = true;

// handle interrupt
volatile unsigned long previousInterrupt = 0;
const long bounceTime = 500; // how much time before we consider interrupt as outdated
volatile bool interruptOn = false;

void setup() {

  // will use bounce of LOW state to maintain fan on
  attachInterrupt(digitalPinToInterrupt(buttonPin), blink, HIGH);

  pinMode(fanSpdPin, OUTPUT);


  // for debug
  pinMode(fanSpdPin, OUTPUT);
  Serial.begin(9600);
}

void loop() {

  noInterrupts();

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


  if (interruptOn && currentMillis - previousInterrupt >= bounceTime)
  {
    Serial.println("STOP interrupt");
    interruptOn = false;
  }

  interrupts();

}

void setSpeed(int fspeed) {
  analogWrite(fanSpdPin, fspeed);
}


void blink() {
  interruptOn = true;
  detachInterrupt(digitalPinToInterrupt(buttonPin));
  previousInterrupt =  millis();
  Serial.println("interrupt");
}


