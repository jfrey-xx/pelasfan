
// bigup to http://www.plastibots.com/index.php/2016/02/27/lulzbot-mini-arduino-temp-monitor-fan-led-controller/

#include <avr/sleep.h>

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
const long bounceTime = 1000; // how much time before we consider interrupt as outdated
volatile bool interruptOn = false;

bool manualFan = false;

void setup() {

  // will use bounce of LOW state to maintain fan on
  attachInterrupt(digitalPinToInterrupt(buttonPin), blink, FALLING);

  pinMode(fanSpdPin, OUTPUT);


  // for debug
  pinMode(fanSpdPin, OUTPUT);
  Serial.begin(9600);
}

// go sleep
void sleepNow()  {
  Serial.println("go to sleep");
  Serial.flush();
  // deep sleep
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();

  interrupts();
  attachInterrupt(digitalPinToInterrupt(buttonPin), blink, FALLING);

  // go to sleep
  sleep_mode();
  // awake

  // disable sleep after awake as in tuto (?)
  sleep_disable();

  Serial.println("awakes");

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

  if (interruptOn &&  !manualFan ) {
    manualFan = true;
    Serial.println("start interrupt");
  }


  // check current state of switch
  int val = digitalRead(buttonPin);

  // reset counter while LOW, or enable new wait for counter
  if (val == LOW) {
    previousInterrupt =  millis();
  }
  else {
    attachInterrupt(digitalPinToInterrupt(buttonPin), blink, FALLING);
  }

  // true interrupt, let's do something
  if (interruptOn && currentMillis - previousInterrupt >= bounceTime)
  {
    interruptOn = false;
    manualFan = false;
    Serial.println("stop interrupt");
    sleepNow();
    Serial.println("after sleep");
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
  //Serial.println("interrupt");
}


