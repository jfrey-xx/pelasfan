
// bigup to http://www.plastibots.com/index.php/2016/02/27/lulzbot-mini-arduino-temp-monitor-fan-led-controller/
// http://playground.arduino.cc/Learning/ArduinoSleepCode
// http://donalmorrissey.blogspot.fr/2010/04/sleeping-arduino-part-5-wake-up-via.html

#include <avr/sleep.h>

#define fanSpdPin 3// was5  //PWM out to control fan speed (blue wire on 4 pin fans) 

#define ledPin 13 // default LED

#define buttonPin 2 // for manual trigger of fan

// reduce fan speed
// TODO: modulate according to actual RPM, especially when powered by more than 12v
#define maxFanSpeed 200

int fanSpeed = 0;

// switch state
volatile unsigned long previousMillis = 0;
// sleep cycles, see watchdog duration
const int fanOnCycle = 2;
const int fanOffCycle = 4;
// current cycle reguarding watchdog (so time passes by even when on interrupt)
bool cycleOn = true;

// if fan actually on/off
bool fanOn = true;

// handle interrupt
volatile unsigned long previousInterrupt = 0;
const long bounceTime = 1000; // how much time before we consider interrupt as outdated
volatile bool interruptOn = false;

bool manualFan = false;

// currently asleep or not
bool asleep = false;

// watchdoc cycles
volatile int f_wdt = 0;

// This is executed when watchdog timed out.
ISR(WDT_vect)
{
  if (asleep) {
    Serial.println("out of sleep from timer");
  }
  else {
    Serial.println("timer but arleady awake");
  }

  f_wdt++;
}

void setup() {

  // will use bounce of LOW state to maintain fan on
  attachInterrupt(digitalPinToInterrupt(buttonPin), blink, FALLING);

  pinMode(fanSpdPin, OUTPUT);


  // for debug
  pinMode(fanSpdPin, OUTPUT);
  Serial.begin(9600);


  /*** Setup the WDT ***/
  /* Clear the reset flag. */
  MCUSR &= ~(1 << WDRF);
  /* In order to change WDE or the prescaler, we need to
     set WDCE (This will allow updates for 4 clock cycles).
  */
  WDTCSR |= (1 << WDCE) | (1 << WDE);
  /* set new watchdog timeout prescaler value */
  // WDTCSR = 1 << WDP0 | 1 << WDP3; /* 8.0 seconds */
  // WDTCSR = 1 << WDP3; /* 4.0 seconds */
  WDTCSR = 1 << WDP1 | 1 << WDP2; /* 1.0 seconds */
  /* Enable the WD interrupt (note no reset). */
  WDTCSR |= _BV(WDIE);
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
  asleep = true;
  sleep_mode();
  // awake
  asleep = false;

  // disable sleep after awake as in tuto (?)
  sleep_disable();

  Serial.println("awakes");

}

// handle interrupt from switch
void checkSwitch() {
  unsigned long currentMillis = millis();


  if (interruptOn &&  !manualFan ) {
    manualFan = true;
    Serial.println("start interrupt");
  }


  // check current state of switch
  int val = digitalRead(buttonPin);

  // reset counter while LOW, or enable new wait for counter
  if (val == LOW) {
    previousInterrupt =  currentMillis;
  }
  else {
    attachInterrupt(digitalPinToInterrupt(buttonPin), blink, FALLING);
  }

  // true interrupt, let's do something
  if (interruptOn && currentMillis - previousInterrupt >= bounceTime)
  {
    interruptOn = false;
    manualFan = false;
    Serial.println("stop interrupt, go to sleep");
    sleepNow();
    Serial.println("after sleep");
  }

}

void checkTimer() {

  if (cycleOn && f_wdt >= fanOnCycle) {
    Serial.println("=== going down ===");
    f_wdt = 0;
    cycleOn = !cycleOn;
    // only mess up fan if on automatic
    if (!interruptOn) {
      fanSetOn();
    }
  }

  if (!cycleOn && f_wdt >= fanOffCycle) {
    Serial.println("=== going up ===");
    f_wdt = 0;
    cycleOn = !cycleOn;
    // only mess up fan if on automatic
    if (!interruptOn) {
      fanSetOff();
    }
  }

  // only sleep if no manual interrupt going on
  if (!interruptOn) {
    Serial.println("periodic mode, go to sleep");
    sleepNow();
    Serial.println("after sleep in checkTimer");
  }

}

void loop() {

  noInterrupts();


  //  unsigned long currentMillis = millis();
  //
  //
  //  if (currentMillis - previousMillis >= interval)
  //  {
  //    Serial.println("interval");
  //    // save the last time state was switched
  //    previousMillis = currentMillis;
  //    fanToggle();
  //  }

  checkTimer();


  interrupts();

}

void fanToggle() {
  fanOn = !fanOn;
  if (fanOn) {
    fanSetOn();
  }
  else {
    fanSetOff();
  }
}

void fanSetOn() {
  setSpeed(maxFanSpeed);
  digitalWrite(13, HIGH);
}

void fanSetOff() {
  setSpeed(0);
  digitalWrite(13, LOW);
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


