
// bigup to http://www.plastibots.com/index.php/2016/02/27/lulzbot-mini-arduino-temp-monitor-fan-led-controller/
// http://playground.arduino.cc/Learning/ArduinoSleepCode
// http://donalmorrissey.blogspot.fr/2010/04/sleeping-arduino-part-5-wake-up-via.html
// http://manicdee.livejournal.com/97726.html

#include <avr/sleep.h>
#include <avr/power.h>


#define fanSpdPin 3// was5  //PWM out to control fan speed (blue wire on 4 pin fans) 

#define ledPin 13 // default LED

#define buttonPin 2 // for manual trigger of fan

// reduce fan speed
// TODO: modulate according to actual RPM, especially when powered by more than 12v
#define maxFanSpeed 225

int fanSpeed = 0;

// switch state
volatile unsigned long previousMillis = 0;
// sleep cycles, see watchdog duration
const int fanOnCycle = 1;
const int fanOffCycle = 1;
// current cycle reguarding watchdog (so time passes by even when on interrupt)
bool cycleOn = false;

// if fan actually on/off
bool fanOn = false;

// watchdoc cycles, starting just prior to on
volatile int f_wdt = fanOffCycle;

// handle interrupt
volatile unsigned long previousInterrupt = 0;
const long bounceTime = 1000; // how much time before we consider interrupt as outdated
volatile bool interruptOn = false;

bool manualFan = false;

// currently asleep or not
bool asleep = false;

// enable / disable output to serial port
const bool debug = true;

// This is executed when watchdog timed out.
ISR(WDT_vect)
{
  if (asleep) {
    debugMsg("out of sleep from timer");
  }
  else {
    debugMsg("timer but arleady awake");
  }

  f_wdt++;
}

void setup() {

  // will use bounce of LOW state to maintain fan on
  attachInterrupt(digitalPinToInterrupt(buttonPin), blink, FALLING);

  pinMode(fanSpdPin, OUTPUT);

  // for debug
  pinMode(ledPin, OUTPUT);
  if (debug) {
    Serial.begin(9600);
  }


  /*** Setup the WDT ***/
  /* Clear the reset flag. */
  MCUSR &= ~(1 << WDRF);
  /* In order to change WDE or the prescaler, we need to
     set WDCE (This will allow updates for 4 clock cycles).
  */
  WDTCSR |= (1 << WDCE) | (1 << WDE);
  /* set new watchdog timeout prescaler value */
  WDTCSR = 1 << WDP0 | 1 << WDP3; /* 8.0 seconds */
  // WDTCSR = 1 << WDP3; /* 4.0 seconds */
  // WDTCSR = 1 << WDP1 | 1 << WDP2; /* 1.0 seconds */
  /* Enable the WD interrupt (note no reset). */
  WDTCSR |= _BV(WDIE);

  // disable everything not needed
  power_adc_disable(); // ADC converter
  power_spi_disable(); // SPI
  // power_timer0_disable();// Timer 0, used for millis() and delay()
  power_timer1_disable();// Timer 1
  power_timer2_disable();// Timer 2, PWM 3 & 11 -- PMW re-enable if needed while controlling fanspeed
  power_twi_disable(); // TWI (I2C)

  // get rid of serial port if not needed
  if (!debug) {
    power_usart0_disable();// Serial (USART)
  }
}

void debugMsg(String msg) {
  if (debug) {
    Serial.println(msg);
  }
}

// go sleep
void sleepNow()  {
  debugMsg("go to sleep");
  Serial.flush();

  // if going sleep on ON phase, need PWM
  if (cycleOn) {
    set_sleep_mode(SLEEP_MODE_PWR_SAVE);
  }
  // otherwise, can go really deep
  else {
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  }
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

  debugMsg("awakes");

}

// handle interrupt from switch
void checkSwitch() {
  unsigned long currentMillis = millis();

  // interrupt has been set, will enable fan
  if (interruptOn &&  !manualFan ) {
    manualFan = true;
    fanSet(true);
    debugMsg("start interrupt");
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

  // for real we left LOW state (and interrupt), leave it to timer
  if (interruptOn && currentMillis - previousInterrupt >= bounceTime)
  {
    interruptOn = false;
    manualFan = false;
  }

}

void checkTimer() {

  if (cycleOn && f_wdt >= fanOnCycle) {
    debugMsg("=== going down ===");
    f_wdt = 0;
    cycleOn = !cycleOn;
  }

  if (!cycleOn && f_wdt >= fanOffCycle) {
    debugMsg("=== going up ===");
    f_wdt = 0;
    cycleOn = !cycleOn;
  }

  // only sleep and mess up with fan if no manual interrupt going on
  if (!interruptOn) {
    // need to change state
    if (cycleOn  != fanOn) {
      fanSet(cycleOn);
      debugMsg("-- change fan");
    }
    debugMsg("periodic mode, go to sleep");
    sleepNow();
    debugMsg("after sleep in checkTimer");
  }

}

void loop() {
  noInterrupts();

  // first check manual switch
  checkSwitch();

  // if not manual normal state will be resolve in checkTimer()
  checkTimer();

  interrupts();
}

void fanSet(bool flag) {
  if (flag) {
    setSpeed(maxFanSpeed);
    digitalWrite(13, HIGH);
  }
  else {
    setSpeed(0);
    digitalWrite(13, LOW);
  }
  fanOn = flag;
}

// handle fanspeed, enable PWM if needed
void setSpeed(int fspeed) {
  if (fspeed == 0 || fspeed == 255) {
    // for full OFF or ON, do not need PMW
    power_timer2_disable();
  }
  else {
    power_timer2_enable();
  }
  analogWrite(fanSpdPin, fspeed);
}


void blink() {
  interruptOn = true;
  detachInterrupt(digitalPinToInterrupt(buttonPin));
  previousInterrupt =  millis();
  //debugMsg("interrupt");
}


