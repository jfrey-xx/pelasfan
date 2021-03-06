
// bigup to http://www.plastibots.com/index.php/2016/02/27/lulzbot-mini-arduino-temp-monitor-fan-led-controller/
// http://playground.arduino.cc/Learning/ArduinoSleepCode
// http://donalmorrissey.blogspot.fr/2010/04/sleeping-arduino-part-5-wake-up-via.html
// http://manicdee.livejournal.com/97726.html
// https://github.com/lopenlab/atmega_power_save/blob/master/interrupt_power_save.ino

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
// sleep cycles, see watchdog duration -- 30minutes cycles with 8s watchdog
const int fanOnCycle =  4;
const int fanOffCycle = 221;
// current cycle reguarding watchdog (so time passes by even when on interrupt)
bool cycleOn = false;

// if fan actually on/off
bool fanOn = false;

// watchdoc cycles, starting just prior to on
volatile int f_wdt = fanOffCycle;

// handle interrupt
volatile unsigned long previousInterrupt = 0;
const long bounceTime = 2000; // how much time before we consider interrupt as outdated
volatile bool interruptOn = false;

bool manualFan = false;

// currently asleep or not
bool asleep = false;

// enable / disable output to serial port
const bool debug = false;

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

/*
   Prescale values:
   0 2K (2048) cycles 16 ms
   1 4K (4096) cycles 32 ms
   2 8K (8192) cycles 64 ms
   3 16K (16384) cycles 0.125 s
   4 32K (32768) cycles 0.25 s
   5 64K (65536) cycles 0.5 s
   6 128K (131072) cycles 1.0 s
   7 256K (262144) cycles 2.0 s
   8 512K (524288) cycles 4.0 s
   9 1024K (1048576) cycles 8.0 s
   Other - reserved.
*/

/* function originates in the CitizenWatt project */
void watchdog_setup(uint8_t prescale)
{
  prescale = min(9, prescale);

  /* 3 least significant bits of prescalar go to WDTCSR firectly,
     first bit goes to WDP3 */
  uint8_t wdtcsr = prescale & ((1 << 0) | (1 << 1) | (1 << 2));
  if (prescale & (1 << 3))
    wdtcsr |= _BV(WDP3);

  /* allow watchdog configuration change */
  WDTCSR = _BV(WDCE) | _BV(WDE);
  /* Set prescale */
  WDTCSR = wdtcsr;
}

void watchdog_int_enable()
{
  uint8_t wdtcsr = WDTCSR;
  WDTCSR = _BV(WDCE) | _BV(WDE);
  /* clear the WD interrupt enable bit */
  WDTCSR = wdtcsr | _BV(WDIE);

}

void watchdog_int_disable()
{
  uint8_t wdtcsr = WDTCSR;
  WDTCSR = _BV(WDCE) | _BV(WDE);
  /* clear the WD interrupt enable bit */
  WDTCSR = wdtcsr & ~_BV(WDIE);
}

void setup() {

  // will use bounce of LOW state to maintain fan on
  // attachInterrupt(digitalPinToInterrupt(buttonPin), blink, FALLING);

  pinMode(fanSpdPin, OUTPUT);

  // for debug
  pinMode(ledPin, OUTPUT);
  if (debug) {
    Serial.begin(9600);
    while (!Serial) {
      ; // wait for serial port to connect. Needed for Leonardo only
    }
  }

  /* Watchdog consumed about 5 uA. Comment out to get down to ~0.5 uA*/
  watchdog_setup(9);
  watchdog_int_enable();

  // disable everything not needed
  power_adc_disable(); // ADC converter
  power_spi_disable(); // SPI
  // power_timer0_disable();// Timer 0, used for millis() and delay()
  power_timer1_disable();// Timer 1
  // power_timer2_disable();// Timer 2, PWM 3 & 11
  power_twi_disable(); // TWI (I2C)

  // get rid of serial port if not needed
  if (!debug) {
    power_usart0_disable();// Serial (USART)
  }

  debugMsg("setup done");
}

void debugMsg(String msg) {
  if (debug) {
    Serial.println(msg);
    Serial.flush();
  }
}

// go sleep
void sleepNow()  {
  debugMsg("go to sleep");

  // if going sleep on ON phase, need PWM
  if (cycleOn) {
    debugMsg("light sleep");
    set_sleep_mode(SLEEP_MODE_PWR_SAVE);
  }
  // otherwise, can go really deep
  else {
    debugMsg("deep sleep");
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  }


  sleep_enable();
  attachInterrupt(digitalPinToInterrupt(buttonPin), blink, FALLING);

  // go to sleep
  asleep = true;
  sleep_mode();
  // awake
  detachInterrupt(digitalPinToInterrupt(buttonPin));
  asleep = false;

  // disable sleep after awake as in tuto (?)
  sleep_disable();

  debugMsg("awakes");

}

// handle interrupt from switch
void checkSwitch() {
  unsigned long currentMillis = millis();

  // check current state of switch
  int val = digitalRead(buttonPin);

  // reset counter while LOW, or enable new wait for counter
  if (val == LOW) {
    previousInterrupt =  currentMillis;
    interruptOn = true;
  }

  // interrupt has been set, will enable fan
  if (interruptOn &&  !manualFan ) {
    manualFan = true;
    fanSet(true);
    debugMsg("start interrupt");
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
  // first check manual switch
  checkSwitch();

  // if not manual normal state will be resolve in checkTimer()
  checkTimer();
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

// handle fanspeed
void setSpeed(int fspeed) {
  analogWrite(fanSpdPin, fspeed);
}


void blink() {
  interruptOn = true;
  previousInterrupt =  millis();
  // debugMsg("interrupt");
  detachInterrupt(digitalPinToInterrupt(buttonPin));
}


