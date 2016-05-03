
// bigup to http://www.plastibots.com/index.php/2016/02/27/lulzbot-mini-arduino-temp-monitor-fan-led-controller/

#define fanSpdPin 3// was5  //PWM out to control fan speed (blue wire on 4 pin fans) 

#define hallsensor 2 //The pin location of the fan PWM sensor (green wire on 4 pin fans)

// reduce fan speed
// TODO: modulate according to actual RPM, especially when powered by more than 12v
#define maxFanSpeed 200

int fanSpeed = 0;

// handling hallsensor
volatile byte half_revolutions; // will be updated by interrupt
unsigned int rpm;
unsigned long timeold;

// switch state
unsigned long previousMillis = 0;
const long interval = 4000;

bool fanOn = true;

void setup() {
  pinMode(fanSpdPin, OUTPUT);

  //pinMode(hallsensor, OUTPUT);
  //digitalWrite(hallsensor, HIGH);  //needs pull up enabled to be readable.
  attachInterrupt(digitalPinToInterrupt(hallsensor), rpm_fun, RISING);  //for pin 2 - hall effect sensor on FAN
  half_revolutions = 0;
  rpm = 0;
  timeold = 0;

  Serial.begin(9600);
}

void loop() {

  unsigned long currentMillis = millis();

  //Get RPM readings
  if (half_revolutions >= 20)
  {
    Serial.println("go");
    //Update RPM every 20 counts, increase this for better RPM resolution,
    //decrease for faster update
    rpm = 30 * 1000 / (millis() - timeold) * half_revolutions;
    timeold = millis();
    half_revolutions = 0;
    Serial.println(rpm, DEC);
  }

  if (currentMillis - previousMillis >= interval)
  {
    Serial.println("interval");
    // save the last time you blinked the LED
    previousMillis = currentMillis;
    fanOn = !fanOn;
    if (fanOn) {
      setSpeed(maxFanSpeed);
    }
    else {
      setSpeed(0);
    }
  }

}

void setSpeed(int fspeed) {
  analogWrite(fanSpdPin, fspeed);
}

void rpm_fun()
{
  half_revolutions++;
  //Serial.println("up");
  //Each rotation, this interrupt function is run twice
}
