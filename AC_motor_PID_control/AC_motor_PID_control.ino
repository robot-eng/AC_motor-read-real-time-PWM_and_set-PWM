#include <avr/io.h>
#include <avr/interrupt.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Rotary.h>
#include <PID_v1.h>

#define TACHO 3            // tacho signals input pin
#define DETECT 2           // zero cross detect pin
#define GATE 17            // TRIAC gate pin
#define RANGE1 9           // range one switch pin
#define RANGE2 10          // range two switch pin
#define BUTTON 4           // rottary encoder button pin
#define RELAY 5            // relay pin
#define PULSE 2            // number of triac trigger pulse width counts. One count is 16 microseconds
#define TACHOPULSES 8      // number of pulses per revolution 

unsigned int RPM;                   // real rpm variable
unsigned int count;                 // tacho pulses count variable
unsigned int lastcount = 0;         // additional tacho pulses count variable
unsigned long lastcounttime = 0;
unsigned long lastflash;
unsigned long lastpiddelay = 0;
unsigned long previousMillis = 0;
unsigned long lastDebounceTime = 0;

const int sampleRate = 1;           // Variable that determines how fast our PID loop
const int rpmcorrection = 86;       // sito kazkodel reikia, kad realus rpm atitiktu matuojamus
const int lcdinterval = 2000;       // lcd refresh interval in milliseconds
const int protection = 2000;        // protection will switch on when real rpm exceeds desired by value
const int debounceDelay = 50;       // the debounce time; increase if the output flickers
const int minoutputlimit = 80;      // limit of PID output
const int maxoutputlimit = 540;     // limit of PID output
const int mindimminglimit = 80;     // the shortest delay before triac fires
const int maxdimminglimit = 625;    // for 60Hz will be 520
const int minrpmR1 = 300;           // min RPM of the range 1
const int maxrpmR1 = 1500;          // max RPM of the range 1
const int minrpmR2 = 1500;          // min RPM of the range 2
const int maxrpmR2 = 3500;          // max RPM of the range 2
const int risetime = 100;           // RPM rise time delay in microseconds (risetime x RPM)

int dimming = 540;                  // this should be the same as maxoutputlimit
int counterR1;                      // desired RPM counter for range 1
int counterR2;                      // desired RPM counter for range 2
int desiredRPM;
int tempcounter = 100;

byte range;
byte lastRangeState = 0;
byte relayState = LOW;              // the current state of the relay pin
byte buttonState;                   // the current reading from the input pin
byte lastButtonState = HIGH;        // the previous reading from the input pin

bool loopflag = false;              // flag for soft start
bool startflag = false;             // flag for motor start delay
bool runflag = false;               // flag for motor running state

double Setpoint, Input, Output;       // define PID variables
double sKp = 0.1, sKi = 0.2, sKd = 0; // PID tuning parameters for starting motor
double rKp = 0.25, rKi = 1, rKd = 0;  // PID tuning parameters for runnig motor

LiquidCrystal_I2C lcd(0x27, 16, 2);   // set the LCD address to 0x26 for a 16 chars and 2 line display
Rotary r = Rotary(12, 11);            // define rottary encoder and pins
PID myPID(&Input, &Output, &Setpoint, sKp, sKi, sKd, DIRECT); // define PID variables and parameters

void setup() {
  Serial.begin(115200);
  // set up pins
  pinMode(BUTTON, INPUT);             // set the button pin
  pinMode(RELAY, OUTPUT);             // set the relay  pin
  pinMode(DETECT, INPUT);             // set the zero cross detect pin
  pinMode(GATE, OUTPUT);              // set the TRIAC gate control pin
  pinMode(TACHO, INPUT);              // set the tacho pulses detect pin
  pinMode(RANGE1, INPUT);             // set the range 1 switch pin
  pinMode(RANGE2, INPUT);             // set the range 1 switch pin
  digitalWrite(BUTTON, HIGH);         // turn on pullup resistors
  digitalWrite(RANGE1, HIGH);         // turn on pullup resistors
  digitalWrite(RANGE2, HIGH);         // turn on pullup resistors
  digitalWrite(RELAY, relayState);    // initialize relay output

  counterR1 = minrpmR1;               // assign start value for range 1
  counterR2 = minrpmR2;               // assign start value for range 2
  Input = 200;                        // asiign initial value for PID
  Setpoint = 200;                     // asiign initial value for PID

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(minoutputlimit, maxoutputlimit);
  myPID.SetSampleTime(sampleRate);    // Sets the sample rate

  // set up Timer1
  OCR1A = 100;                        // initialize the comparator
  TIMSK1 = 0x03;                      // enable comparator A and overflow interrupts
  TCCR1A = 0x00;                      // timer control registers set for
  TCCR1B = 0x00;                      // normal operation, timer disabled

  // set up zero crossing interrupt IRQ0 on pin 2.
  // set up tacho sensor interrupt IRQ1 on pin3
  attachInterrupt(0, zeroCrossingInterrupt, RISING);
  attachInterrupt(1, tacho, FALLING);

  lcd.init();        // initialize the lcd
  lcd.backlight();   // turn on the backlight

  // check the RPM range state at startup and display it
  int rangeOne = digitalRead(RANGE1);
  int rangeTwo = digitalRead(RANGE2);

  if (rangeOne == 1 && rangeTwo == 1) {
    range = 0;
    range0();
  }
  if (rangeOne == 0 && rangeTwo == 1) {
    range = 1;
    RPMrange1();
  }
  if (rangeOne == 1 && rangeTwo == 0) {
    range = 2;
    RPMrange2();
  }
}

// Interrupt Service Routines
void zeroCrossingInterrupt() { // zero cross detect
  TCCR1B = 0x04;               // start timer with divide by 256 input
  TCNT1 = 0;                   // reset timer - count from zero
  OCR1A = dimming;             // set the compare register brightness desired.
}


ISR(TIMER1_COMPA_vect) {       // comparator match
  if (startflag == true) {     // flag for start up delay
    digitalWrite(GATE, HIGH);  // set TRIAC gate to high
    TCNT1 = 65536 - PULSE;     // trigger pulse width
  }
}

ISR(TIMER1_OVF_vect) {         // timer1 overflow
  digitalWrite(GATE, LOW);     // turn off TRIAC gate
  TCCR1B = 0x00;               // disable timer stops unintended triggers
}

// RPM counting routine
void tacho() {
  count++;
  unsigned long time = micros() - lastflash;
  float time_in_sec  = ((float)time + rpmcorrection) / 1000000;
  float prerpm = 60 / time_in_sec;
  RPM = prerpm / TACHOPULSES;
  lastflash = micros();
}

void loop() {

  // check the RPM range switch state
  int rangeOne = digitalRead(RANGE1);
  int rangeTwo = digitalRead(RANGE2);

  if (rangeOne == 1 && rangeTwo == 1) {
    range = 0;
  }
  if (rangeOne == 0 && rangeTwo == 1) {
    range = 1;
    desiredRPM = counterR1;
  }
  if (rangeOne == 1 && rangeTwo == 0) {
    range = 2;
    desiredRPM = counterR2;
  }

  // check the RPM range switch state changes
  if (range != lastRangeState) {
    if (range == 0) {
      range0();
      runflag = false;
      startflag = false;            // flag to turn off triac before relay turns off
      delay (300);                  // delay to prevent sparks on relay contacts
      digitalWrite(RELAY, LOW);
      relayState = LOW;
    }
    if (range == 1) {
      RPMrange1();
    }
    if (range == 2) {
      RPMrange2();
    }
    if (relayState == LOW && range != 0) {
      motorStateStop();
    }
  }
  lastRangeState = range;

  // check the start / stop button state
  if (range != 0) {
    int reading = digitalRead(BUTTON); // read the state of the switch into a local variable:
    if (reading != lastButtonState) {  // If the switch changed, due to noise or pressing
      lastDebounceTime = millis();     // reset the debouncing timer
    }
    if ((millis() - lastDebounceTime) > debounceDelay) {
      if (reading != buttonState) {     // if the button state has changed:
        buttonState = reading;
        if (buttonState == LOW) {       // only toggle the relay if the new button state is LOW
          relayState = !relayState;
          if (relayState == HIGH) {
            loopflag = true;
            digitalWrite(RELAY, relayState); // set the Relay:
            delay (300);                     // delay to prevent sparks on relay contacts
            startflag = true;                // flag to start motor
          }
          if (relayState == LOW) {
            Setpoint = 200;
            Input = 200;
            runflag = false;
            startflag = false;
            delay (300);                     // delay to prevent sparks on relay contacts
            digitalWrite(RELAY, relayState); // set the Relay:
            motorStateStop();
          }
        }
      }
    }
    lastButtonState = reading;            // save the reading. Next time through the loop, it'll be the lastButtonState:
  }

  //rotarry encoder process
  unsigned char result = r.process();
  if (range == 1 && result == DIR_CW) {
    if (counterR1 >= 500)
    {
      counterR1 += 50;
    }
    else counterR1 += 20;
    if (counterR1 >= maxrpmR1) {
      counterR1 = maxrpmR1;
    }
    RPMrange1();
  }

  else if (range == 1 && result == DIR_CCW) {
    if (counterR1 <= 500)
    {
      counterR1 -= 20;
    }
    else counterR1 -= 50;
    if (counterR1 <= minrpmR1) {
      counterR1 = minrpmR1;
    }
    RPMrange1();
  }

  if (range == 2 && result == DIR_CW) {
    counterR2 += 100;
    if (counterR2 >= maxrpmR2) {
      counterR2 = maxrpmR2;
    }
    RPMrange2();
  }
  else if (range == 2 && result == DIR_CCW) {
    counterR2 -= 100;
    if (counterR2 <= minrpmR2) {
      counterR2 = minrpmR2;
    }
    RPMrange2();
  }

  //soft start
  if (loopflag == true) {
    myPID.SetTunings(sKp, sKi, sKd);        // Set the PID gain constants and start
    int i = (desiredRPM - tempcounter);
    for (int j = 1; j <= i; j++) {
      Input = RPM;
      Setpoint = tempcounter;
      myPID.Compute();
      dimming = map(Output, minoutputlimit, maxoutputlimit, maxoutputlimit, minoutputlimit); // inverse the output
      dimming = constrain(dimming, mindimminglimit, maxdimminglimit);     // check that dimming is in 20-625 range
      tempcounter++;
      delayMicroseconds (risetime);
    }
    if (tempcounter >= desiredRPM) {
      lastcounttime = millis();
      lastpiddelay = millis();
      loopflag = false;
      runflag = true;
      tempcounter = 100;
    }
  }

  // normal motor running state
  if (relayState == HIGH && loopflag == false) {
    unsigned long piddelay = millis();

    if ((piddelay - lastpiddelay) > 1000) {     // delay to switch PID values. Prevents hard start
      myPID.SetTunings(rKp, rKi, rKd);          // Set the PID gain constants and start
      lastpiddelay = millis();
    }

    Input = RPM;
    Setpoint = desiredRPM;
    myPID.Compute();
    dimming = map(Output, minoutputlimit, maxoutputlimit, maxoutputlimit, minoutputlimit); // reverse the output
    dimming = constrain(dimming, mindimminglimit, maxdimminglimit);     // check that dimming is in 20-625 range
  }
  // diagnose a fault and turn on protection

  unsigned long counttime = millis();
  if (counttime - lastcounttime >= 1000) {
    if (count == 0 && relayState == HIGH && runflag == true) {
      startflag = false;            // flag to turn off triac before relay turns off
      delay (300);                  // delay to prevent sparks on relay contacts
      digitalWrite(RELAY, LOW);
      relayState = LOW;
      stuckerror();
    }
    lastcount = count;
    count = 0;
    lastcounttime = millis();
  }

  //reset rpm after motor stops
  if (count == 0 && relayState == LOW) {
    RPM = 0;
  }

  // protection against high rpm. i e triac damage
  if (relayState == HIGH && RPM > desiredRPM + protection) {
    startflag = false;            // flag to turn off triac before relay turns off
    delay (300);                  // delay to prevent sparks on relay contacts
    digitalWrite(RELAY, LOW);
    relayState = LOW;
    exceederror();
  }
  // real RPM display
  if (relayState == HIGH && range != 0) {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= lcdinterval) {
      previousMillis = currentMillis;
      int rpmdisplay = RPM;
      lcd.setCursor(0, 1);
      lcd.print("Real RPM:   ");
      if (rpmdisplay >= 1000) {
        lcd.setCursor(12, 1);
        lcd.print(rpmdisplay);
      }
      else if (RPM < 1000) {
        lcd.setCursor(12, 1);
        lcd.print(" ");
        lcd.setCursor(13, 1);
        lcd.print(rpmdisplay);
      }
    }
  }
}

void range0() {
  lcd.setCursor(0, 0);
  lcd.print(" Please  select ");
  lcd.setCursor(0, 1);
  lcd.print(" the RPM range! ");
}

void motorStateStop() {
  lcd.setCursor(0, 1);
  lcd.print ("  Press  START  ");

}

void RPMrange1() {
  lcd.setCursor(0, 0);
  lcd.print("R1 RPM set: ");
  if (counterR1 >= 1000) {
    lcd.setCursor(12, 0);
    lcd.print(counterR1);
  }
  else {
    lcd.setCursor(12, 0);
    lcd.print(" ");
    lcd.setCursor(13, 0);
    lcd.print(counterR1);
  }
}

void RPMrange2() {
  lcd.setCursor(0, 0);
  lcd.print("R2 RPM set: ");
  lcd.setCursor(12, 0);
  lcd.print(counterR2);
}

void exceederror() {
  lcd.clear();
  while (1) {
    lcd.setCursor(5, 0);
    lcd.print("ERROR!");
    lcd.setCursor(2, 1);
    lcd.print("TRIAC DAMAGE");
  }
}

void stuckerror() {
  lcd.clear();
  while (1) {
    lcd.setCursor(5, 0);
    lcd.print("ERROR!");
    lcd.setCursor(2, 1);
    lcd.print("MOTOR STUCK!");
  }
}
