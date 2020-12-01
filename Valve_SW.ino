 #include <LiquidCrystal_I2C.h>
#include <Wire.h>

/*
 * Challenges:
 * Probably only want start and stop buttons to be interrupts (not sure if rotary encoder supports)
 * For lower frequencies need a way to poll without busy waiting 
 * First lets try a non-interrupting loop
 */
#define btnIntPin 2
#define encoder0PinA 3
#define encoder0PinB 4
#define encoder0Btn 5
#define startBtn 6
#define stopBtn 7
#define mtr_in1 11
#define mtr_in2 10

//Logic inputs for the DRV8871 unit
//By default the valve is off when there is no voltage. Keep both pins to 0 (low)
//To turn on, set in1 to high, in2 to low (current flows OUT1 to OUT2)

LiquidCrystal_I2C lcd(0x27,20,4);

uint32_t prev_micros;
int frequency;             // in Hz
int freqOffset = 0;        // in Hz
volatile int encoder0Pos = 0;
volatile int valRotary;
int lastValRotary;
int freqDigit;
int freqDigits[] = {0,0,0,1};
int period_in_ms;
int period_in_us;

enum rigState {
  paused,
  active
};
rigState state = paused;

void doEncoder()
{
  if (digitalRead(encoder0PinA) == digitalRead(encoder0PinB))
  {
  encoder0Pos++;
  }
  else
  {
  encoder0Pos--;
  }
  valRotary = encoder0Pos/2.5;
}

void doButtons() {
  if (digitalRead(startBtn)) {
    // pressed start
    state = active;
  }
  if (digitalRead(stopBtn)) {
    // pressed stop 
    state = paused;
  }
}

void printLCD() {
  lcd.clear();
  lcd.setCursor(0,0);
  for (int i = 0; i < 4; ++i) {
    lcd.print(freqDigits[i]);
  }
  lcd.print(" Hz");
  lcd.setCursor(freqDigit,1);
  lcd.print("^");
  lcd.setCursor(10,1);
  if (state == active) {
    lcd.print("ACTIVE");
  } else {
    lcd.print("PAUSED");
  }
}

void checkDigits() {
  if(valRotary > lastValRotary) {
    Serial.print("  CCW");      // decrement
//    if (freqDigit == 0) {
      // only 0-1
//      freqDigits[freqDigit] = (--freqDigits[freqDigit] < 0)? 1 : freqDigits[freqDigit];
//    } else if (freqDigit == 1) {
      // 0-2 or 0-9 depending on digit 0
//      freqDigits[freqDigit] = (--freqDigits[freqDigit] < 0)? 2 : freqDigits[freqDigit];
//    } else {
      freqDigits[freqDigit] = (--freqDigits[freqDigit] < 0)? 9 : freqDigits[freqDigit];
//    }
  }
  if(valRotary < lastValRotary) {
    Serial.print("  CW");       // increment
//    if (freqDigit == 0) {
      // only 0-1
//      freqDigits[freqDigit] = (++freqDigits[freqDigit] > 1)? 0 : freqDigits[freqDigit];
//    } else {
      freqDigits[freqDigit] = (++freqDigits[freqDigit] > 9)? 0 : freqDigits[freqDigit];
//    }
  }

  frequency = freqDigits[0]*1000 + freqDigits[1]*100 + freqDigits[2]*10 + freqDigits[3];
  Serial.print(frequency);
  if (frequency > 1200) {         // trim frequency
    frequency = 1200;
    freqDigits[0] = 1;
    freqDigits[1] = 2;
    freqDigits[2] = 0;
    freqDigits[3] = 0;
  }
  
  period_in_ms = 1000 / (frequency + freqOffset);
  period_in_us = 1000000 / (frequency + freqOffset);
  
  lastValRotary = valRotary;
  
}

// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(mtr_in1, OUTPUT);
  pinMode(mtr_in2, OUTPUT);

  // init serial output
  Serial.begin(9600);

  // init lcd screen
  lcd.init();
  lcd.backlight();
  lcd.setCursor(1,0);
  lcd.print("Fibos");

  //set the initial valve start
  digitalWrite(mtr_in1, LOW);
  digitalWrite(mtr_in2, LOW);

  // init rotary encoder
  pinMode(encoder0PinA, INPUT_PULLUP);
  pinMode(encoder0PinB, INPUT_PULLUP);
  pinMode(encoder0Btn, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoder0PinA), doEncoder, CHANGE);

  // init buttons
  pinMode(btnIntPin, INPUT);
  pinMode(startBtn, INPUT);
  pinMode(stopBtn, INPUT);
  attachInterrupt(digitalPinToInterrupt(btnIntPin), doButtons, CHANGE);
  
  frequency = 300;
  freqDigit = 0;
  delay(1000);
}

// the loop function runs over and over again forever
void loop() {
  int btn = digitalRead(encoder0Btn);
  if (!btn) {
    freqDigit = (++freqDigit >= 4)? 0 : freqDigit;  // rollover to starting digit
  }
  printLCD();

  checkDigits();

  // for 1s of delay, toggle mtr1 output pin
  int period_in_ms = 1000 / (frequency + freqOffset);
  int period_in_us = 1000000 / (frequency + freqOffset);
  if (state) {
    if (frequency > 500) {
      // use us
      for (int i = 0; i < 1000000/period_in_us; ++i){
        digitalWrite(mtr_in1, HIGH);
        delayMicroseconds(period_in_us/2);
        digitalWrite(mtr_in1, LOW);
        delayMicroseconds(period_in_us/2);
      }
    } else {
      for (int i = 0; i < 1000/period_in_ms; ++i){
        digitalWrite(mtr_in1, HIGH);
        delay(period_in_ms/2);
        digitalWrite(mtr_in1, LOW);
        delay(period_in_ms/2);
      }
    }
  } else {
    delay(1000);
  }
}
