#include <toneAC.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>

/*
 * For this iteration the focus is building upon the existing async-loop code except use a library toneAC, 
 * which utilizes two pins to generate tones from 1Hz onwards.
 * ToneAC requires connecting both the pin and ground to the digital pins; this may not be compatible with the motor?
 */
#define btnIntPin 2
#define encoder0PinA 3
#define encoder0PinB 4
#define encoder0Btn 5
//#define mtr_in1 9
//#define mtr_in2 10

#define SW_DEBOUNCE_VAL 10
#define BTN_DEBOUNCE_VAL 50
#define ENC_DEBOUNCE_VAL 7
#define ENC_DELTA_VAL 1

//Logic inputs for the DRV8871 unit
//By default the valve is off when there is no voltage. Keep both pins to 0 (low)
//To turn on, set in1 to high, in2 to low (current flows OUT1 to OUT2)
// toneAC() uses pins 9 and 10 on Arduino UNO

LiquidCrystal_I2C lcd(0x27,20,4);

uint32_t prev_micros;
int frequency;             // in Hz
int freqDelta = 50;        // in Hz
volatile int encoder0Pos = 0;
volatile int valRotary;
int lastValRotary;
int freqDigit;
int freqDigits[] = {0,0,0,1};
volatile static bool digitsChanged = false;

volatile static bool state = 0; // 0: off 1: on
volatile static bool stateChanged = false;
static int toneFreq = 0;
static int prevTime = 0;
static int period_in_ms = 0;

volatile static int btn_debounce = 0;
static int sw_debounce = 0;
volatile static int encoder_debounce;

void doEncoder()
{
  if(encoder_debounce == 0) {
    if (digitalRead(encoder0PinA) == digitalRead(encoder0PinB))
    {
    encoder0Pos++;
    }
    else
    {
    encoder0Pos--;
    }
    valRotary = encoder0Pos/2;
    digitsChanged = true;
    encoder_debounce = ENC_DEBOUNCE_VAL;
  }
}

void doButtons() {
  // pressed start: toggle
  if (btn_debounce == 0) {
    state = !state;
    stateChanged = true;
    btn_debounce = BTN_DEBOUNCE_VAL;
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
  if (state) {
    lcd.print("ACTIVE");
  } else {
    lcd.print("PAUSED");
  }
}

bool checkDigits() {
  int prevFreq = frequency;
  Serial.print("val, last: ");
  Serial.print(valRotary);
  Serial.print(" ");
  Serial.println(lastValRotary);
  if(valRotary - lastValRotary >= ENC_DELTA_VAL) {
//    Serial.print("  CCW");      // decrement
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
  if(lastValRotary - valRotary >= ENC_DELTA_VAL) {
//    Serial.print("  CW");       // increment
//    if (freqDigit == 0) {
      // only 0-1
//      freqDigits[freqDigit] = (++freqDigits[freqDigit] > 1)? 0 : freqDigits[freqDigit];
//    } else {
      freqDigits[freqDigit] = (++freqDigits[freqDigit] > 9)? 0 : freqDigits[freqDigit];
//    }
  }

  frequency = freqDigits[0]*1000 + freqDigits[1]*100 + freqDigits[2]*10 + freqDigits[3];
//  Serial.print(frequency);
  if (frequency > 1200) {         // trim frequency
    frequency = 1200;
    freqDigits[0] = 1;
    freqDigits[1] = 2;
    freqDigits[2] = 0;
    freqDigits[3] = 0;
  }
  if (frequency < 1) {         // trim frequency
  frequency = 1;
  freqDigits[0] = 0;
  freqDigits[1] = 0;
  freqDigits[2] = 0;
  freqDigits[3] = 1;
}
  period_in_ms = 1000/frequency;
  lastValRotary = valRotary;
  return (prevFreq != frequency); // return true if frequency has changed
}

// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
//  pinMode(mtr_in1, OUTPUT);
//  pinMode(mtr_in2, OUTPUT);

  // init serial output
  Serial.begin(9600);

  // init lcd screen
  lcd.init();
  lcd.backlight();
  lcd.setCursor(1,0);
  lcd.print("Fibos");

  // init rotary encoder
  pinMode(encoder0PinA, INPUT_PULLUP);
  pinMode(encoder0PinB, INPUT_PULLUP);
  pinMode(encoder0Btn, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoder0PinA), doEncoder, CHANGE);

  // init button
  pinMode(btnIntPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(btnIntPin), doButtons, FALLING);
  
  frequency = 300;
  freqDigit = 0;
  delay(1000);
  printLCD();
}

// the loop function runs over and over again forever
void loop() {
  int sw = digitalRead(encoder0Btn);
  if (!sw) {
    if (sw_debounce == 0) {
      // new instance of button press
      sw_debounce = SW_DEBOUNCE_VAL;
      freqDigit = (++freqDigit >= 4)? 0 : freqDigit;  // rollover to starting digit
    }
  }
  if (btn_debounce > 0) {
    btn_debounce --;
  }
  if (sw_debounce > 0) {
    sw_debounce --;
  }
  if (encoder_debounce > 0) {
    encoder_debounce --;
  }

  // only print lcd if state or digits or digit select changed
  if (!sw || digitsChanged || stateChanged) { 
    checkDigits();
    printLCD();
    digitsChanged = false;
    stateChanged = false;
  }
  if (toneFreq != frequency && state) {
    // only change tone if freq changed from what is currently playing and play state
    toneFreq = frequency;
    toneAC(toneFreq, true);
  }
  
  if (!state) {
    toneAC();
    toneFreq = 0;
  }
}
