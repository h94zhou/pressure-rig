#include <Tone.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>

#define btnIntPin 2
#define encoder0PinA 3
#define encoder0PinB 4
#define encoder0Btn 5
#define mtr_in1 11
#define mtr_in2 10
#define PIN_UNUSED 13

#define SW_DEBOUNCE_VAL 10
#define BTN_DEBOUNCE_VAL 50
#define ENC_DEBOUNCE_VAL 50
#define ENC_DELTA_VAL 1

//Logic inputs for the DRV8871 unit
//By default the valve is off when there is no voltage. Keep both pins to 0 (low)
//To turn on, set in1 to high, in2 to low (current flows OUT1 to OUT2)

LiquidCrystal_I2C lcd(0x27,20,4);

int frequency = 1;              // in Hz
int freqDelta = 0;              // in Hz- manual frequency offset
int freqDigit;
int freqDigits[] = {0,0,0,1};
volatile static bool digitsChanged = false;

volatile static bool state = 0;   // 0: off 1: on
volatile static bool stateChanged = false;

static int toneFreq = 0;
Tone tone2;
Tone tone1;

volatile int encoder0Pos = 0;
volatile int valRotary;
int lastValRotary;

volatile static int btn_debounce = 0;
static int sw_debounce = 0;
volatile static int encoder_debounce = 0;

/*
 * ISR (Interrupt Service Routine) triggered by encoder pin encoderPinA 
 */
void doEncoder()
{
  // only trigger if finished debouncing
  if(encoder_debounce == 0) {
    if (digitalRead(encoder0PinA) == digitalRead(encoder0PinB))
    {
    encoder0Pos++;
    }
    else
    {
    encoder0Pos--;
    }
    valRotary = encoder0Pos/2.5;
    digitsChanged = true;
    encoder_debounce = ENC_DEBOUNCE_VAL;
  }
}

/*
 * ISR (Interrupt Service Routine) triggered by start/stop button
 */
void doButtons() {
  // pressed start: toggle
  if (btn_debounce == 0) {
    state = !state;
    stateChanged = true;
    btn_debounce = BTN_DEBOUNCE_VAL;
  }
}

/*
 * Update lcd screen based on global variables
 */
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

/*
 * Change frequency based on rotary encoder inputs
 * returns if frequency has changed
 */
bool checkDigits() {
  int prevFreq = frequency;
  // print out rotary values
  Serial.print("val, last: ");
  Serial.print(valRotary);
  Serial.print(" ");
  Serial.println(lastValRotary);

  int pow10[] = { 1000, 100, 10, 1 };
  
  if(valRotary - lastValRotary >= ENC_DELTA_VAL) {
    // decrement with rollover
    frequency = frequency - pow10[freqDigit];
    if ( frequency <= 0 ) frequency = 1;
  }
  if(lastValRotary - valRotary >= ENC_DELTA_VAL) {
    // incremement
    frequency = frequency + pow10[freqDigit];
    if ( frequency > 1200 ) frequency = 1200;
  }
  
  int freqToArray = frequency;
  for (int i = 3; i >= 0; --i) {
    freqDigits[i] = freqToArray % 10;
    freqToArray /= 10;
  }

  lastValRotary = valRotary;
  return (prevFreq != frequency); // return true if frequency has changed
}

// the setup function runs once when you press reset or power the board
void setup() {
  // mtr_in2 remains low
  pinMode(mtr_in2, OUTPUT);

  // init serial output
  Serial.begin(9600);

  // init lcd screen
  lcd.init();
  lcd.backlight();
  lcd.setCursor(1,0);
  lcd.print("Fibos");

  //set the initial valve start
  digitalWrite(mtr_in2, LOW);

  // init rotary encoder
  pinMode(encoder0PinA, INPUT_PULLUP);
  pinMode(encoder0PinB, INPUT_PULLUP);
  pinMode(encoder0Btn, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoder0PinA), doEncoder, CHANGE);

  // init button
  pinMode(btnIntPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(btnIntPin), doButtons, FALLING);

  // init tones. tone2 needs to be init although unused because the library
  // does not allow for selection of timer, and assigns them in order (timer2, timer1, timer0)
  // on UNO timer1 is 16 bit which allows for 1Hz output waves unlike the other 8 bit timers
  tone2.begin(PIN_UNUSED);
  tone1.begin(mtr_in1);

  freqDigit = 0;
  delay(1000);
  printLCD();
}

// the loop function runs over and over again forever
void loop() {
  // read rotary encoder switch
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
  if (sw_debounce > 0){
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
  
  // only change tone if freq changed from what is currently playing and play state
  if (toneFreq != frequency && state) {
    toneFreq = frequency;
    tone1.play(toneFreq);
  }
  if (!state) {
    tone1.stop();
    toneFreq = 0;
  }
}
