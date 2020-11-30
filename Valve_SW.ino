 #include <LiquidCrystal_I2C.h>
#include <Wire.h>

/*
 * Challenges:
 * Probably only want start and stop buttons to be interrupts (not sure if rotary encoder supports)
 * For lower frequencies need a way to poll without busy waiting 
 * First lets try a non-interrupting loop
 */
#define encoder0PinA 3
#define encoder0PinB 4
#define encoder0Btn 5
#define mtr_in1 11
#define mtr_in2 10

//Logic inputs for the DRV8871 unit
//By default the valve is off when there is no voltage. Keep both pins to 0 (low)
//To turn on, set in1 to high, in2 to low (current flows OUT1 to OUT2)

LiquidCrystal_I2C lcd(0x27,20,4);

uint32_t prev_micros;
int frequency;             // in Hz
int freqDelta = 50;        // in Hz
volatile int encoder0Pos = 0;
volatile int valRotary;
int lastValRotary;
int freqState;

enum rigState {
  paused,
  active
};

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

void printLCD(int frequency, int freqState) {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(frequency);
  lcd.print(" Hz");
  lcd.setCursor(0,1);
  lcd.print("digit: ");
  lcd.print(freqState);
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
  
  frequency = 300;
  freqState = 0;
  delay(1000);
}

// the loop function runs over and over again forever
void loop() {
  int btn = digitalRead(encoder0Btn);
  if (!btn) {
    freqState = (++freqState >= 4)? 0 : freqState;  // rollover to starting digit
  }
  printLCD(frequency, freqState);

  if(valRotary > lastValRotary) {
    Serial.print("  CW");
  }
  if(valRotary < lastValRotary) {
    Serial.print("  CCW");
  }
  lastValRotary = valRotary;
  
  delay(1000);
}
