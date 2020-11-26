#include <LiquidCrystal_I2C.h>
#include <Wire.h>

/*
 * Challenges:
 * Probably only want start and stop buttons to be interrupts (not sure if rotary encoder supports)
 * For lower frequencies need a way to poll without busy waiting 
 * First lets try a non-interrupting loop
 */

//Logic inputs for the DRV8871 unit
//By default the valve is off when there is no voltage. Keep both pins to 0 (low)
//To turn on, set in1 to high, in2 to low (current flows OUT1 to OUT2)
const int mtr_in1 = 11;
const int mtr_in2 = 10;

LiquidCrystal_I2C lcd(0x27,20,4);

uint32_t prev_micros;
int frequency;             // in Hz
int freq_delta = 50;        // in Hz

// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(mtr_in1, OUTPUT);
  pinMode(mtr_in2, OUTPUT);

  Serial.begin(9600);

  lcd.init();

  //set the initial valve start
  digitalWrite(mtr_in1, LOW);
  digitalWrite(mtr_in2, LOW);
  
  lcd.backlight();
  lcd.setCursor(1,0);
  lcd.print("Fibos");
  
  frequency = 300;
  delay(1000);
}

// the loop function runs over and over again forever
void loop() {

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(frequency);
  lcd.print(" Hz");

  tone(mtr_in1, frequency);
  // cycle frequency
  if (frequency <= 50 || frequency >= 1200) {
    freq_delta *= -1;
  }
  frequency += freq_delta;
  delay(1000);
}
