#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <Keypad.h>
#include <Adafruit_PWMServoDriver.h>

#define SERVOMIN  180 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  585 // this is the 'maximum' pulse length count (out of 4096)
#define SERVOINC  45  // the increment to cover the range from MIN to MAX in 9 steps

LiquidCrystal_I2C lcd(0x27,2,1,0,4,5,6,7,3,POSITIVE);  // Set the LCD I2C address and parameters
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

const byte ROWS = 4; //four rows
const byte COLS = 4; //four columns
char keys[ROWS][COLS] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};
byte rowPins[ROWS] = {39,41,43,45}; //connect to the row pinouts of the keypad
byte colPins[COLS] = {47,49,51,53}; //connect to the column pinouts of the keypad

Keypad keypad = Keypad(makeKeymap(keys),rowPins,colPins,ROWS,COLS);

void setup() {
  
  lcd.begin(16,2);  // initialize the lcd 
  lcd.home();       // go to the top line
  lcd.print("SMU Lyle KNW2300");  
  lcd.setCursor(0,1);        // go to the bottom line
  lcd.print("Servo: ");
  pwm.begin();
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
  delay(10);
}

void loop() {
  char key;
  int servo;
  
  key = keypad.getKey();
  if (key) {
    if (key >= '0' && key <= '9') {
      servo = SERVOMIN + (key - '0') * SERVOINC;
      lcd.setCursor(7,1);  // go to the bottom line
      lcd.print(servo);
      pwm.setPWM(8,0,servo);
    }
  }
}
