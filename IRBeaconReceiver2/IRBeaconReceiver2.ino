/* 
This program interprets a series of IR on-off events as a series of eight binary bits
as encoded by the NEC remote control protocol. The ASCII character represented
by the bit seqence is displayed on the LCD screen. The receiver is assumed to give an
inverted output, so the initial event to be captured is a falling edge.
This code version uses software recognition of IR receiver states, and makes use
of the micros() Arduino library routine for capturing event timing.
The IR receiver output should be wired to digital pin 2, which is PORTD bit 2.
*/

#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#define receiverPin 2
#define EVENT_RISING 1
#define EVENT_FALLING 2
#define EVENT_NONE 0

LiquidCrystal_I2C lcd(0x3F,2,1,0,4,5,6,7,3,POSITIVE);  // Set the LCD I2C address and parameters
unsigned char IRChar,IRCharBitMask,necState,buffer[8],row[16];
int col;
int buffer_in,buffer_out;
boolean receiverState = false;
unsigned long prev_time,cur_time,ticks;

void setup() {
  int i;
  i = 2; // set D2 through D13 to be digital outputs, except for receiver input
  while (i <= 13) {
    if (i != receiverPin) pinMode(i,OUTPUT);
    else pinMode(i,INPUT);
    i++;
  }
  lcd.begin(16,2);  // initialize the lcd 
  lcd.home();       // go to the top line
  lcd.print("KNW2300 Beacons");  
  lcd.setCursor(0,1);        // go to the bottom line
  lcd.blink();      // enable blinking cursor
  col = 0;
  necState = 0;
  buffer_in = 0;
  buffer_out = 0;
  prev_time = 0;
  delay(1000);
}

void loop() {
  unsigned char event;

  if (digitalRead(receiverPin)) {  // Digital level from IR receiver will be inverted
    if (receiverState) event = EVENT_FALLING;
    else event = EVENT_NONE;
    receiverState = false;
  }
  else {
    if (!receiverState) event = EVENT_RISING;
    else event = EVENT_NONE;
    receiverState = true;
  }
  if (event != EVENT_NONE) {
    cur_time = micros();
    ticks = cur_time - prev_time;
    if (necState == 0) {  // Expecting rising edge of leading pulse
      if (event == EVENT_RISING) {
        necState = 1;
      }
    }
    else if (necState == 1) {  // Expecting falling edge of leading pulse
      if (event == EVENT_FALLING) {
        if (ticks > 8900L) necState = 2;  // Check for leading pulse > 8.9msec
        else {  // Stray short pulse found, reset NEC state
          necState = 0;
        }
      }
    }
    else if (necState == 2) {  // Expecting rising edge of first pulse after leading pulse
      if (event == EVENT_RISING) {
        if (ticks > 3375L) {  // Check for space after leading pulse > 3.375 msec
          IRCharBitMask = 0x80;
          IRChar = 0;
          necState = 3;
        }
        else {  // Space too short, reset NEC state to wait for another leading pulse
          necState = 0;
        }
      }
    }
    else if (necState == 3) {  // Expecting falling edge of data pulse
      if (event == EVENT_FALLING) {
        if (ticks < 648) necState = 4;  // Check if data pulse width < 648 usec
        else {  // Width too short, reset NEC state to wait for another leading pulse
          necState = 0;
        }
      }
    }
    else if (necState == 4) {  // Expecting rising edge of pulse after data pulse
      if (event == EVENT_RISING) {
        if (ticks > 1120) {  // Record a '1' bit for space > 1120 usec
          IRChar = IRChar | IRCharBitMask;
        }
        IRCharBitMask = IRCharBitMask >> 1;
        if (IRCharBitMask == 0) {  // Check if eighth bit received and character complete
          buffer[buffer_in] = IRChar;  // Record complete character received in circular output buffer
          buffer_in = (buffer_in + 1) & 0x07;
          necState = 0;  // Reset NEC state to wait for another leading pulse
        }
        else necState = 3;  // Wait for falling edge of data pulse
      }
    }
    prev_time = cur_time;
  }
  if (buffer_out != buffer_in) {
    lcd.write(buffer[buffer_out]);  // Report received characters to LCD
    row[col] = buffer[buffer_out];
    col++;
    if (col == 16) {
      lcd.clear();
      lcd.home();
      lcd.write(row,16);
      lcd.setCursor(0,1);        // go to the bottom line
      col = 0;
    }
    buffer_out = (buffer_out + 1) & 0x07;
  }
}
