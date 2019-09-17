/*
Arduino code for the dual slave IR beacon in the robot navigational system, which
must be the last slave in the beacon chain.
The modulation ASCII characters are transmitted when a negative-going trigger pulse
has been received on the D2 pin.
The 38kHz square wave subcarrier is generated using timer2 in
CTC waveform generation mode. The 38kHz square wave subcarrier will be output on
digital pin 11. Digital pin 10, which is PORTB bit 2, is the LED modulation output for
character 1, and Digital pin 9 is the LED modulation output for character 2.
The driver for IR LED bank 1 should send current through the LED when both D10 AND D11
are high, and the driver for IR LED bank 2 should send current through the LED when
both D9 AND D11 are high.
Measuring elapsed time between output LED change events uses timer1 in hardware
compare mode.
  Serial commands:
    ?                Identify module and ASCII modulation characters
    C<char1><char2>  Set ASCII characters to supply modulation bit sequence to LED
    H                Set modulation outputs high
    L                Set modulation outputs low
*/

#include <EEPROM.h>
#define ledPin 4  // Red monitor LED pin
#define carrierPin 11  // OC2A pin for 38kHz square wave carrier output - cannot be changed
#define modulation1Pin 10  // LED bank 1 modulation output pin
#define modulation2Pin 9   // LED bank 2 modulation output pin
#define triggerInPin 2  // Trigger input pin from master or previous slave

unsigned char incomingChar,commandChar,serialState,modulationState;
unsigned char modulation1Char,modulation2Char;
volatile boolean timer1Overflow = false;
volatile unsigned char modulationMask = 0x80;

void setup() {
  int i;
  i = 2; // set D2 through D13 to be digital outputs, except for trigger input pin
  while (i <= 13) {
    if (i == triggerInPin) pinMode(i,INPUT);
    else pinMode(i,OUTPUT);
    i++;
  }

  // initialize timer1 
  noInterrupts();           // disable all interrupts
// Set timer 1 prescaler factor = 64 (ticks every 4usec) and 'Clear Timer on Compare Match' mode
  TCCR1A = 0;
  TCCR1B = ((1 << CS11) | (1 << CS10) | (1 << WGM12));
  TCNT1 = 0;   // reset timer1 tick counter
  TIMSK1 |= (1 << TOIE1) | (1 << OCIE1A); // enable timer overflow and compare A interrupts

// Set timer 2 prescale factor = 1, 'Toggle OC2A' and 'Clear Timer on Compare Match' modes
  TCCR2A = ((1 << COM2A0) | (1 << WGM21));
  TCCR2B = (1 << CS20);
// Set OCR2A to (16MHz / (output frequency * 2 * prescale factor)) - 1
// or output frequency = 16MHz / ( 2 * prescale factor * (OCR2A+1))
  OCR2A = 209;  // set output frequency = 38.1kHz

  OCR1A = 62500;
  digitalWrite(modulation1Pin,LOW); // set modulation low
  digitalWrite(modulation2Pin,LOW);
  digitalWrite(ledPin,LOW); // reset LED
  interrupts();             // enable all interrupts
  Serial.begin(9600);      // setup serial
  modulation1Char = EEPROM.read(0);
  if (modulation1Char != 'G' && modulation1Char != 'K' && modulation1Char != 'M' && modulation1Char != 'N'
      && modulation1Char != 'S' && modulation1Char != 'U' && modulation1Char != 'V' && modulation1Char != 'Y'
      && modulation1Char != 'Z') {
    modulation1Char = 'G';
    EEPROM.write(0,modulation1Char);
  }
  modulation2Char = EEPROM.read(1);
  if (modulation2Char != 'G' && modulation2Char != 'K' && modulation2Char != 'M' && modulation2Char != 'N'
      && modulation2Char != 'S' && modulation2Char != 'U' && modulation2Char != 'V' && modulation2Char != 'Y'
      && modulation2Char != 'Z') {
    modulation2Char = 'G';
    EEPROM.write(1,modulation2Char);
  }
  modulationState = 0;  // Wait indefinitely for a low signal on the trigger input
  serialState = 0;
}

ISR(TIMER1_COMPA_vect){
  if (modulationState == 1) {
    digitalWrite(modulation1Pin,LOW); // set modulation low for leading space of character 1
    modulationState = 2;  // Transmit 4.5 msec leading space
    OCR1A = 1125;
  }
  else if (modulationState == 2) {
    digitalWrite(modulation1Pin,HIGH); // set modulation high for first data bit of character 1
    modulationState = 3;  // Transmit 560 usec data pulse
    OCR1A = 140;
    modulationMask = 0x80;
  }
  else if (modulationState == 3) {
    digitalWrite(modulation1Pin,LOW); // set modulation low for data space of character 1
    modulationState = 4;  // Transmit either 560 or 1676 usec data space
    if ((modulation1Char & modulationMask) != 0) OCR1A = 419;
    else OCR1A = 140;
  }
  else if (modulationState == 4) {
    digitalWrite(modulation1Pin,HIGH); // set modulation high for next data bit of character 1
    modulationMask = modulationMask >> 1;
    if (modulationMask) modulationState = 3;  // Transmit 560 usec data pulse
    else modulationState = 5;  // Transmit 560 usec final pulse
    OCR1A = 140;
  }
  else if (modulationState == 5) {
    digitalWrite(modulation1Pin,LOW); // set modulation low for final space of character 1
    modulationState = 10;  // Wait 10 msec delay for transmitting second character
    OCR1A = 2500;
  }
  else if (modulationState == 10) {
    digitalWrite(modulation2Pin,HIGH); // set modulation high for leading pulse of character 2
    modulationState = 11;  // Transmit 9 msec leading pulse
    OCR1A = 2250;
  }
  else if (modulationState == 11) {
    digitalWrite(modulation2Pin,LOW); // set modulation low for leading space of character 2
    modulationState = 12;  // Transmit 4.5 msec leading space
    OCR1A = 1125;
  }
  else if (modulationState == 12) {
    digitalWrite(modulation2Pin,HIGH); // set modulation high for first data bit of character 2
    modulationState = 13;  // Transmit 560 usec data pulse
    OCR1A = 140;
    modulationMask = 0x80;
  }
  else if (modulationState == 13) {
    digitalWrite(modulation2Pin,LOW); // set modulation low for data space of character 2
    modulationState = 14;  // Transmit either 560 or 1676 usec data space
    if ((modulation2Char & modulationMask) != 0) OCR1A = 419;
    else OCR1A = 140;
  }
  else if (modulationState == 14) {
    digitalWrite(modulation2Pin,HIGH); // set modulation high for next data bit of character 2
    modulationMask = modulationMask >> 1;
    if (modulationMask) modulationState = 13;  // Transmit 560 usec data pulse
    else modulationState = 15;  // Transmit 560 usec final pulse
    OCR1A = 140;
  }
  else if (modulationState == 15) {
    digitalWrite(modulation2Pin,LOW); // set modulation low for final space of character 2
    digitalWrite(ledPin,LOW); // reset LED
    modulationState = 0;  // Wait for next trigger
    OCR1A = 62500;
  }
}

ISR(TIMER1_OVF_vect) {      // interrupt service routine, called once every 262.144 milliseconds
  timer1Overflow = true;   // increment high order timer1 count word
}

void loop() {
  unsigned short count;

  if (Serial.available() > 0) {
    incomingChar = Serial.read();
    if (serialState == 0) {
      commandChar = incomingChar;
      if (commandChar == '?') {
        Serial.print("Infrared Beacon Slave: Characters ");
        Serial.write(modulation1Char);
        Serial.write(modulation2Char);
        Serial.println("");
      }
      else if (commandChar == 'H') {
        modulationState = 100;
        digitalWrite(modulation1Pin,HIGH); // set modulation high
        digitalWrite(modulation2Pin,HIGH); // set modulation high
        digitalWrite(ledPin,HIGH); // set LED
      }
      else if (commandChar == 'L') {
        modulationState = 100;
        digitalWrite(modulation1Pin,LOW); // set modulation low
        digitalWrite(modulation2Pin,LOW); // set modulation low
        digitalWrite(ledPin,LOW); // reset LED
      }
      else if (commandChar == 'C') {
        modulationState = 100;
        digitalWrite(modulation1Pin,LOW); // set modulation low
        digitalWrite(modulation2Pin,LOW); // set modulation low
        digitalWrite(ledPin,LOW); // reset LED
        serialState = 1;
      }
      else if (!isspace(commandChar)) {
        Serial.println("Unrecognized command");
      }
    }
    else if (serialState == 1) {
      modulation1Char = incomingChar;
      EEPROM.write(0,modulation1Char);
      serialState = 2;
    }
    else if (serialState == 2) {
      modulation2Char = incomingChar;
      EEPROM.write(1,modulation2Char);
      serialState = 0;
      modulationState = 0;
    }
  }
  if (modulationState == 0) {
    if (digitalRead(triggerInPin) == LOW) {
      digitalWrite(modulation1Pin,HIGH); // set modulation high for leading pulse of character 1
      digitalWrite(ledPin,HIGH); // set LED
      TCNT1 = 0;
      modulationState = 1;  // Transmit 9 msec leading pulse
      OCR1A = 2250;
    }
  }
}
