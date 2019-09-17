/*
Arduino code for the slave IR beacon in the robot navigational system.
The modulation ASCII character is transmitted when a negative going trigger
input pulse is received on the D2 pin. After each transmission a 1 msec pulse
is sent to pin D9 to send a trigger to the next slave down the beacon chain.
The 38kHz square wave subcarrier is generated using timer2 in
CTC waveform generation mode. The 38kHz square wave subcarrier will be output on
digital pin 11, and digital pin 10, which is PORTB bit 2, is the LED modulation output.
The IR LED driver should send current through the LED when both D10 AND D11 are high.
Measuring elapsed time between output LED change events uses timer1 in hardware
compare mode.
  Serial commands:
    ?        Identify module and ASCII modulation character
    C<char>  Set ASCII character to supply modulation bit sequence to LED
    H        Set modulation output high
    L        Set modulation output low
*/

#include <EEPROM.h>
#define ledPin 4  // Red monitor LED pin
#define carrierPin 11  // OC2A pin for 38kHz square wave carrier output - cannot be changed
#define modulationPin 10  // LED modulation output pin
#define triggerInPin 2  // Trigger input pin from master or previous slave
#define triggerOutPin 9  // Trigger output pin to first slave

unsigned char incomingChar,commandChar,serialState,modulationState,modulationChar;
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
  digitalWrite(modulationPin,LOW); // set modulation low
  digitalWrite(ledPin,LOW); // reset LED
  digitalWrite(triggerOutPin,LOW); // reset trigger output
  interrupts();             // enable all interrupts
  Serial.begin(9600);      // setup serial
  modulationChar = EEPROM.read(0);
  if (modulationChar != 'G' && modulationChar != 'K' && modulationChar != 'M' && modulationChar != 'N'
      && modulationChar != 'S' && modulationChar != 'U' && modulationChar != 'V' && modulationChar != 'Y'
      && modulationChar != 'Z') {
    modulationChar = 'G';
    EEPROM.write(0,modulationChar);
  }
  modulationState = 0;  // Wait indefinitely for a low signal on the trigger input
  serialState = 0;
}

ISR(TIMER1_COMPA_vect){
  if (modulationState == 1) {
    digitalWrite(modulationPin,LOW); // set modulation low for leading space
    modulationState = 2;  // Transmit 4.5 msec leading space
    OCR1A = 1125;
  }
  else if (modulationState == 2) {
    digitalWrite(modulationPin,HIGH); // set modulation high for first data bit
    modulationState = 3;  // Transmit 560 usec data pulse
    OCR1A = 140;
    modulationMask = 0x80;
  }
  else if (modulationState == 3) {
    digitalWrite(modulationPin,LOW); // set modulation low for data space
    modulationState = 4;  // Transmit either 560 or 1676 usec data space
    if ((modulationChar & modulationMask) != 0) OCR1A = 419;
    else OCR1A = 140;
  }
  else if (modulationState == 4) {
    digitalWrite(modulationPin,HIGH); // set modulation high for next data bit
    modulationMask = modulationMask >> 1;
    if (modulationMask) modulationState = 3;  // Transmit 560 usec data pulse
    else modulationState = 5;  // Transmit 560 usec final pulse
    OCR1A = 140;
  }
  else if (modulationState == 5) {
    digitalWrite(modulationPin,LOW); // set modulation low for final space
    modulationState = 6;  // Wait 10 msec delay for trigger
    OCR1A = 2500;
  }
  else if (modulationState == 6) {
    digitalWrite(triggerOutPin,HIGH); // set trigger output
    digitalWrite(ledPin,LOW); // reset LED
    modulationState = 7;  // Wait 10 msec trigger pulse width
    OCR1A = 2500;
  }
  else if (modulationState == 7) {
    digitalWrite(triggerOutPin,LOW); // reset trigger output
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
        Serial.print("Infrared Beacon Slave: Character ");
        Serial.write(modulationChar);
        Serial.println("");
      }
      else if (commandChar == 'H') {
        modulationState = 10;
        digitalWrite(modulationPin,HIGH); // set modulation high
        digitalWrite(ledPin,HIGH); // set LED
      }
      else if (commandChar == 'L') {
        modulationState = 10;
        digitalWrite(modulationPin,LOW); // set modulation low
        digitalWrite(ledPin,LOW); // reset LED
      }
      else if (commandChar == 'C') {
        modulationState = 10;
        digitalWrite(modulationPin,LOW); // set modulation low
        digitalWrite(ledPin,LOW); // reset LED
        serialState = 1;
      }
      else if (!isspace(commandChar)) {
        Serial.println("Unrecognized command");
      }
    }
    else if (serialState == 1) {
      modulationChar = incomingChar;
      EEPROM.write(0,modulationChar);
      serialState = 0;
      modulationState = 0;
    }
  }
  if (modulationState == 0) {
    if (digitalRead(triggerInPin) == LOW) {
      digitalWrite(modulationPin,HIGH); // set modulation high for leading pulse
      digitalWrite(ledPin,HIGH); // set LED
      TCNT1 = 0;
      modulationState = 1;  // Transmit 9 msec leading pulse
      OCR1A = 2250;
    }
  }
}
