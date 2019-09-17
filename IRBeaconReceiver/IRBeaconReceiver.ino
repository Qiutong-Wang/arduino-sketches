/* 
This program interprets a series of IR on-off events as a series of eight binary bits
as encoded by the NEC remote control protocol. The ASCII character represented
by the bit seqence is sent to the serial port. The receiver is assumed to give an
inverted output, so the initial event to be captured is a falling edge.
This code version uses software recognition of IR receiver states, and makes use
of the micros() Arduino library routine for capturing event timing.
The IR receiver output should be wired to digital pin 2, which is PORTD bit 2.
For debugging, the Arduino LED on pin D13 is lit while a character is being
received, but this code should be removed for use in robot firmware.
*/

#define ledPin 13
#define receiverPin 2
#define EVENT_RISING 1
#define EVENT_FALLING 2
#define EVENT_NONE 0

unsigned char IRChar,IRCharBitMask,necState,buffer[8];
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
  Serial.begin(9600);      // setup serial
  digitalWrite(ledPin,LOW); // reset mode LED
  necState = 0;
  buffer_in = 0;
  buffer_out = 0;
  prev_time = 0;
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
        digitalWrite(ledPin,HIGH);
      }
    }
    else if (necState == 1) {  // Expecting falling edge of leading pulse
      if (event == EVENT_FALLING) {
        if (ticks > 8900L) necState = 2;  // Check for leading pulse > 8.9msec
        else {  // Stray short pulse found, reset NEC state
          digitalWrite(ledPin,LOW);
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
          digitalWrite(ledPin,LOW);
          necState = 0;
        }
      }
    }
    else if (necState == 3) {  // Expecting falling edge of data pulse
      if (event == EVENT_FALLING) {
        if (ticks < 648) necState = 4;  // Check if data pulse width < 648 usec
        else {  // Width too short, reset NEC state to wait for another leading pulse
          digitalWrite(ledPin,LOW);
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
          digitalWrite(ledPin,LOW);
          necState = 0;  // Reset NEC state to wait for another leading pulse
        }
        else necState = 3;  // Wait for falling edge of data pulse
      }
    }
    prev_time = cur_time;
  }
  if (buffer_out != buffer_in) {
    Serial.write(buffer[buffer_out]);  // Report received characters to serial port
    Serial.write('\r');
    Serial.write('\n');
    buffer_out = (buffer_out + 1) & 0x07;
  }
}
