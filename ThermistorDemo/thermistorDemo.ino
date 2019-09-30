#include <KNWRobot.h>
KNWRobot* bot;
const int TEMP_PIN = A0;            //Can be any analog pin
void setup() {
  bot = new KNWRobot();
  bot->setupTemp(TEMP_PIN);
  bot->clearLCD();
  char str[] = "ADC Code: ";
  bot->printLCD(str);
}

void loop() {
  bot->moveCursor(10,0);            //Set cursor to end of str
  int code = bot->getTemp();        //Read ADC code from sensor
  bot->printLCD(code);              //Print ADC code on LCD display
  delay(1000);                      //wait 1s before refreshing value
}
