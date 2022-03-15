#include <Arduino.h>

int cmd_anguler_vel = 0;
int cmd_linear_vel = 0;
String inputstring = "";
boolean commcomplete = false;
void setup()
{
  Serial.begin(115200);
}

void loop()
{
  // if(commcomplete)
  // {
    // Serial.println("M" + inputstring);
    // inputstring = "";
    // Serial.println("Mdasdasd");
    // commcomplete = false;
  // }
  // if(Serial.available())
  // 
  //   char input = (char)Serial.read();
  //   if (input == 'c')
  //   {
      Serial.println("Megseafadf");

  // while(Serial.available())
  // {
  //   inputstring = Serial.readString();
  //   Serial.println("M" + inputstring);
  // }
}



