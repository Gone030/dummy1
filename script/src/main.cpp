#include <Arduino.h>
#include <String.h>

int cmd_anguler_vel = 0;
int cmd_linear_vel = 0;

void setup()
{
  Serial.begin(115200);
}

void loop()
{
  String temp = "";
  if (Serial.available())
  {
    temp = Serial.readStringUntil('\r');
  }
  int first = temp.indexOf("&");
  int lenth = temp.length();

  int check = temp.indexOf("!");
  if (first != -1)
  {
    if (check > 0)
    {
      String angular_vel = temp.substring(first, check);
      cmd_anguler_vel = angular_vel.toInt();
      String linear_vel = temp.substring(check + 1, lenth);
      cmd_linear_vel = linear_vel.toInt();
    }
    if (temp.indexOf("Roll") > -1)
      Serial.println("@Hi");
    else if (temp.indexOf("@Pitch") > -1)
      Serial.println("@I'm");
    else if (temp.indexOf("@Yaw") > -1)
      Serial.println("@ji");
    else if (temp.indexOf("@Ax") > -1)
      Serial.println("@won");
  }
  else
  {
    temp = "";
  }
}
