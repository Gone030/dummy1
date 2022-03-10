#include <Arduino.h>

class Serialcomm
{
    private:
        float linear_vel_;
        float Anguler_vel_;


    public:
        Serialcomm();
        ~Serialcomm();
        float readstr_tofloat();
};

float Serialcomm::readstr_tofloat()
{
    String temp = "";
    if (Serial.available())
    {
        temp = Serial.readString();
    }
    int first = temp.indexOf("&");
    int check = temp.indexOf("!");
    int end = temp.indexOf("\r\n");
    if (check < 0)
    {
        return 0;
    }
    else if (check > 0)
    {
        
    }
}