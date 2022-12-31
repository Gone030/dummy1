#include <Arduino.h>

class PID
{
    private:
        int min_value_;
        int max_value_;
        double kp_;
        double ki_;
        double kd_;
        double integral_;
        double derivative_;
        double prev_error_;
        unsigned long prev_time_;

    public:
        PID(int min_value, int max_value, double kp, double ki, double kd);
        double pidcompute(float setpoint, float mesured_value);
        void updateparam(double kp, double ki, double kd);
};
