#ifndef pid_controller_h
#define pid_controller_h


#include "Arduino.h"



class PID
{
private:
    /* data */
    void initialize();
    
public:

    #define DIRECT  0
    #define REVERSE  1

    PID(double kp, double ki, double kd, double sat_min, double sat_max, int direction);
    PID(double kp, double ki, double kd, int direction);
    double compute (double setpoint, double measured);
    double get_Kp();
    double get_Ki();
    double get_Kd();
    void tune (double kp, double ki, double kd);
    void reset ();
    int get_direction();

    
};







#endif
