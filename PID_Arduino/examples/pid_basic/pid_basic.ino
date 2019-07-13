#include "pid_controller.h"

double  kp,
        ki,
        kd,
        setpoint,
        sat_max,
        measured,
        sat_min;


PID mypid(kp,ki,kd, REVERSE);
double saida = 0;
void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:
  saida = mypid.compute(setpoint, measured);// calcula a saida com base no sinal de entrada do sensor (measured)
  //mypid.tune (kp,ki,kd); // atualiza os ganhos

}
