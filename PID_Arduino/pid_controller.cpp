/*
 ******************************************************************
 * Arduino PID Library
 * Author: Paulo Bonfim @git: https://github.com/PauloJr05
 * Version: 1.0.0	
 * Date: 12/jul/2019
 ******************************************************************
 */

#include "Arduino.h"
#include "pid_controller.h"

#define MAX 255
#define MIN 0
// coeficientes do PID
    double bn[3];
    double yn[2];
    double xn[3];
   
// variaveis do PID
    double kp_aux;
    double ki_aux;
    double kd_aux;
    double sat_max_aux;
    double sat_min_aux;
	
    int direction_aux;


PID::PID(double kp, double ki, double kd, double sat_min, double sat_max, int direction){
	kd_aux = kd;
	ki_aux = ki;
	kp_aux = kp;
	sat_max_aux = sat_max;
	sat_min_aux = sat_min;
	direction_aux = direction;

	PID::initialize();
    

}

PID::PID(double kp, double ki, double kd, int direction)
	:PID::PID(kp, ki, kd, MIN, MAX, direction)

{

}

void PID::initialize(){
//Coef do controlador
	xn[0] = 0.0;
    xn[1] = 0.0;
    xn[2] = 0.0;
//Saida do controlador
    yn[0] = 0.0;
    yn[1] = 0.0;
	bn[0] = kp_aux + ki_aux + kd_aux;
	bn[1] = kp_aux + (2.0 * ki_aux);
	bn[2] = kd_aux;

}

double PID::compute(double setpoint, double measured){

    xn[0] = setpoint - measured;

	switch (direction_aux)
	{
	case DIRECT:
		yn[0] = -(bn[0] * xn[0]) -
		(bn[1] * xn[1]) -
		(bn[2] * xn[2]) +
		(-yn[1]);
		break;
	case REVERSE:
		/* code */
		yn[0] = (bn[0] * xn[0]) +
		(bn[1] * xn[1]) +
		(bn[2] * xn[2]) +
		(-yn[1]);
		break;
	default:
		return -1;
		break;
	}
	
	if (yn[0] > sat_max_aux) yn[0] = sat_max_aux;
	else if (yn[0] < sat_min_aux) yn[0] = sat_min_aux;

	xn[2] = xn[1];
	xn[1] = xn[0];
	yn[1] = yn[0];

	return (yn[0]);

}

void PID::tune(double kp, double ki, double kd){

    bn[0] = kp + ki + kd;
	bn[1] = kp + (2.0 * ki);
	bn[2] = kd;

}

void PID::reset(){

    xn[0] = 0.0;
	xn[1] = 0.0;
	xn[2] = 0.0;
	yn[0] = 0.0;
	yn[1] = 0.0;
}

double PID::get_Kp(){

	return kp_aux;
}

double PID::get_Ki(){

	return ki_aux;
}

double PID::get_Kd(){

	return kd_aux;
}

int PID::get_direction(){

	return direction_aux;
}


