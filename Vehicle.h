#ifndef _Vehicle_
#define _Vehicle_
#include "Signode.h"

class Vehicle : public Signode {
public:
	Vehicle(uint8_t c,uint8_t s,int lg,int lb,int rg,int rb);
	void Setup();
	void RequestPipeId();
	void Standby();
private:
	void Brake();
	void Go_forward(float distance);
	void Go_back(float distance);
	void Go_left(float degree);
	void Go_right(float degree);
	void Alarm(int milliseconds);
private:
	int left_motor_go;
	int left_motor_back;
	int right_motor_go;
	int right_motor_back;
private:
	const int key_pin = A2;
	const int beep_pin = A3;
	const int sensor_right = 3;
	const int sensor_left = 4;
};

#endif