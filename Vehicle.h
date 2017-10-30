#ifndef _Vehicle_
#define _Vehicle_
#include "Signode.h"

const int TRACKER_LEFT = 4;
const int TRACKER_RIGHT = 3;
const int KEY_PIN = A2;
const int BEEP_PIN = A3;

const int WHITE_WHITE = 0;
const int BLACK_WHITE = 1;
const int WHITE_BLACK = 2;
const int BLACK_BLACK = 3;

const int SPEED_MIN = 0;
const int SPEED_MAX = 255;
const int SPEED_INIT = 150;
const int CALI_AMOUNT = 1;
const int CALI_DELAY = 500;

const bool BACK = false;
const bool GO = true;
const bool LEFT = false;
const bool RIGHT = true;
const bool SOLO = false;
const bool CONTU = true;

const int CALI_CTMT = 50;
const int CALI_WTTM_STRT = 500;
const int CALI_WTTM_ROTT = 500;

class Vehicle : public Signode {
public:
	Vehicle(uint8_t c,uint8_t s,int lg,int lb,int rg,int rb);
	void Setup();
	void RequestPipeId();
	void Standby();
private:
	void Brake();
	void Start_forward(int left_speed,int right_speed);
	void Start_forward();
	void Start_back(int left_speed,int right_speed);
	void Start_back();
	void Start_left(int right_speed);
	void Start_left();
	void Start_right(int left_speed);
	void Start_right();

	void Skew(bool go,bool side,int amount);
	bool Calibrate_relative_process(bool go);
	bool Calibrate_relative(int tries);
	bool Calibrate_straight(int tries);
	bool Calibrate_rotate(int tries);

	void Go_forward(int centimeters,bool continuous);
	void Go_back(int centimeters,bool continuous);
	void Go_left(int sectors,bool continuous);
	void Go_right(int sectors,bool continuous);

	static int Tracker();
	void Alarm(int milliseconds);
private:
	int left_motor_go;
	int left_motor_back;
	int right_motor_go;
	int right_motor_back;

	int left_speed_go;
	int left_speed_back;
	int right_speed_go;
	int right_speed_back;
	
	int centim_millis_go;
	int centim_millis_back;
	int sector_millis_left;
	int sector_millis_right;
};

#endif