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
const int CRCL_SECT = 60;
const int POSTN_GROUP = 2;

const int SPEED_INIT = 100;
const unsigned long CMMSG_INIT = 40;
const unsigned long STMSG_INIT = 45;
const int POSTN_INIT = -1;

const bool LEFT = false;
const bool RIGHT = true;
const bool SLOW = false;
const bool FAST = true;
const bool SOLO = false;
const bool CONTU = true;

const int CALI_CTMT = 49;
const unsigned long CALI_WTTM = 500;
const unsigned long BRAKE_TIME = 60;

class Vehicle : public Signode {
public:
	Vehicle(uint8_t c,uint8_t s,int lg,int lb,int rg,int rb);
	void Setup();
	void RequestPipeId();
	void Standby();
private:
	void Alarm(unsigned long milliseconds);
	static int Tracker();

	void Brake();
	void Start_forward(int left_speed,int right_speed);
	void Start_forward(bool accelerate);
	void Start_forward();
	void Start_left(int right_speed);
	void Start_left();
	void Start_right(int left_speed);
	void Start_right();

	void Tracking_thin();
	void Tracking_crude();

	bool Calibrate_rotation();
	void Calibrate_relative();
	void Calibrate_position();
	void Calibrate_straight();

	void Go_forward(int centimeters,bool continuous);
	void Go_left(int degrees,bool continuous);
	void Go_right(int degrees,bool continuous);
private:
	int left_motor_go;
	int left_motor_back;
	int right_motor_go;
	int right_motor_back;

	int left_speed_go;
	int right_speed_go;
	
	unsigned long centim_millis_go;
	unsigned long sector_millis_left;
	unsigned long sector_millis_right;

	int position;
};

#endif