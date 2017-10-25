#ifndef _Signode_
#define _Signode_
#include "Arduino.h"
#include "SPI.h"
#include "nRF24L01.h"
#include "RF24.h"

const uint64_t PIPE_IDS[4] = {
	0xE8E8F0F0E0LL,
	0xE8E8F0F0E1LL,
	0xE8E8F0F0E2LL,
	0xE8E8F0F0E3LL,
};
const int VALUE_MAX = 1000;
const int TYPE_MAX = 10000;
const int PIV_PPID = 0;
const int CAR_ORDER_REQID = 0;
const int PIV_ORDER_GRTID = 0;
const int PIV_ORDER_BRAKE = 1;
const int PIV_ORDER_FORWD = 2;
const int PIV_ORDER_BACK = 3;
const int PIV_ORDER_ROTAT = 4;
const int REQID_VALUE = 100;

class Signode {
protected:
    Signode(uint8_t c,uint8_t s);
    void SwitchToListenMode();
	void SwitchToWriteMode(int target);
	void SendOrder(int target,int source,int type,int value);
    void DecodeOrder(int data,int &source,int &type,int &value);
public:
    RF24 radio;
protected:
    int pipe;
};

#endif