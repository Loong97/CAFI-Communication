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
const int MAX_TRIES = 100;
const int PIV_PPID = 0;

const int CAR_ORDER_REQID = 0;
const int CAR_FEDBK_BRAKE = 1;
const int CAR_FEDBK_FORWD = 2;
const int CAR_FEDBK_BACK = 3;
const int CAR_FEDBK_ROTAT = 4;
const int CAR_FEDBK_CLBRT = 5;

const int PIV_ORDER_GRTID = 0;
const int PIV_ORDER_BRAKE = 1;
const int PIV_ORDER_FORWD = 2;
const int PIV_ORDER_BACK = 3;
const int PIV_ORDER_ROTAT = 4;
const int PIV_ORDER_CLBRT = 5;

const int REQID_VALUE = 100;
const int CLBRT_RLTIV_VALUE = 100;
const int CLBRT_STRAT_VALUE = 200;
const int CLBRT_ROTAT_VALUE = 300;

const int CRCL_SECT = 2;

class Signode {
protected:
    Signode(uint8_t c,uint8_t s);
    void SwitchToListenMode();
	void SwitchToWriteMode(int target);
	void SendOrder(int target,int source,int type,int value);
    void DecodeOrder(int data,int &source,int &type,int &value);
protected:
    template <typename T>
    bool WaitFor(T (*function)(),T target,int milliseconds,int &wait_micros);
    template <typename T>
	bool WaitFor(T (*function)(),T target,int milliseconds);
public:
    RF24 radio;
protected:
    int pipe;
};

template <typename T>
bool Signode::WaitFor(T (*function)(),T target,int milliseconds,int &waittime){
    unsigned long start_time = millis();
    while(millis()-start_time<=milliseconds || milliseconds<=0){
        if(function()==target){
            waittime = millis()-start_time;
            return true;
        }
    }
    waittime = millis()-start_time;
    return false;
}

template <typename T>
bool Signode::WaitFor(T (*function)(),T target,int milliseconds){
    int waittime = 0;
    return WaitFor(function,target,milliseconds,waittime);
}

#endif