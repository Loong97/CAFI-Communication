#include "Signode.h"

Signode::Signode(uint8_t c,uint8_t s):radio(c,s){}

void Signode::SwitchToListenMode(){
    radio.openReadingPipe(1,PIPE_IDS[pipe]);
    radio.begin();
    radio.startListening();
}

void Signode::SwitchToWriteMode(int target){
    radio.stopListening();
    radio.openWritingPipe(PIPE_IDS[target]);
}

void Signode::SendOrder(int target,int source,int type,int value){
    SwitchToWriteMode(target);
    int data = source * TYPE_MAX + type * VALUE_MAX + value;
    int tries = 100; 
    while(tries>0){
        if(radio.write(&data,sizeof(int))) break;
        tries -= 1;
    }
    SwitchToListenMode();
}

void Signode::DecodeOrder(int data,int &source,int &type,int &value){
    source = data / TYPE_MAX;
    type = data % TYPE_MAX / VALUE_MAX;
    value = data % VALUE_MAX;
}