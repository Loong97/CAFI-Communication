#include "Pivot.h"

Pivot::Pivot(uint8_t c,uint8_t s):Signode(c,s){
    pipe = PIV_PPID;
    car_count = PIV_PPID;
}

void Pivot::Setup(){
    Serial.begin(57600);
    radio.begin();
    SwitchToListenMode();
}

void Pivot::Standby(){
    String serialData;
    int receiveData,target,source,type,value;
    if(radio.available()){
        radio.read(&receiveData,sizeof(int));
        DecodeOrder(receiveData,source,type,value);
        if(source<0||source>car_count) return;
        if(type!=CAR_ORDER_REQID) return;
        if(value==REQID_VALUE) GrantPipeId(source);
    }
    if(Serial.available()>0){
        serialData = Serial.readString();
        int index_1 = serialData.indexOf('o');
        int index_2 = serialData.indexOf('o',index_1+1);
        int target = serialData.substring(0,index_1).toInt();
        int type = serialData.substring(index_1+1,index_2).toInt();
        int value = serialData.substring(index_2+1).toInt();
        if(target<0||target>car_count) return;
        if(type<0||type>9) return;
        if(value<=0||value>=VALUE_MAX) return;
        else SendOrder(target,PIV_PPID,type,value);
        Serial.print("Send order ");
        Serial.print(type);
        Serial.print(" of value ");
        Serial.print(value);
        Serial.print(" to car No.");
        Serial.println(target);
    }
}

void Pivot::GrantPipeId(int target){
    car_count += 1;
    SendOrder(target,PIV_PPID,PIV_ORDER_GRTID,car_count);
    Serial.print("Granted ID for car No.");
    Serial.println(car_count);
}