#include "Vehicle.h"

Vehicle::Vehicle(uint8_t c,uint8_t s,int lg,int lb,int rg,int rb):Signode(c,s){
    left_motor_go = lg;
    left_motor_back = lb;
    right_motor_go = rg;
    right_motor_back = rb;
    pipe = PIV_PPID;
}

void Vehicle::Setup(){
    pinMode(left_motor_go,OUTPUT);
    pinMode(left_motor_back,OUTPUT);
    pinMode(right_motor_go,OUTPUT);
    pinMode(right_motor_back,OUTPUT);
    Brake();
    pinMode(key_pin,INPUT);
    pinMode(beep_pin,OUTPUT);
    pinMode(sensor_right,INPUT);
    pinMode(sensor_left,INPUT);
    Serial.begin(57600);
    radio.begin();
    SwitchToListenMode();
}

void Vehicle::RequestPipeId(){
    SendOrder(PIV_PPID,pipe,CAR_ORDER_REQID,REQID_VALUE);
    bool success = false;
    int receiveData,source,type,value;
    while(!success){
        if(!radio.available()) continue;
        radio.read(&receiveData,sizeof(int));
        DecodeOrder(receiveData,source,type,value);
        if(source!=PIV_PPID) continue;
        if(type!=PIV_ORDER_GRTID) continue;
        pipe = value;
        SwitchToListenMode();
        success = true;
    }
    Alarm(500);
}

void Vehicle::Standby(){
    int receiveData,source,type,value;
    if(!radio.available()) return;
    radio.read(&receiveData,sizeof(int));
    DecodeOrder(receiveData,source,type,value);
    if(source!=PIV_PPID) return;
    if(type==PIV_ORDER_BRAKE) Brake();
    else if(type==PIV_ORDER_FORWD) Go_forward(value);
    else if(type==PIV_ORDER_BACK) Go_back(value);
    else if(type==PIV_ORDER_ROTAT){
        value %= 360;
        if(value<180) Go_left(value);
        else Go_right(360-value);
    }
}

void Vehicle::Brake(){
    digitalWrite(left_motor_go,LOW);
    digitalWrite(left_motor_back,LOW);
    digitalWrite(right_motor_go,LOW);
    digitalWrite(right_motor_back,LOW);
}

void Vehicle::Go_forward(float distance){
    analogWrite(left_motor_go,100);
    analogWrite(left_motor_back,0);
    analogWrite(right_motor_go,100);
    analogWrite(right_motor_back,0);
}

void Vehicle::Go_back(float distance){
    analogWrite(left_motor_go,0);
    analogWrite(left_motor_back,100);
    analogWrite(right_motor_go,0);
    analogWrite(right_motor_back,100);
}

void Vehicle::Go_left(float degree){
    analogWrite(left_motor_go,0);
    analogWrite(left_motor_back,0);
    analogWrite(right_motor_go,100);
    analogWrite(right_motor_back,0);
}

void Vehicle::Go_right(float degree){
    analogWrite(left_motor_go,100);
    analogWrite(left_motor_back,0);
    analogWrite(right_motor_go,0);
    analogWrite(right_motor_back,0);
}

void Vehicle::Alarm(int milliseconds){
    digitalWrite(beep_pin,HIGH);
    delay(milliseconds);
    digitalWrite(beep_pin,LOW);
}