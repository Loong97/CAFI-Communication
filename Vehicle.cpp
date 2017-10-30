#include "Vehicle.h"

Vehicle::Vehicle(uint8_t c,uint8_t s,int lg,int lb,int rg,int rb):Signode(c,s){
    left_motor_go = lg;
    left_motor_back = lb;
    right_motor_go = rg;
    right_motor_back = rb;

    left_speed_go = SPEED_INIT;
    left_speed_back = SPEED_INIT;
    right_speed_go = SPEED_INIT;
    right_speed_back = SPEED_INIT;

    centim_millis_go = 30;
    centim_millis_back = 30;
    sector_millis_left = 30;
    sector_millis_right = 30;

    pipe = PIV_PPID;
}

void Vehicle::Setup(){
    pinMode(left_motor_go,OUTPUT);
    pinMode(left_motor_back,OUTPUT);
    pinMode(right_motor_go,OUTPUT);
    pinMode(right_motor_back,OUTPUT);
    Brake();
    pinMode(KEY_PIN,INPUT);
    pinMode(BEEP_PIN,OUTPUT);
    pinMode(TRACKER_LEFT,INPUT);
    pinMode(TRACKER_RIGHT,INPUT);
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
    switch(type){
        case PIV_ORDER_BRAKE: Brake(); break;
        case PIV_ORDER_FORWD: Go_forward(value,SOLO); break;
        case PIV_ORDER_BACK: Go_back(value,SOLO); break;
        case PIV_ORDER_ROTAT:
            value %= CRCL_SECT;
            if(value<CRCL_SECT/2) Go_left(value,SOLO);
            else Go_right(CRCL_SECT-value,SOLO);
            break;
        case PIV_ORDER_CLBRT:{
            int tries = value % 100;
            switch(value-tries){
                case CLBRT_RLTIV_VALUE: Calibrate_relative(tries); break;
                case CLBRT_STRAT_VALUE: Calibrate_straight(tries); break;
                case CLBRT_ROTAT_VALUE: Calibrate_rotate(tries); break;
                default: break;
            }
            break;
        }
        default:
            break;
    }
}

void Vehicle::Brake(){
    digitalWrite(left_motor_go,LOW);
    digitalWrite(left_motor_back,LOW);
    digitalWrite(right_motor_go,LOW);
    digitalWrite(right_motor_back,LOW);
}

void Vehicle::Start_forward(int left_speed,int right_speed){
    analogWrite(left_motor_go,left_speed);
    analogWrite(left_motor_back,0);
    analogWrite(right_motor_go,right_speed);
    analogWrite(right_motor_back,0);
}

void Vehicle::Start_forward(){
    Start_forward(left_speed_go,right_speed_go);
}

void Vehicle::Start_back(int left_speed,int right_speed){
    analogWrite(left_motor_go,0);
    analogWrite(left_motor_back,left_speed);
    analogWrite(right_motor_go,0);
    analogWrite(right_motor_back,right_speed);
}

void Vehicle::Start_back(){
    Start_back(left_speed_back,right_speed_back);
}

void Vehicle::Start_left(int right_speed){
    analogWrite(left_motor_go,0);
    analogWrite(left_motor_back,0);
    analogWrite(right_motor_go,right_speed);
    analogWrite(right_motor_back,0);
}

void Vehicle::Start_left(){
    Start_left(right_speed_go);
}

void Vehicle::Start_right(int left_speed){
    analogWrite(left_motor_go,left_speed);
    analogWrite(left_motor_back,0);
    analogWrite(right_motor_go,0);
    analogWrite(right_motor_back,0);
}

void Vehicle::Start_right(){
    Start_right(left_speed_go);
}

void Vehicle::Skew(bool go,bool side,int amount){
    Brake();
    if(go){
        if(side==RIGHT) Go_right(1,SOLO);
        else Go_left(1,SOLO);
    }
    else{
        Go_forward(3,SOLO);
        if(side==RIGHT) Go_left(1,SOLO);
        else Go_right(1,SOLO);
    }
    int& left_speed = go==GO ? left_speed_go : left_speed_back;
    int& right_speed = go==GO ? right_speed_go : right_speed_back;
    if(side==RIGHT){
        left_speed += amount;
    }
    else if(side==LEFT){
        right_speed += amount;
    }
    if(left_speed>SPEED_INIT && right_speed>SPEED_INIT){
        int difference = left_speed < right_speed ? left_speed - SPEED_INIT : right_speed - SPEED_INIT;
        left_speed -= difference;
        right_speed -= difference;
    }
    delay(CALI_DELAY);
    if(go) Start_forward();
    else Start_back();
}

bool Vehicle::Calibrate_relative_process(bool go){
    Alarm(500);
    bool unchanged = true;
    while(Tracker()!=WHITE_WHITE){
        switch(Tracker()){
            case BLACK_WHITE:
                Skew(go,LEFT,CALI_AMOUNT);
                unchanged = false;
                break;
            case WHITE_BLACK:
                Skew(go,RIGHT,CALI_AMOUNT);
                unchanged = false;
                break;
            default: break;
        }
    }
    Brake();
    return unchanged;
}

bool Vehicle::Calibrate_relative(int tries){
    bool go_done = false;
    bool back_done = false;
    for(; tries >= 1; tries -= 1){
        Start_forward();
        if(!WaitFor(Tracker,BLACK_BLACK,CALI_WTTM_STRT)){Brake(); return false;}
        go_done = Calibrate_relative_process(GO);
        
        Start_back();
        if(!WaitFor(Tracker,BLACK_BLACK,CALI_WTTM_STRT)){Brake(); return false;}
        back_done = Calibrate_relative_process(BACK);

        if(go_done && back_done) return true;
    }
    return false;
}

bool Vehicle::Calibrate_straight(int tries){
    for(; tries >= 1; tries -= 1){
        int waittime = 0;

        Start_forward();
        if(!WaitFor(Tracker,BLACK_BLACK,CALI_WTTM_STRT)){Brake(); return false;}
        if(!WaitFor(Tracker,WHITE_WHITE,0,waittime)){Brake(); return false;}
        Brake();
        centim_millis_go = waittime / CALI_CTMT;

        Start_back();
        if(!WaitFor(Tracker,BLACK_BLACK,CALI_WTTM_STRT)){Brake(); return false;}
        if(!WaitFor(Tracker,WHITE_WHITE,0,waittime)){Brake(); return false;}
        Brake();
        centim_millis_back = waittime / CALI_CTMT;
    }
    return true;
}

bool Vehicle::Calibrate_rotate(int tries){
    for(; tries >= 1; tries -= 1){
        int waittime = 0;
        int circle_time = 0;
        Start_forward();
        if(!WaitFor(Tracker,BLACK_BLACK,CALI_WTTM_STRT)){Brake(); return false;}
        Go_forward(5,SOLO);

        Start_left();
        if(!WaitFor(Tracker,WHITE_BLACK,0,waittime)){Brake(); return false;}
        delay(CALI_WTTM_ROTT);
        if(!WaitFor(Tracker,WHITE_BLACK,0,circle_time)){Brake(); return false;}
        circle_time += CALI_WTTM_ROTT;
        sector_millis_left = circle_time / CRCL_SECT;
        delay(circle_time-waittime); Brake();

        Start_right();
        if(!WaitFor(Tracker,BLACK_WHITE,0,waittime)){Brake(); return false;}
        delay(CALI_WTTM_ROTT);
        if(!WaitFor(Tracker,BLACK_WHITE,0,circle_time)){Brake(); return false;}
        circle_time += CALI_WTTM_ROTT;
        sector_millis_right = circle_time / CRCL_SECT;
        delay(circle_time-waittime); Brake();
    }
    return true;
}

void Vehicle::Go_forward(int centimeters,bool continuous){
    Start_forward();
    delay(centimeters * centim_millis_go);
    if(continuous==SOLO) Brake();
}

void Vehicle::Go_back(int centimeters,bool continuous){
    Start_back();
    delay(centimeters * centim_millis_back);
    if(continuous==SOLO) Brake();
}

void Vehicle::Go_left(int sectors,bool continuous){
    Start_left();
    delay(sectors * sector_millis_left);
    if(continuous==SOLO) Brake();
}

void Vehicle::Go_right(int sectors,bool continuous){
    Start_right();
    delay(sectors * sector_millis_right);
    if(continuous==SOLO) Brake();
}

int Vehicle::Tracker(){
    int left = digitalRead(TRACKER_LEFT);
    int right = digitalRead(TRACKER_RIGHT);
    return left + right * 2;
}

void Vehicle::Alarm(int milliseconds){
    digitalWrite(BEEP_PIN,HIGH);
    delay(milliseconds);
    digitalWrite(BEEP_PIN,LOW);
}