#include "Vehicle.h"

Vehicle::Vehicle(uint8_t c,uint8_t s,int lg,int lb,int rg,int rb):Signode(c,s){
    left_motor_go = lg;
    left_motor_back = lb;
    right_motor_go = rg;
    right_motor_back = rb;

    left_speed_go = SPEED_INIT;
    right_speed_go = SPEED_INIT;

    centim_millis_go = CMMSG_INIT;
    sector_millis_left = STMSG_INIT;
    sector_millis_right = STMSG_INIT;

    position = POSTN_INIT;
    pipe = PIV_PPID;
}

void Vehicle::Setup(){
    pinMode(left_motor_go,OUTPUT);
    pinMode(left_motor_back,OUTPUT);
    pinMode(right_motor_go,OUTPUT);
    pinMode(right_motor_back,OUTPUT);
    digitalWrite(left_motor_back,LOW);
    digitalWrite(right_motor_back,LOW);
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
        case PIV_ORDER_ROTAT:
            value %= 360;
            if(value<180) Go_left(value,SOLO);
            else Go_right(360-value,SOLO);
            break;
        case PIV_ORDER_TRACK:
            switch(value){
                case TRACK_THIN_VALUE: Tracking_thin(); break;
                case TRACK_CRUDE_VALUE: Tracking_crude(); break;
                default: break;
            } break;
        case PIV_ORDER_CLBRT:
            switch(value){
                case CLBRT_ROTAT_VALUE: Calibrate_rotation(); break;
                case CLBRT_RLTIV_VALUE: Calibrate_relative(); break;
                case CLBRT_POSTN_VALUE: Calibrate_position(); break;
                case CLBRT_STRAT_VALUE: Calibrate_straight(); break;
                default: break;
            } break;
        default: break;
    }
}

void Vehicle::Alarm(unsigned long milliseconds){
    digitalWrite(BEEP_PIN,HIGH);
    delay(milliseconds);
    digitalWrite(BEEP_PIN,LOW);
}

int Vehicle::Tracker(){
    int left = digitalRead(TRACKER_LEFT);
    int right = digitalRead(TRACKER_RIGHT);
    return left + right * 2;
}

void Vehicle::Brake(){
    digitalWrite(left_motor_go,LOW);
    digitalWrite(right_motor_go,LOW);
}

void Vehicle::Start_forward(int left_speed,int right_speed){
    analogWrite(left_motor_go,left_speed);
    analogWrite(right_motor_go,right_speed);
}

void Vehicle::Start_forward(bool accelerate){
    int left_speed = accelerate==SLOW ? left_speed_go/2 : (left_speed_go+SPEED_MAX)*2;
    int right_speed = accelerate==SLOW ? right_speed_go/2 : (right_speed_go+SPEED_MAX)*2;
    Start_forward(left_speed,right_speed);
}

void Vehicle::Start_forward(){
    Start_forward(left_speed_go,right_speed_go);
}

void Vehicle::Start_left(int right_speed){
    analogWrite(left_motor_go,0);
    analogWrite(right_motor_go,right_speed);
}

void Vehicle::Start_left(){
    Start_left(right_speed_go);
}

void Vehicle::Start_right(int left_speed){
    analogWrite(left_motor_go,left_speed);
    analogWrite(right_motor_go,0);
}

void Vehicle::Start_right(){
    Start_right(left_speed_go);
}

void Vehicle::Tracking_thin(){
    //从轨道上正位开始
    while(Tracker()!=BLACK_BLACK){
        switch(Tracker()){
            case WHITE_WHITE: Start_forward(); break;
            case BLACK_WHITE: Start_left(); break;
            case WHITE_BLACK: Start_right(); break;
            default: break;
        }
    }
    Brake();
}

void Vehicle::Tracking_crude(){
    //从轨道上正位开始
    while(Tracker()!=WHITE_WHITE){
        switch(Tracker()){
            case BLACK_BLACK: Start_forward(); break;
            case BLACK_WHITE: Start_left(); break;
            case WHITE_BLACK: Start_right(); break;
            default: break;
        }
    }
    Brake();
}

bool Vehicle::Calibrate_rotation(){
    unsigned long wait_time = 0;
    unsigned long semi_time_1 = 0;
    unsigned long semi_time_2 = 0;
    unsigned long circle_time = 0;
    //从轨道上正位开始
    if(Tracker()!=WHITE_WHITE){return false;}

    Start_left();
    if(!WaitFor(Tracker,WHITE_BLACK,0,wait_time)){Brake(); return false;}
    delay(CALI_WTTM);
    if(!WaitFor(Tracker,WHITE_BLACK,0,semi_time_1)){Brake(); return false;}
    delay(CALI_WTTM);
    if(!WaitFor(Tracker,WHITE_BLACK,0,semi_time_2)){Brake(); return false;}
    circle_time = semi_time_1 + semi_time_2 + CALI_WTTM + CALI_WTTM;
    sector_millis_left = circle_time / CRCL_SECT;
    delay(circle_time-wait_time-BRAKE_TIME); Brake();

    delay(CALI_WTTM);

    Start_right();
    if(!WaitFor(Tracker,BLACK_WHITE,0,wait_time)){Brake(); return false;}
    delay(CALI_WTTM);
    if(!WaitFor(Tracker,BLACK_WHITE,0,semi_time_1)){Brake(); return false;}
    delay(CALI_WTTM);
    if(!WaitFor(Tracker,BLACK_WHITE,0,semi_time_2)){Brake(); return false;}
    circle_time = semi_time_1 + semi_time_2 + CALI_WTTM + CALI_WTTM;
    sector_millis_right = circle_time / CRCL_SECT;
    delay(circle_time-wait_time-BRAKE_TIME); Brake();
    Alarm(500);
    return true;
}

void Vehicle::Calibrate_relative(){
    unsigned long forward_time = 0;
    unsigned long left_time = 0;
    unsigned long right_time = 0;
    unsigned long last_time = millis();
    left_speed_go = SPEED_INIT;
    right_speed_go = SPEED_INIT;
    //从轨道上正位开始
    int last_tracker = WHITE_WHITE;
    Start_forward();
    while(Tracker()!=BLACK_BLACK){
        if(Tracker()==last_tracker) continue;
        switch(last_tracker){
            case WHITE_WHITE: forward_time += millis()-last_time; break;
            case BLACK_WHITE: left_time += millis()-last_time; break;
            case WHITE_BLACK: right_time += millis()-last_time; break;
            default: break;
        }
        last_time = millis();
        last_tracker = Tracker();
        switch(Tracker()){
            case WHITE_WHITE: Start_forward(); break;
            case BLACK_WHITE: Start_left(); break;
            case WHITE_BLACK: Start_right(); break;
            default: break;
        }
    }
    while(Tracker()!=WHITE_WHITE){}
    Brake();
    unsigned long left_motor_time = forward_time + right_time;
    unsigned long right_motor_time = forward_time + left_time;
    //涉及到int和unsigned long之间的转换
    left_speed_go = SPEED_INIT * left_motor_time / (left_motor_time + right_motor_time) * 2;
    right_speed_go = SPEED_INIT * right_motor_time / (left_motor_time + right_motor_time) * 2;
    Alarm(500);
}

void Vehicle::Calibrate_position(){
    position = -1;
    //从白色区域开始
    Start_forward();
    while(Tracker()!=BLACK_BLACK){}
    Brake();
    unsigned long start_time = millis();
    while(millis()-start_time<=2000){
        switch(Tracker()){
            case BLACK_BLACK: Start_forward(); break;
            case BLACK_WHITE: Start_left(); break;
            case WHITE_BLACK: Start_right(); break;
            default: break;
        }
    }
    unsigned long last_time = millis();
    int last_tracker = BLACK_BLACK;
    int this_tracker = BLACK_BLACK;
    int normal_tracker = BLACK_BLACK;
    while(millis()-start_time<=20000){
        this_tracker = Tracker();
        if(this_tracker!=WHITE_WHITE && this_tracker!=BLACK_BLACK && this_tracker!=last_tracker){
            last_time = millis();
            last_tracker = this_tracker;
        }
        switch(this_tracker){
            case BLACK_BLACK: Start_forward(); break;
            case BLACK_WHITE: Start_left(); break;
            case WHITE_BLACK: Start_right(); break;
            default: break;
        }
        if(millis()-last_time>=2000){
            break;
        }
    }
    normal_tracker = last_tracker;    
    while(Tracker()!=3-normal_tracker && Tracker()!=WHITE_WHITE){
        switch(Tracker()){
            case BLACK_BLACK: Start_forward(); break;
            case BLACK_WHITE: Start_left(); break;
            case WHITE_BLACK: Start_right(); break;
            default: break;
        }
    }
    switch(normal_tracker){
        case BLACK_WHITE: Go_left(120,SOLO); break;
        case WHITE_BLACK: Go_right(120,SOLO); break;
        default: break;
    }
    // Tracking_crude();
    // Start_forward();
    // while(Tracker()==WHITE_WHITE){}
    // last_tracker = WHITE_WHITE;
    // while(Tracker()!=BLACK_BLACK){
    //     if(Tracker()==last_tracker) continue;
    //     last_tracker = Tracker();
    //     switch(last_tracker){
    //         case BLACK_WHITE: position+=1; break;
    //         case WHITE_BLACK: position+=POSTN_GROUP; break;
    //         case WHITE_WHITE: break;
    //         default: break;
    //     }
    // }
    // while(Tracker()==BLACK_BLACK){}
    // Brake();
    Alarm(500);
}

void Vehicle::Calibrate_straight(){
    unsigned long go_time = 0;
    //从白色区域开始
    Start_forward();
    while(Tracker()!=BLACK_BLACK){}
    go_time = millis();
    while(Tracker()!=WHITE_WHITE){}
    Tracking_thin();
    go_time = millis() - go_time;
    centim_millis_go = go_time / CALI_CTMT;
    Alarm(500);
}

void Vehicle::Go_forward(int centimeters,bool continuous){
    unsigned long delay_time = centimeters * centim_millis_go;
    Start_forward();
    delay(delay_time);
    if(continuous==SOLO) Brake();
}

void Vehicle::Go_left(int degrees,bool continuous){
    unsigned long delay_time = degrees * sector_millis_left / (360/CRCL_SECT);
    Start_left();
    delay(delay_time);
    if(continuous==SOLO) Brake();
}

void Vehicle::Go_right(int degrees,bool continuous){
    unsigned long delay_time = degrees * sector_millis_right / (360/CRCL_SECT);
    Start_right();
    delay(delay_time);
    if(continuous==SOLO) Brake();
}