#ifndef _Pivot_
#define _Pivot_
#include "Signode.h"

class Pivot : public Signode {
public:
    Pivot(uint8_t c,uint8_t s);
    void Setup();
    void Standby();
private:
    void GrantPipeId(int target);
private:
    int car_count;
};

#endif