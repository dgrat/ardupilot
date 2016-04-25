#pragma once

#include "RCInput.h"

#define CHANNEL_COUNT 8

class Linux::RCInput_Navio2 : public Linux::RCInput
{
public:
    template<int iNumChans = CHANNEL_COUNT>
    void init() override;
    
    template<int iNumChans = CHANNEL_COUNT>
    void _timer_tick(void) override;
    
    RCInput_Navio2();
    ~RCInput_Navio2();

private:
    int open_channel(int ch);

    uint64_t _last_timestamp = 0l;
    int _channels[CHANNEL_COUNT];
    uint16_t _periods[CHANNEL_COUNT] = {0};
};
