#ifndef __AP_HAL_RCINPUT_NAVIO2_H__
#define __AP_HAL_RCINPUT_NAVIO2_H__

#include "RCInput.h"

class Linux::RCInput_Navio2 : public Linux::RCInput
{
public:
    void init() override;
    void _timer_tick(void) override;
    RCInput_Navio2();
    ~RCInput_Navio2();

private:
    int open_channel(int ch);

    uint64_t _last_timestamp = 0l;
    static const size_t CHANNEL_COUNT = 8;
    int channels[CHANNEL_COUNT];
    char _channel_path_buf[256] = {0};
};

#endif // __AP_HAL_RCINPUT_NAVIO2_H__
