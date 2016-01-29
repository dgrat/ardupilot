#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX

#include "AnalogIn_Navio.h"

#include <cstdlib>
#include <unistd.h>
#include <cstdio>
#include <errno.h>

extern const AP_HAL::HAL& hal;

#define RCIN_BASE_PATH "/sys/kernel/rcio/adc"
#define RCIN_PATH_MAX (sizeof(RCIN_BASE_PATH) + sizeof("XXXX") - 1)


void NavioAnalogSource::set_channel(int16_t pin)
{
    if(pin < 0) {
        return;
    }
  
    memset(_channel_path_buf, 0, sizeof(_channel_path_buf));
    if (sprintf(_channel_path_buf, "%s/ch%d", RCIN_BASE_PATH, pin) == -1) {
        AP_HAL::panic("NavioAnalogSource: sprintf() in set_channel()\n");
    }

    if (_fd >= 0) {
        ::close(_fd);
    }

    _fd = ::open(_channel_path_buf, O_RDONLY);

    if (_fd < 0) {
        hal.console->printf("%s not opened: %s", _channel_path_buf, strerror(errno));
    }
}

NavioAnalogSource::NavioAnalogSource() : _pin(-1)
{
    set_channel(-1);
}

NavioAnalogSource::NavioAnalogSource(int16_t pin):
    _pin(pin)
{
    set_channel(pin);
}

int16_t NavioAnalogSource::get_pin() const 
{
    return _pin;
}

void NavioAnalogSource::set_pin(uint8_t pin)
{
    if (_pin == pin) {
        return;
    }

    set_channel(pin);

    _pin = pin;
}

float NavioAnalogSource::read_average()
{
    return read_latest();
}

float NavioAnalogSource::read_latest()
{
    return _value;
}

float NavioAnalogSource::voltage_average()
{
    char buffer[RCIN_PATH_MAX] = {0};

    int ret = ::pread(_fd, buffer, sizeof(buffer), 0);
    if (ret > 0) {
        _value = ((float) atoi(buffer)) / 1000;
    }

    return _value;
}

float NavioAnalogSource::voltage_latest()
{
    return _value;
}

float NavioAnalogSource::voltage_average_ratiometric()
{
    return _value;
}

extern const AP_HAL::HAL& hal;

NavioAnalogIn::NavioAnalogIn()
{
    _channels_number = NAVIO_ADC_MAX_CHANNELS;
}

float NavioAnalogIn::board_voltage(void)
{
    auto voltage = _board_voltage_pin->voltage_average();
    return voltage;
}

float NavioAnalogIn::servorail_voltage(void)
{
    auto voltage = _servorail_pin->voltage_average();
    return voltage;
}

AP_HAL::AnalogSource* NavioAnalogIn::channel(int16_t pin)
{
    for (uint8_t j = 0; j < _channels_number; j++) {
        if (_channels[j].get_pin() == -1) {
            _channels[j].set_pin(pin);
            return &_channels[j];
        }
    }

    return NULL;
}

void NavioAnalogIn::init()
{
    _board_voltage_pin = channel(0);
    _servorail_pin = channel(1);

    hal.scheduler->suspend_timer_procs();
    hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&NavioAnalogIn::_update, void));
    hal.scheduler->resume_timer_procs();
}

void NavioAnalogIn::_update()
{
    if (AP_HAL::micros() - _last_update_timestamp < 100000) {
        return;
    }

    _last_update_timestamp = AP_HAL::micros();
}

#endif
