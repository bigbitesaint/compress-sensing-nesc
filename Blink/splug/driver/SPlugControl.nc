#include "main.h"

interface SPlugControl
{
	command error_t init();
    command void setState(uint8_t state);
    command uint8_t getState();
    command error_t write(uint8_t address, uint32_t value);
    command uint32_t read(uint8_t address);
}
