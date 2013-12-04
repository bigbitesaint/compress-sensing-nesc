#include "printfZ1.h"

#define GPIO_SET(p) (*TCAST(volatile TYPE_PORT_OUT* ONE, p.port_out_addr)) |= p.pin
#define GPIO_CLR(p) (*TCAST(volatile TYPE_PORT_OUT* ONE, p.port_out_addr)) &= ~p.pin
#define GPIO_TOGGLE(p) (*TCAST(volatile TYPE_PORT_OUT* ONE, p.port_out_addr)) ^= p.pin
#define GPIO_GET(p) ((*TCAST(volatile TYPE_PORT_IN* ONE, p.port_in_addr)) & p.pin)
#define GPIO_MAKEINPUT(p) (*TCAST(volatile TYPE_PORT_DIR* ONE, p.port_dir_addr)) &= ~p.pin
#define GPIO_MAKEOUTPUT(p) (*TCAST(volatile TYPE_PORT_DIR* ONE, p.port_dir_addr)) |= p.pin

module ADE7763P {
  provides interface ADE7763;

  uses interface Leds;
  uses interface LocalTime<T32khz> as LocalTime;
  //uses interface BusyWait<TMicro, uint16_t>;
}

implementation {
  typedef struct {
    uint16_t port_in_addr;
    uint16_t port_out_addr;
    uint16_t port_dir_addr;
    uint16_t pin;
  } mygpio;

  mygpio sck = {P4IN_, P4OUT_, P4DIR_, 0x01<<0};
  mygpio miso = {P3IN_, P3OUT_, P3DIR_, 0x01<<5};
  mygpio mosi = {P3IN_, P3OUT_, P3DIR_, 0x01<<4};
  mygpio csb = {P3IN_, P3OUT_, P3DIR_, 0x01<<0};

  inline void spi_clk() {
    GPIO_SET(sck);
    //call BusyWait.wait(10);
    GPIO_CLR(sck);
  }	

  uint32_t read(uint8_t size) {
    uint32_t data = 0;
    uint32_t i = 0x01;

    i <<= (size-1);
    do {
      spi_clk();
      if (GPIO_GET(mosi))
        data += i;
      i >>= 1;
    } while(i);
    return data;
  }

  void write(uint32_t data, uint8_t size) {
    uint32_t i = 0x01;

    i <<= (size-1);
    do {
      if (data & i)
        GPIO_SET(miso);
      else
        GPIO_CLR(miso);
      i >>= 1;
      spi_clk();
    } while(i);
  }

  command void ADE7763.init() {
    atomic {
      GPIO_MAKEOUTPUT(sck);
      GPIO_MAKEOUTPUT(csb);
      GPIO_MAKEINPUT(mosi);
      GPIO_MAKEOUTPUT(miso);
      GPIO_CLR(mosi);
      GPIO_SET(csb);
      GPIO_CLR(sck);
    }     
  }

  command void ADE7763.cs_high() {atomic GPIO_SET(csb);}

  command void ADE7763.cs_low() {atomic GPIO_CLR(csb);}

  command void ADE7763.write(uint8_t address, uint32_t value, uint8_t size) {
    uint32_t _address = address + 0x80; //MSB = 1 to indicate a write function
    _address <<= size;
    _address += value;
    write(_address, size + 8);
  }

  command uint32_t ADE7763.read(uint8_t address, uint8_t size) {
    uint32_t data = 0;
    write(address, ADDRESS_SIZE);
    data = read(size);
    return data;
  }
}
