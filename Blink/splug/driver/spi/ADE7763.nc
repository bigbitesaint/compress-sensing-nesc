interface ADE7763 {
  command void init(); 
  command void cs_high();
  command void cs_low();

  command void write(uint8_t address, uint32_t value, uint8_t size);
  command uint32_t read(uint8_t address, uint8_t size);
}

