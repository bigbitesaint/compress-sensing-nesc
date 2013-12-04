#ifndef MAIN_H
#define MAIN_H

#include "printfZ1.h"
#include "splug/driver/spi/ADE7763.h"

#define SET(config, mask) config->status |= mask
#define CLR(config, mask) config->status &= ~mask
#define GET(config, mask) config->status & mask

//==================================================
#define BASERX_ID 90
#define BASERX_CHANNEL 26
#define BASERX_SERIAL A700azND
#define BASERX_PLATFORM telosb

#define BASETX_ID 89
#define BASETX_CHANNEL 16
#define BASETX_SERIAL A4002327
#define BASETX_PLATFORM telosb
//==================================================

#define RETRANSMISSION_MAX 10

#define INTERVAL_UPPER 500
#define INTERVAL_LOWER 500

#define INTERVAL_MIN 10

#define CHANNEL_MAX 26
#define CHANNEL_MIN 11

#define RFPOWER_MAX 31
#define RFPOWER_MIN 1

#define READ_REG1 TEMP
#define READ_REG2 VRMS
#define WRITE_REG TMODE
#define WRITE_VAL 0
#define REG_MAX DIEREV

#define SPLUG_DATA_BIT 0x08
#define SPLUG_STATE_BIT 0x04
#define SPLUG_RESET_BIT 0x02
#define SPLUG_SET_BIT 0x01

enum {
  AM_SPLUG_CFG_MSG = 123,
  AM_SPLUG_DATA_MSG = 124,
  AM_PIR_DATA_MSG = 125,
};

enum {
  SPLUG_OFF_STATE = 0,
  SPLUG_ON_STATE = 1,
};

typedef nx_struct splug_data_msg {
  // send periodically
  nx_uint16_t sensor_id;
  nx_uint8_t seq_no;
  nx_uint32_t read_val1;
  nx_uint32_t read_val2;
} splug_data_msg_t;

typedef nx_struct splug_cfg_msg {
  // send on demand, contain all configurable information
  nx_uint16_t sensor_id;
  nx_uint16_t baserx_id;
  nx_uint16_t interval_upper;
  nx_uint16_t interval_lower;
  nx_uint8_t channel;
  nx_uint8_t rfpower;
  nx_uint8_t read_reg1;
  nx_uint8_t read_reg2;
  nx_uint8_t write_reg;
  nx_uint16_t write_val;
  nx_uint8_t status; //rsved, rsved, rsved, rsved, data, state, reset, set
} splug_cfg_msg_t;

typedef nx_struct pir_data_msg {
  nx_uint16_t sensor_id;
  nx_uint8_t seq_no;
} pir_data_msg_t;

void printSPlugDataMsg(splug_data_msg_t *data, int8_t rssi, int8_t lqi) {
  printfz1("%u,%u,%lu,%lu,%d,%d\n", data->sensor_id, data->seq_no, 
           data->read_val1, data->read_val2, rssi, lqi);
}

void initSPlugCfgMsg(splug_cfg_msg_t *cfg) {
  cfg->sensor_id = TOS_NODE_ID;
  cfg->baserx_id = BASERX_ID;
  cfg->interval_lower = INTERVAL_LOWER;
  cfg->interval_upper = INTERVAL_UPPER;
  cfg->channel = CHANNEL_MAX;
  cfg->rfpower = RFPOWER_MAX;
  cfg->read_reg1 = READ_REG1;
  cfg->read_reg2 = READ_REG2;
  cfg->write_reg = WRITE_REG;
  cfg->write_val = WRITE_VAL;
  SET(cfg, SPLUG_DATA_BIT);
  SET(cfg, SPLUG_STATE_BIT);
}

#define BYTETOBIN(byte)  \
  (byte & 0x80 ? 1 : 0), \
  (byte & 0x40 ? 1 : 0), \
  (byte & 0x20 ? 1 : 0), \
  (byte & 0x10 ? 1 : 0), \
  (byte & 0x08 ? 1 : 0), \
  (byte & 0x04 ? 1 : 0), \
  (byte & 0x02 ? 1 : 0), \
  (byte & 0x01 ? 1 : 0) 

void printSPlugCfgMsg(splug_cfg_msg_t *cfg, int8_t rssi, int8_t lqi) {
  printfz1("%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%d%d%d%d%d%d%d%d,%d,%d\n", 
           cfg->sensor_id, cfg->baserx_id, cfg->interval_lower, cfg->interval_upper, 
           cfg->channel, cfg->rfpower, cfg->read_reg1, cfg->read_reg2, cfg->write_reg,
           cfg->write_val, BYTETOBIN(cfg->status), rssi, lqi);
}

void printPIRDataMsg(pir_data_msg_t *data, int8_t rssi, int8_t lqi) {
  printfz1("%u,%u,%d,%d\n", data->sensor_id, data->seq_no, rssi, lqi);  
}

#endif /* MAIN_H */
