COMPONENT=BLAppC
ROOT_DIR= /home/adsc/Desktop/frank/tinyos-2.1.1
SOURCE_ROOT= /home/adsc/Desktop/frank/Blink
CFLAGS += -DPRINTFUART_ENABLED
CFLAGS += -I$(SOURCE_ROOT)/soft_aes  -I$(SOURCE_ROOT)/splug -I$(SOURCE_ROOT) -I$(SOURCE_ROOT)/splug/driver -I$(SOURCE_ROOT)/splug/driver/spi -I$(SOURCE_ROOT)/reliable -I$(SOURCE_ROOT)/baseline



# baseline, iterative-transform
CFLAGS += -DITERATIVE

#CFLAGS += -DCC2420_DEF_CHANNEL=17

CFLAGS += -DTOSH_DATA_LENGTH=120
include $(MAKERULES)

