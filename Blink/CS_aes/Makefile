COMPONENT=CSAppC
ROOT_DIR= $(COMPRESSIVE_SENSING)/tinyos-2.1.1
SOURCE_ROOT= $(COMPRESSIVE_SENSING)/Blink
CFLAGS += -DPRINTFUART_ENABLED -DTEST
CFLAGS += -I$(SOURCE_ROOT)/splug -I$(SOURCE_ROOT) -I$(SOURCE_ROOT)/splug/driver -I$(SOURCE_ROOT)/splug/driver/spi -I$(SOURCE_ROOT)/reliable -I$(SOURCE_ROOT)/CS -I$(SOURCE_ROOT)/soft_aes

# compressive sensing, sparse-binary matrix
CFLAGS += -DUSE_SMATRIX

# compressive sensing, gaussian matrix
#CFLAGS += -DUSE_GMATRIX
#PFLAGS += $(COMPRESSIVE_SENSING)/Blink/CS/box_muller/box_muller.c

# compressive sensing, bernoulli matrix
#CFLAGS += -DUSE_BMATRIX

CFLAGS += -DCC2420_DEF_CHANNEL=17
CFLAGS += -DCC2420_DEF_RFPOWER=4
CFLAGS += -DTOSH_DATA_LENGTH=120
#PFLAGS += $(COMPRESSIVE_SENSING)/Blink/slzw/slzw.c
include $(MAKERULES)

