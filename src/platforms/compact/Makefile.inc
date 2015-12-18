CROSS_COMPILE ?= arm-none-eabi-
CC = $(CROSS_COMPILE)gcc
OBJCOPY = $(CROSS_COMPILE)objcopy

CFLAGS := $(patsubst -O%,,$(CFLAGS))
CFLAGS += -Os -Istm32/include -mcpu=cortex-m3 -mthumb \
	-DSTM32F1 -DBLACKMAGIC -I../libopencm3/include \
	-Iplatforms/stm32

LDFLAGS := $(LDFLAGS) -lopencm3_stm32f1 -Wl,--defsym,_stack=0x20005000 \
	-Wl,-T,../libopencm3/lib/stm32/f1/stm32f100x8.ld -nostartfiles -lc -lnosys \
	-Wl,-Map=mapfile -mthumb -mcpu=cortex-m3 -Wl,-gc-sections \
	-L../libopencm3/lib -L../libopencm3/lib/stm32/f1/

VPATH += platforms/stm32

SRC += 	cdcacm.c	\
	traceswo.c	\
	usbuart.c	\
	serialno.c	\
	timing.c	\

all:	blackmagic.bin

host_clean:
	-$(Q)$(RM) -f blackmagic.bin

