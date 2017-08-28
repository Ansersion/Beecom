ARM_GCC_MAIN_DIR=/opt/gcc-arm-none-eabi-5_3-2016q1
PREFIX=$(ARM_GCC_MAIN_DIR)/bin/arm-none-eabi
# PREFIX=arm-linux-gnueabi
# PREFIX=arm-linux-gnueabi

BC_SRC_DIR=..
ARCH_LIB_DIR=../../STM32F10xFWLib

TARGET=libdatahub.a

# CORE_DIR=../core
# ARCH_DIR=../arch
# KERNEL_DIR=../kernel

DATAHUB_DIR=./inc
PUBLIC_DIR=$(BC_SRC_DIR)/other/public
STMLIB_DIR=$(ARCH_LIB_DIR)/inc
IRQ_DIR=$(BC_SRC_DIR)/other/irq
CONFIG_DIR=$(BC_SRC_DIR)/other/config
FREERTOS_DIR=$(BC_SRC_DIR)/freertos_src/include

AS=$(PREFIX)-as
CC=$(PREFIX)-gcc
LD=$(PREFIX)-ld
OBJCOPY=$(PREFIX)-objcopy
OBJDUMP=$(PREFIX)-objdump

INCLUDE=
MACRO=

INCLUDE += -I$(DATAHUB_DIR)
INCLUDE += -I$(PUBLIC_DIR)
INCLUDE += -I$(STMLIB_DIR)
INCLUDE += -I$(IRQ_DIR)
INCLUDE += -I$(CONFIG_DIR)
INCLUDE += -I$(FREERTOS_DIR)

MACRO += -DRVDS_ARMCM3_LM3S102
MACRO += -DportFORCE_INLINE=inline

CFLAGS 	=

CFLAGS 	+=-g -T $(BC_SRC_DIR)/gnu/sct.ld -Wall -c -mcpu=cortex-m3 -mthumb -nostartfiles $(INCLUDE) $(MACRO)

C_SRC 	=
A_SRC 	=
SRC 	=

C_OBJ 	=
A_OBJ 	=
OBJ 	=

C_SRC 	+= ${wildcard src/*.c}
A_SRC 	+= ${wildcard src/*.s}
SRC 	+= $(C_SRC)
SRC 	+= $(A_SRC)

C_OBJ 	+= ${patsubst %.c, %.o, $(C_SRC)}
A_OBJ 	+= ${patsubst %.s, %.o, $(A_SRC)}
OBJ 	+= $(C_OBJ)
OBJ 	+= $(A_OBJ)

ALL: $(TARGET)

$(TARGET): $(OBJ)
	ar rcs $(TARGET) $(OBJ)
	cscope -Rbq

$(A_OBJ):%.o:%.s
	$(AS) -mcpu=cortex-m3 -mthumb $< -o $@

$(C_OBJ):%.o:%.c
	$(CC) $(CFLAGS) $< -o $@

clean:
	rm -f $(OBJ) $(TARGET) *.out *.list
