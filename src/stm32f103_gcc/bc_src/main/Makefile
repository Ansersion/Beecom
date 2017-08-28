include ../../gnu/rules

BC_SRC_DIR=..
ARCH_LIB_DIR=../../STM32F10xFWLib

TARGET:=libdatahub.a

# current module directory
MAIN_DIR:=./inc

# other modules' directory
DATAHUB_DIR:=$(BC_SRC_DIR)/datahub/inc
PUBLIC_DIR:=$(BC_SRC_DIR)/other/public
STMLIB_DIR:=$(ARCH_LIB_DIR)/inc
IRQ_DIR:=$(BC_SRC_DIR)/other/irq
CONFIG_DIR:=$(BC_SRC_DIR)/other/config
FREERTOS_DIR:=$(BC_SRC_DIR)/freertos_src/include
WIFI_DIR:=$(BC_SRC_DIR)/wifi/inc
APP_AGENT_DIR:=$(BC_SRC_DIR)/phone_app/inc
ZIGBEE_DIR:=$(BC_SRC_DIR)/zigbee/inc
BLUETOOTH_DIR:=$(BC_SRC_DIR)/bluetooth/inc
NET_SERVER_DIR:=$(BC_SRC_DIR)/net_server/inc

INCLUDE += -I$(MAIN_DIR)

INCLUDE += -I$(DATAHUB_DIR)
INCLUDE += -I$(PUBLIC_DIR)
INCLUDE += -I$(STMLIB_DIR)
INCLUDE += -I$(IRQ_DIR)
INCLUDE += -I$(CONFIG_DIR)
INCLUDE += -I$(FREERTOS_DIR)
INCLUDE += -I$(WIFI_DIR)
INCLUDE += -I$(APP_AGENT_DIR)
INCLUDE += -I$(ZIGBEE_DIR)
INCLUDE += -I$(BLUETOOTH_DIR)
INCLUDE += -I$(NET_SERVER_DIR)

CFLAGS 	:=

CFLAGS 	+=-g -T $(BC_SRC_DIR)/gnu/sct.ld -Wall -c -mcpu=cortex-m3 -mthumb -nostartfiles $(INCLUDE) $(MACRO)

C_SRC 	:=
A_SRC 	:=
SRC 	:=

C_OBJ 	:=
A_OBJ 	:=
OBJ 	:=

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
