include ../../gnu/rules

BC_SRC_DIR=..
ARCH_LIB_DIR=../../STM32F10xFWLib

TARGET:=libzigbee.a

# current module directory
ZIGBEE_DIR:=./inc
ZIGBEE_SRC:=./src

# other modules' directory
PUBLIC_DIR:=$(BC_SRC_DIR)/other/public
STMLIB_DIR:=$(ARCH_LIB_DIR)/inc
IRQ_DIR:=$(BC_SRC_DIR)/other/irq
CONFIG_DIR:=$(BC_SRC_DIR)/other/config
FREERTOS_DIR:=$(BC_SRC_DIR)/freertos_src/include

INCLUDE += -I$(ZIGBEE_DIR)

INCLUDE += -I$(PUBLIC_DIR)
INCLUDE += -I$(STMLIB_DIR)
INCLUDE += -I$(IRQ_DIR)
INCLUDE += -I$(CONFIG_DIR)
INCLUDE += -I$(FREERTOS_DIR)

CFLAGS 	+= $(INCLUDE) $(MACRO)

C_SRC 	:=
A_SRC 	:=
SRC 	:=

C_OBJ 	:=
A_OBJ 	:=
OBJ 	:=

C_SRC 	+= ${wildcard $(ZIGBEE_SRC)/*.c}
A_SRC 	+= ${wildcard $(ZIGBEE_SRC)/*.s}
SRC 	+= $(C_SRC)
SRC 	+= $(A_SRC)

C_OBJ 	+= ${patsubst %.c, %.o, $(C_SRC)}
A_OBJ 	+= ${patsubst %.s, %.o, $(A_SRC)}
OBJ 	+= $(C_OBJ)
OBJ 	+= $(A_OBJ)

ALL: $(TARGET)

$(TARGET): $(OBJ)
	ar rcs $(TARGET) $(OBJ)

$(A_OBJ):%.o:%.s
	$(AS) -mcpu=cortex-m3 -mthumb $< -o $@

$(C_OBJ):%.o:%.c
	$(CC) $(CFLAGS) $< -o $@

clean:
	rm -f $(OBJ) $(TARGET) *.out *.list
