include gnu/rules

DATAHUB_DIR     :=bc_src/datahub
BLUETOOTH_DIR   :=bc_src/bluetooth
FREERTOS_DIR    :=bc_src/freertos_src
NET_SERVER_DIR  :=bc_src/net_server
PHONE_APP_DIR   :=bc_src/phone_app
ZIGBEE_DIR      :=bc_src/zigbee
MAIN_DIR        :=bc_src/main
WIFI_DIR        :=bc_src/wifi
CONFIG_DIR      :=bc_src/other/config
IRQ_DIR         :=bc_src/other/irq
PUBLIC_DIR      :=bc_src/other/public
STMLIB_DIR      :=STM32F10xFWLib

TARGET_BIN=beecom.bin
TARGET_HEX=beecom.hex

LIB_DATAHUB     :=$(DATAHUB_DIR)/libdatahub.a
LIB_BLUETOOTH   :=$(BLUETOOTH_DIR)/libbluetooth.a
LIB_FREERTOS    :=$(FREERTOS_DIR)/libfreertos.a
LIB_NET_NETVER  :=$(NET_SERVER_DIR)/libnet_server.a
LIB_PHONE_APP   :=$(PHONE_APP_DIR)/libphone_app.a
LIB_ZIGBEE      :=$(ZIGBEE_DIR)/libzigbee.a
LIB_MAIN        :=$(MAIN_DIR)/libmain.a
LIB_WIFI        :=$(WIFI_DIR)/libwifi.a
LIB_CONFIG      :=$(CONFIG_DIR)/libconfig.a
LIB_IRQ         :=$(IRQ_DIR)/libirq.a
LIB_PUBLIC      :=$(PUBLIC_DIR)/libpublic.a
LIB_STMLIB      :=$(STMLIB_DIR)/libstm32.a

INCLUDE += -I$(DATAHUB_DIR)/inc
INCLUDE += -I$(BLUETOOTH_DIR)/inc
INCLUDE += -I$(FREERTOS_DIR)/include
INCLUDE += -I$(NET_SERVER_DIR)/inc
INCLUDE += -I$(PHONE_APP_DIR)/inc
INCLUDE += -I$(ZIGBEE_DIR)/inc
INCLUDE += -I$(MAIN_DIR)/inc
INCLUDE += -I$(WIFI_DIR)/inc
INCLUDE += -I$(CONFIG_DIR)/inc
INCLUDE += -I$(IRQ_DIR)/inc
INCLUDE += -I$(PUBLIC_DIR)/inc
INCLUDE += -I$(STMLIB_DIR)/inc

CFLAGS 	:= 
CFLAGS  += -T ./gnu/sct.ld -mcpu=cortex-m3 -mthumb -flto -fno-builtin -nostartfiles -Wall -mcpu=cortex-m3 -mthumb -nostartfiles --specs=rdimon.specs 
MACRO   += -DSTARTUP_FROM_RESET

CFLAGS += $(INCLUDE)
CFLAGS += $(MACRO)

ALL: $(TARGET_BIN)

$(TARGET_BIN): $(LIB_DATAHUB)     $(LIB_BLUETOOTH)   $(LIB_FREERTOS)    $(LIB_NET_NETVER)  $(LIB_PHONE_APP)   $(LIB_ZIGBEE)      $(LIB_MAIN)        $(LIB_WIFI)        $(LIB_CONFIG)      $(LIB_IRQ)         $(LIB_PUBLIC)      $(LIB_STMLIB)

	ar xv $(LIB_DATAHUB)     
	ar xv $(LIB_BLUETOOTH)   
	ar xv $(LIB_FREERTOS)    
	ar xv $(LIB_NET_NETVER)  
	ar xv $(LIB_PHONE_APP)   
	ar xv $(LIB_ZIGBEE)      
	ar xv $(LIB_MAIN)        
	ar xv $(LIB_WIFI)        
	ar xv $(LIB_CONFIG)      
	ar xv $(LIB_IRQ)         
	ar xv $(LIB_PUBLIC)      
	ar xv $(LIB_STMLIB)      

	$(CC) $(CFLAGS) -o beecom.out *.o
	# $(STRIP) beecom.out
	$(OBJCOPY) -Obinary beecom.out $(TARGET_BIN)
	$(OBJCOPY) -Oihex  beecom.out $(TARGET_HEX)
	rm -f *.o
	rm -f $(LIB_DATAHUB)     $(LIB_BLUETOOTH)   $(LIB_FREERTOS)    $(LIB_NET_NETVER)  $(LIB_PHONE_APP)   $(LIB_ZIGBEE)      $(LIB_MAIN)        $(LIB_WIFI)        $(LIB_CONFIG)      $(LIB_IRQ)         $(LIB_PUBLIC)      $(LIB_STMLIB)
	$(OBJDUMP) -S beecom.out > beecom.list
	$(OBJDUMP) -D beecom.out > beecom.dis
	cscope -Rbq

$(LIB_DATAHUB):
	(cd $(DATAHUB_DIR); make)	
$(LIB_BLUETOOTH):
	(cd $(BLUETOOTH_DIR); make)	
$(LIB_FREERTOS):
	(cd $(FREERTOS_DIR); make)	
$(LIB_NET_NETVER):
	(cd $(NET_SERVER_DIR); make)	
$(LIB_PHONE_APP):
	(cd $(PHONE_APP_DIR); make)	
$(LIB_ZIGBEE):
	(cd $(ZIGBEE_DIR); make)	
$(LIB_MAIN):
	(cd $(MAIN_DIR); make)	
$(LIB_WIFI):
	(cd $(WIFI_DIR); make)	
$(LIB_CONFIG):
	(cd $(CONFIG_DIR); make)	
$(LIB_IRQ):
	(cd $(IRQ_DIR); make)	
$(LIB_PUBLIC):
	(cd $(PUBLIC_DIR); make)	
$(LIB_STMLIB):
	(cd $(STMLIB_DIR); make)	

clean:
	(cd $(DATAHUB_DIR); make clean)	
	(cd $(BLUETOOTH_DIR); make clean)	
	(cd $(FREERTOS_DIR); make clean)	
	(cd $(NET_SERVER_DIR); make clean)	
	(cd $(PHONE_APP_DIR); make clean)	
	(cd $(ZIGBEE_DIR); make clean)	
	(cd $(MAIN_DIR); make clean)	
	(cd $(WIFI_DIR); make clean)	
	(cd $(CONFIG_DIR); make clean)	
	(cd $(IRQ_DIR); make clean)	
	(cd $(PUBLIC_DIR); make clean)	
	(cd $(STMLIB_DIR); make clean)	
	rm -f *.o $(TARGET_BIN) $(TARGET_HEX) *.out *.list *.dis 
