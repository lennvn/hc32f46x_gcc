
# $@: 目标文件
# $^: 所有的依赖文件
# $<: 第一个依赖文件
# $?: 比目标文件还要新的依赖文件列表

# The top layer makefile

######################################################
# Project name and target MCU.
PROJ_NAME    = HC32F46x_GCC
TARGET_MCU   = HC32F46x

######################################################
# Output directory./obj, ./exe.
OBJ_DIR		 = obj
EXE_DIR      = exe

######################################################
# Output files.
ELF_FILE     = $(PROJ_NAME).elf
BIN_FILE     = $(PROJ_NAME).bin
HEX_FILE     = $(PROJ_NAME).hex
LST_FILE     = $(PROJ_NAME).lst
DIS_FILE     = $(PROJ_NAME).dis
MAP_FILE     = $(PROJ_NAME).map
SYMBOL_FILE  = $(PROJ_NAME).symbol_table
SECTION_FILE = $(PROJ_NAME).section_talbe

######################################################
# Paths of source files and head files.
SRC_DIR += ./hc32f46x_ddl/mcu/common
SRC_DIR += ./hc32f46x_ddl/driver/src
SRC_DIR += ./user/src

INC_DIR += ./hc32f46x_ddl/mcu/common
INC_DIR += ./hc32f46x_ddl/mcu/CMSIS/Core/Include
INC_DIR += ./hc32f46x_ddl/driver/inc
INC_DIR += ./user/inc

BOOT_DIR += ./startup/gcc

LINK_DIR += ./ldscripts

######################################################
#
SRCS += $(foreach dir, $(SRC_DIR), $(wildcard $(dir)/*.c))
OBJS += $(patsubst %.c, $(OBJ_DIR)/%.o, $(notdir $(SRCS)))

BOOT_SRC += $(BOOT_DIR)/$(TARGET_MCU).S
BOOT_OBJ += $(patsubst %.S, $(OBJ_DIR)/%.o, $(notdir $(BOOT_SRC)))

TARGET_ELF = $(EXE_DIR)/$(ELF_FILE)
TARGET_BIN = $(EXE_DIR)/$(BIN_FILE)
TARGET_HEX = $(EXE_DIR)/$(HEX_FILE)
TARGET_LST = $(EXE_DIR)/$(LST_FILE)
TARGET_DIS = $(EXE_DIR)/$(DIS_FILE)
TARGET_MAP = $(EXE_DIR)/$(MAP_FILE)
TARGET_SYMBOL  = $(EXE_DIR)/$(SYMBOL_FILE)
TARGET_SECTION = $(EXE_DIR)/$(SECTION_FILE)

LD_SRC += $(LINK_DIR)/$(TARGET_MCU).ld

######################################################
#
INCLUDES = $(foreach i, $(INC_DIR), -I $(i))

VPATH += $(SRC_DIR)
VPATH += $(INC_DIR)
VPATH += $(OBJ_DIR)
VPATH += $(EXE_DIR)

######################################################
# Tool chain.
CC = arm-none-eabi-gcc
AS = arm-none-eabi-as
AR = arm-none-eabi-ar
LD = arm-none-eabi-ld
OBJCOPY = arm-none-eabi-objcopy
OBJDUMP = arm-none-eabi-objdump
RDELF   = arm-none-eabi-readelf
SIZE    = arm-none-eabi-size

######################################################
#
CFLAGS :=
CFLAGS += -g -O1 -Wall
CFLAGS += -Wno-main
CFLAGS += -mthumb -mthumb-interwork -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16
CFLAGS += -mlittle-endian

######################################################
# GLOBAL DEFINITIONS
CFLAGS += -D__xDEBUG -DHC32F46x -DUSE_DEVICE_DRIVER_LIB
CFLAGS += $(INCLUDES)

######################################################
# ASFLAGS
AFLAGS  = -mcpu=cortex-m4 -mthumb -mthumb-interwork

######################################################
#
LDFLAGS :=
LDFLAGS += -nostartfiles --gc-sections
#LDFLAGS += -L$(LIBPATH)/v7e-m/fpv4-sp/hard -lc
#LDFLAGS += -L$(LIBPATH)/v7e-m/fpv4-sp/hard -lg
#LDFLAGS += -L$(LIBPATH)/arm-none-eabi/6.3.1/thumb/v7e-m/fpv4-sp/hard -lgcc
LDFLAGS += -T $(LD_SRC)
LDFLAGS += -Map=$(TARGET_MAP)

######################################################
#
ODFLAGS   = -D -S -r
SEC_FLAGS = -S
SYB_FLAGS = -s

######################################################
#
DIV_LINE = -----------------------------------------------------
######################################################

all: start complie link dn
start:
	@echo "Making directories for object files and executable files..."
	@mkdir -p $(OBJ_DIR)
	@mkdir -p $(EXE_DIR)
	@echo "Making directories done."

complie: $(OBJS) $(BOOT_OBJ)
$(OBJ_DIR)/%.o : %.c
	@echo $(DIV_LINE)
	@echo Compiling... $<
	$(CC) -c $< $(CFLAGS) -o $@
$(BOOT_OBJ) : $(BOOT_SRC)
	@echo $(DEV)
	@echo Compiling... $<
	$(AS) $^ $(ASFLAGS) -o $@

link: $(TARGET_ELF) $(TARGET_BIN) $(TARGET_HEX) $(TARGET_DIS) $(TARGET_SYMBOL) $(TARGET_SECTION)
$(TARGET_ELF) : $(BOOT_OBJ) $(OBJS)
	@echo $(DIV_LINE)
	@echo "Linking..."
	$(LD) $^ $(LDFLAGS) -o $@
$(TARGET_BIN) : $(TARGET_ELF)
	@echo $(DIV_LINE)
	@echo Outputing bin file...
	$(OBJCOPY) -O binary $< $@
$(TARGET_HEX) : $(TARGET_ELF)
	@echo $(DIV_LINE)
	@echo Outputing hex file...
	$(OBJCOPY) -O ihex $< $@
$(TARGET_DIS) : $(TARGET_ELF)
	@echo $(DIV_LINE)
	@echo Outputing list file...
	$(OBJDUMP) $(ODFLAGS) $(TARGET_ELF) > $(TARGET_DIS)
$(TARGET_SYMBOL) : $(TARGET_ELF)
	@echo $(DIV_LINE)
	@echo Outputing symbol table file...
	$(RDELF) $(SYB_FLAGS) $(TARGET_ELF) > $(TARGET_SYMBOL)
$(TARGET_SECTION) : $(TARGET_ELF)
	@echo $(DIV_LINE)
	@echo Outputing section table file...
	$(RDELF) $(SEC_FLAGS) $(TARGET_ELF) > $(TARGET_SECTION)
	@echo $(DIV_LINE)
	@echo Size of $(ELF_FILE) file...
	@$(SIZE) $(TARGET_ELF)

dn:
	@echo $(DIV_LINE)
	@echo "Downloading..."
	JLinkExe -autoconnect 1 -device $(TARGET_MCU) -if swd -speed 100000 -ExitOnError -CommandFile download.jlink
	@echo "Success :)"

.PHONY: cl re
cl:
	@echo "Deleting object files and executable files..."
	@rm -rf $(OBJ_DIR)/*
	@rm -rf $(EXE_DIR)/*
	@echo "Deleting done."

re: cl all

######################################################
