
# $@: 目标文件
# $^: 所有的依赖文件
# $<: 第一个依赖文件
# $?: 比目标文件还要新的依赖文件列表

# Makefile belongs to User

CC = arm-none-eabi-gcc
CFLAGS = -g -O0 -Wall

CUR_DIR = $(shell pwd)
SRCDIR  = ./src/
INCDIR  = ./inc/
OBJDIR  = ../out/objects/
#######################################################
# GLOBAL DEFINITIONS
CFLAGS += -D__xDEBUG -DHC32F46x -DUSE_DEVICE_DRIVER_LIB

# Includes
CFLAGS += -I${INCDIR}
CFLAGS += -I../hc32f46x_ddl/mcu/common/
CFLAGS += -I../hc32f46x_ddl/mcu/CMSIS/Core/Include/
CFLAGS += -I../hc32f46x_ddl/driver/inc/

CFLAGS += -mthumb -mcpu=cortex-m4
CFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
CFLAGS += -mlittle-endian -mthumb-interwork

SRCS = $(notdir $(wildcard ${SRCDIR}*.c))
OBJS = $(addprefix ${OBJDIR}, $(SRCS:.c=.o))

.PHONY: all
all: $(OBJS)
$(OBJDIR)%.o: $(SRCDIR)%.c
	$(CC) $^ $(CFLAGS) -c -o $@

clean:
	rm -f $(OBJS)
