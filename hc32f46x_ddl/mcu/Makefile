
# $@: 目标文件
# $^: 所有的依赖文件
# $<: 第一个依赖文件
# $?: 比目标文件还要新的依赖文件列表

# Makefile belongs to hc32f46x_ddl/mcu

CC = arm-none-eabi-gcc
AR = arm-none-eabi-ar
CFLAGS = -g -O3 -Wall

CUR_DIR = $(shell pwd)
SRCDIR  = ./common/
INCDIR  = ./common/
OBJDIR  = ../../out/objects/

#######################################################
# GLOBAL DEFINITIONS
CFLAGS += -D__xDEBUG -DHC32F46x -DUSE_DEVICE_DRIVER_LIB

# Includes
CFLAGS += -I${INCDIR}
CFLAGS += -I../../user/inc/
CFLAGS += -I./CMSIS/Core/Include/

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
