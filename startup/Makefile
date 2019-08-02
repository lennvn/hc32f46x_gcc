
# $@: 目标文件
# $^: 所有的依赖文件
# $<: 第一个依赖文件
# $?: 比目标文件还要新的依赖文件列表

# Makefile belongs to startup

AS = arm-none-eabi-as
#CFLAGS = -g -O0 -Wall

CUR_DIR = $(shell pwd)
SRCDIR  = ./gcc/
OBJDIR  = ../out/objects/

#######################################################
#CFLAGS += -mthumb -mcpu=cortex-m4
#CFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
#CFLAGS += -mlittle-endian  -mthumb-interwork

ASFLAGS =

SRCS = $(notdir $(wildcard ${SRCDIR}*.S))
OBJS = $(addprefix ${OBJDIR}, $(SRCS:.S=.o))

.PHONY: all
all: $(OBJS)
$(OBJDIR)%.o: $(SRCDIR)%.S
	$(AS) $^ $(ASFLAGS) -o $@

clean:
	rm -f $(OBJS)
