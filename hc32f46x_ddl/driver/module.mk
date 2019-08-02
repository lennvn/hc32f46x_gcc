
APP_SRCDIR = $(PWD)/hc32f46x_ddl/driver/src/
C_FILES += $(notdir $(wildcard ${APP_SRCDIR}*.c))
