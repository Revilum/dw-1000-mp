# MicroPython DW1000 module makefile

DW1000_MOD_DIR := $(USERMOD_DIR)

# Add our source files to the respective variables
SRC_USERMOD_C += $(DW1000_MOD_DIR)/moddw1000.c
SRC_USERMOD_C += $(DW1000_MOD_DIR)/dw1000_hal.c

# Add the DW1000 driver source files
SRC_USERMOD_LIB_C += $(DW1000_MOD_DIR)/../../dw1000-driver/deca_device.c
SRC_USERMOD_LIB_C += $(DW1000_MOD_DIR)/../../dw1000-driver/deca_params_init.c
SRC_USERMOD_LIB_C += $(DW1000_MOD_DIR)/../../dw1000-driver/deca_range_tables.c

# Add include directories
CFLAGS_USERMOD += -I$(DW1000_MOD_DIR)
CFLAGS_USERMOD += -I$(DW1000_MOD_DIR)/../../dw1000-driver

# Disable some warnings for the DW1000 driver code
CFLAGS_USERMOD += -Wno-unused-function -Wno-unused-variable