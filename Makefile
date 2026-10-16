###############################################################################
# Project
###############################################################################

TARGET = firmware
BUILD_DIR = build

###############################################################################
# Toolchain
###############################################################################

CC       = arm-none-eabi-gcc
OBJCOPY  = arm-none-eabi-objcopy
OBJDUMP  = arm-none-eabi-objdump
SIZE     = arm-none-eabi-size

###############################################################################
# MCU
###############################################################################

MCU_FLAGS = -mcpu=cortex-m4 \
            -mthumb \
            -mfpu=fpv4-sp-d16 \
            -mfloat-abi=hard

###############################################################################
# Includes
###############################################################################

INCLUDES = \
    -I. \
    -I./include \
    -I./board \
    -I./drivers \
    -I./config \
    -I./middleware/RTOS/include \
    -I./middleware/RTOS/portable/GCC/ARM_CM4F \
    -I./middleware/logging \
    -I./middleware/logging/backend \
    -I./middleware/usb \
    -I./protocol \
    -I./middleware/tinyusb/src \
    -I./middleware/tinyusb/vendor/cmsis_core/Include \
    -I./middleware/tinyusb/vendor/cmsis_device_f4 \


###############################################################################
# Compiler
###############################################################################

CFLAGS = $(MCU_FLAGS) \
          -std=gnu11 \
          -Wall \
          -Wextra \
          -O0 \
          -g3 \
          -ffunction-sections \
          -fdata-sections \
          -MMD \
          -MP \
          -DSTM32F411xE \
          -DCFG_TUSB_MCU=OPT_MCU_STM32F4 \
          $(INCLUDES)
###############################################################################
# Linker
###############################################################################

LDFLAGS = -T ./linker/stm32f411.ld \
           -nostartfiles \
           --specs=nano.specs \
           -Wl,--gc-sections \
           -Wl,-Map=$(BUILD_DIR)/$(TARGET).map \
           -Wl,--print-memory-usage

LDLIBS = -lm

###############################################################################
# Sources
###############################################################################

SRCS = \
    ./app/main.c \
    ./app/app_tasks.c \
    ./middleware/cli/console.c \
    ./drivers/led.c \
    ./drivers/uart.c \
    ./drivers/i2c.c \
    ./middleware/mctp/mctp.c \
    ./middleware/mctp/mctp_smbus.c \
    ./middleware/adapter/adapter_service.c \
    ./middleware/usb/usb_device.c \
    ./middleware/usb/usb_descriptors.c \
    ./middleware/usb/usb_protocol.c \
    ./middleware/tinyusb/src/tusb.c \
    ./middleware/tinyusb/src/common/tusb_fifo.c \
    ./middleware/tinyusb/src/device/usbd.c \
    ./middleware/tinyusb/src/class/vendor/vendor_device.c \
    ./middleware/tinyusb/src/portable/synopsys/dwc2/dcd_dwc2.c \
    ./middleware/tinyusb/src/portable/synopsys/dwc2/dwc2_common.c \
    ./startup/startup.c \
    ./startup/system_stm32f411.c \
    ./board/board.c \
    ./board/system.c \
    ./middleware/RTOS/tasks.c \
    ./middleware/RTOS/list.c \
    ./middleware/RTOS/queue.c \
    ./middleware/RTOS/timers.c \
    ./middleware/RTOS/event_groups.c \
    ./middleware/RTOS/stream_buffer.c \
    ./middleware/RTOS/heap_4.c \
    ./middleware/RTOS/portable/GCC/ARM_CM4F/port.c \
    ./middleware/logging/backend/logger_backend_uart.c \
    ./middleware/logging/logger.c

###############################################################################
# Objects
###############################################################################

OBJS = $(patsubst %.c,$(BUILD_DIR)/%.o,$(SRCS))

###############################################################################
# Output Files
###############################################################################

ELF = $(BUILD_DIR)/$(TARGET).elf
BIN = $(BUILD_DIR)/$(TARGET).bin
HEX = $(BUILD_DIR)/$(TARGET).hex
MAP = $(BUILD_DIR)/$(TARGET).map
ASM = $(BUILD_DIR)/$(TARGET).asm
LST = $(BUILD_DIR)/$(TARGET).lst

###############################################################################
# Build Rules
###############################################################################

.PHONY: all clean flash disasm host

all: $(ELF) $(BIN) $(HEX) $(ASM) $(LST)

host:
	$(MAKE) -C host/usb_client

$(ELF): $(OBJS)
	$(CC) $(CFLAGS) $(OBJS) $(LDFLAGS) $(LDLIBS) -o $@
	$(SIZE) $@

$(BIN): $(ELF)
	$(OBJCOPY) -O binary $< $@

$(ASM): $(ELF)
	$(OBJDUMP) -dS -C $< > $@

$(HEX): $(ELF)
	$(OBJCOPY) -O ihex $< $@

$(LST): $(ELF)
	$(OBJDUMP) -h -S $< > $@

$(BUILD_DIR)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

###############################################################################
# Flash
###############################################################################

flash: $(BIN)
	st-flash write $(BIN) 0x08000000

###############################################################################
# Clean
###############################################################################

clean:
	@echo "Cleaning..."
	@rm -rf $(BUILD_DIR)
	@find . -type d -name "__pycache__" -exec rm -rf {} +
	@$(MAKE) -C host/usb_client clean
	@echo "Done."


-include $(OBJS:.o=.d)
