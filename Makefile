TARGET = stm32g030f6
MCU = cortex-m0plus

# Toolchain
PREFIX = arm-none-eabi-

CC      = $(PREFIX)gcc
AS      = $(PREFIX)gcc
LD      = $(PREFIX)gcc
OBJCOPY = $(PREFIX)objcopy
SIZE    = $(PREFIX)size

# Dossiers
SRC_DIR = .
BUILD_DIR = build
SDK_SRC = /lib/stm32/g0cube
HAL_DRV_SRC = $(SDK_SRC)/Drivers/STM32G0xx_HAL_Driver

# Fichiers sources
APP_C_SOURCES = $(wildcard $(SRC_DIR)/*.c)
HAL_SOURCES = \
	$(HAL_DRV_SRC)/Src/stm32g0xx_hal.c \
	$(HAL_DRV_SRC)/Src/stm32g0xx_hal_cortex.c \
	$(HAL_DRV_SRC)/Src/stm32g0xx_hal_dma.c \
	$(HAL_DRV_SRC)/Src/stm32g0xx_hal_dma_ex.c \
	$(HAL_DRV_SRC)/Src/stm32g0xx_hal_exti.c \
	$(HAL_DRV_SRC)/Src/stm32g0xx_hal_flash.c \
	$(HAL_DRV_SRC)/Src/stm32g0xx_hal_flash_ex.c \
	$(HAL_DRV_SRC)/Src/stm32g0xx_hal_gpio.c \
	$(HAL_DRV_SRC)/Src/stm32g0xx_hal_pwr.c \
	$(HAL_DRV_SRC)/Src/stm32g0xx_hal_pwr_ex.c \
	$(HAL_DRV_SRC)/Src/stm32g0xx_hal_rcc.c \
	$(HAL_DRV_SRC)/Src/stm32g0xx_hal_rcc_ex.c \
	$(HAL_DRV_SRC)/Src/stm32g0xx_hal_spi.c \
	$(HAL_DRV_SRC)/Src/stm32g0xx_hal_tim.c \
	$(HAL_DRV_SRC)/Src/stm32g0xx_hal_tim_ex.c \
	$(HAL_DRV_SRC)/Src/stm32g0xx_ll_rcc.c
C_SOURCES = $(HAL_SOURCES) $(APP_C_SOURCES)
ASM_SOURCES = $(wildcard $(SRC_DIR)/*.s)

OBJECTS = \
 	$(C_SOURCES:%.c=$(BUILD_DIR)/%.o) \
 	$(ASM_SOURCES:%.s=$(BUILD_DIR)/%.o)

# Flags CPU
CPU_FLAGS = -mcpu=$(MCU) -mthumb

# Définitions
DEFINES = \
	-DSTM32G030xx \
	-DUSE_HAL_DRIVER

# Includes
INCLUDES = \
	-I$(HAL_DRV_SRC)/Inc \
	-I$(HAL_DRV_SRC)/Inc/Legacy \
	-I$(SDK_SRC)/Drivers/CMSIS/Include \
	-I$(SDK_SRC)/Drivers/CMSIS/Device/ST/STM32G0xx/Include \
	-I$(SRC_DIR)

# Compilation C
CFLAGS = \
	$(CPU_FLAGS) \
	$(DEFINES) \
	$(INCLUDES) \
	-std=c11 \
	-Og \
	-g3 \
	-Wall \
	-ffunction-sections \
	-fdata-sections

# Assemblage
ASFLAGS = \
	$(CPU_FLAGS) \
	-x assembler-with-cpp \
	-g3

# Linker
LDSCRIPT = $(SRC_DIR)/STM32G030xx_FLASH.ld
LDFLAGS = \
	$(CPU_FLAGS) \
	-T $(LDSCRIPT) \
	-specs=nano.specs \
	-Wl,--gc-sections \
	-Wl,-Map=$(BUILD_DIR)/$(TARGET).map
# CubeMX ajoute ça à la phase link : -lc -lm -lnosys

# ============================================================
# Targets
# ============================================================

all: $(BUILD_DIR)/$(TARGET).elf \
	$(BUILD_DIR)/$(TARGET).hex \
	$(BUILD_DIR)/$(TARGET).bin

# Compilation C
$(BUILD_DIR)/%.o: %.c
	@mkdir -p $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $(BUILD_DIR)/$(notdir $@)

# Assemblage startup
$(BUILD_DIR)/%.o: %.s
	@mkdir -p $(BUILD_DIR)
	$(AS) $(ASFLAGS) -c $< -o $(BUILD_DIR)/$(notdir $@)

# Edition de liens
$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) $(LDSCRIPT)
	$(LD) $(foreach f,$(OBJECTS),$(BUILD_DIR)/$(notdir $(f))) $(LDFLAGS) -o $@
	$(SIZE) $@

# Conversion HEX
$(BUILD_DIR)/$(TARGET).hex: $(BUILD_DIR)/$(TARGET).elf
	$(OBJCOPY) -O ihex $< $@

# Conversion BIN
$(BUILD_DIR)/$(TARGET).bin: $(BUILD_DIR)/$(TARGET).elf
	$(OBJCOPY) -O binary $< $@

# Nettoyage
clean:
	rm -rf $(BUILD_DIR)

# Flash via J-Link (Segger) piloté par OpenOCD.
# OPENOCD_SCRIPTS pointe vers un dossier contenant interface/jlink.cfg (avec le
# numéro de série de la sonde) et target/stm32g0x.cfg (transport SWD forcé).
# Surchargeable sans modifier ce fichier :
#     make flash OPENOCD_SCRIPTS=/chemin/vers/mes/configs
OPENOCD = openocd
OPENOCD_SCRIPTS ?= ../OpenOCD
OPENOCD_FLAGS = \
	-s $(OPENOCD_SCRIPTS) \
	-f interface/jlink.cfg \
	-f target/stm32g0x.cfg

flash: $(BUILD_DIR)/$(TARGET).elf
	$(OPENOCD) $(OPENOCD_FLAGS) -c "program $< verify reset exit"

# Rebuild complet
rebuild: clean all

.PHONY: all clean flash rebuild