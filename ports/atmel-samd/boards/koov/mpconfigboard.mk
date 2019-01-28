LD_FILE = boards/samd21x18-bootloader-koov.ld
USB_VID = 0x054C
USB_PID = 0x0BE6
USB_PRODUCT = "KOOV Core"
USB_MANUFACTURER = "Sony Corporation"

INTERNAL_FLASH_FILESYSTEM = 1
LONGINT_IMPL = NONE

CHIP_VARIANT = SAMD21G18A
CHIP_FAMILY = samd21

BOOTLOADER_SIZE = 0x4000
NO_GAMEPAD = 1
NO_TOUCHIO = 1
RESET_WITH_WDT = 1

FROZEN_MPY_DIRS += $(TOP)/frozen/Adafruit_CircuitPython_BusDevice
FROZEN_MPY_DIRS += $(TOP)/frozen/Adafruit_CircuitPython_MMA8451
FROZEN_MPY_DIRS += $(TOP)/ports/atmel-samd/boards/koov/frozon/koov
