CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
SET (CMAKE_VERBOSE_MAKEFILE OFF)

PROJECT (nrf-test)
ENABLE_LANGUAGE (ASM-ATT)

INCLUDE_DIRECTORIES("src/")
INCLUDE_DIRECTORIES("../libmicro/src/")

# +-----------------+
# | My code         |
# +-----------------+
ADD_EXECUTABLE(${CMAKE_PROJECT_NAME}.elf
        "src/main.cc"
        "../libmicro/src/storage/ICircullarQueueStorage.h"
        "../libmicro/src/storage/IRandomAccessStorage.h"
        "../libmicro/src/storage/FlashEepromStorage.h"
        "../libmicro/src/ErrorHandler.h"
        "../libmicro/src/ErrorHandler.c"
        "../libmicro/src/Hal.h"
        "../libmicro/src/HalStm32F0.h"
        "../libmicro/src/HalStm32F0.cc"
        "../libmicro/src/Timer.cc"
        "../libmicro/src/Timer.h"
        "../libmicro/src/Debug.cc"
        "../libmicro/src/Debug.h"
        "../libmicro/src/Gpio.cc"
        "../libmicro/src/Gpio.h"
        "../libmicro/src/Usart.cc"
        "../libmicro/src/Usart.h"
        "../libmicro/src/Spi.cc"
        "../libmicro/src/Spi.h"
        "../libmicro/src/Console.cc"
        "../libmicro/src/Console.h"
        "../libmicro/src/rf/Nrf24L01P.cc"
        "../libmicro/src/rf/Nrf24L01P.h"
        "../libmicro/src/rf/SymaX5HWRxProtocol.cc"
        "../libmicro/src/rf/SymaX5HWRxProtocol.h"
        "../libmicro/src/collection/CharacterCircularQueue.h"
        "../libmicro/src/character/ICharacterInput.h"
        "../libmicro/src/character/ICharacterOutput.h"
        "../libmicro/src/character/ICharacterSink.h"
)

# +--------------+
# | STM Cube     |
# +--------------+
add_library ("stm" STATIC
        "src/stm32f4xx_it.c"
        "src/syscalls.c"
        "src/system_stm32f0xx.c"
        "src/stm32f0xx_hal_conf.h"
        "${CUBE_ROOT}/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_adc.c"
        "${CUBE_ROOT}/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_adc_ex.c"
        "${CUBE_ROOT}/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal.c"
        "${CUBE_ROOT}/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_can.c"
        "${CUBE_ROOT}/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_cec.c"
        "${CUBE_ROOT}/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_comp.c"
        "${CUBE_ROOT}/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_cortex.c"
        "${CUBE_ROOT}/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_crc.c"
        "${CUBE_ROOT}/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_crc_ex.c"
        "${CUBE_ROOT}/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_dac.c"
        "${CUBE_ROOT}/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_dac_ex.c"
        "${CUBE_ROOT}/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_dma.c"
        "${CUBE_ROOT}/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_flash.c"
        "${CUBE_ROOT}/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_flash_ex.c"
        "${CUBE_ROOT}/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_gpio.c"
        "${CUBE_ROOT}/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_i2c.c"
        "${CUBE_ROOT}/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_i2c_ex.c"
        "${CUBE_ROOT}/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_i2s.c"
        "${CUBE_ROOT}/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_irda.c"
        "${CUBE_ROOT}/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_iwdg.c"
        "${CUBE_ROOT}/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_msp_template.c"
        "${CUBE_ROOT}/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_pcd.c"
        "${CUBE_ROOT}/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_pcd_ex.c"
        "${CUBE_ROOT}/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_pwr.c"
        "${CUBE_ROOT}/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_pwr_ex.c"
        "${CUBE_ROOT}/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_rcc.c"
        "${CUBE_ROOT}/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_rcc_ex.c"
        "${CUBE_ROOT}/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_rtc.c"
        "${CUBE_ROOT}/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_rtc_ex.c"
        "${CUBE_ROOT}/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_smartcard.c"
        "${CUBE_ROOT}/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_smartcard_ex.c"
        "${CUBE_ROOT}/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_smbus.c"
        "${CUBE_ROOT}/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_spi.c"
        "${CUBE_ROOT}/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_spi_ex.c"
        "${CUBE_ROOT}/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_tim.c"
        "${CUBE_ROOT}/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_tim_ex.c"
        "${CUBE_ROOT}/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_tsc.c"
        "${CUBE_ROOT}/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_uart.c"
        "${CUBE_ROOT}/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_uart_ex.c"
        "${CUBE_ROOT}/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_usart.c"
        "${CUBE_ROOT}/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_wwdg.c"
        "${CUBE_ROOT}/Drivers/STM32F0xx_HAL_Driver/Inc/stm32f0xx_hal_uart_ex.h"
        "${CUBE_ROOT}/Drivers/STM32F0xx_HAL_Driver/Inc/stm32f0xx_hal_iwdg.h"
        "${CUBE_ROOT}/Drivers/STM32F0xx_HAL_Driver/Inc/stm32f0xx_hal_rcc.h"
        "${CUBE_ROOT}/Drivers/STM32F0xx_HAL_Driver/Inc/stm32f0xx_hal_def.h"
        "${CUBE_ROOT}/Drivers/STM32F0xx_HAL_Driver/Inc/stm32f0xx_hal_pwr_ex.h"
        "${CUBE_ROOT}/Drivers/STM32F0xx_HAL_Driver/Inc/stm32f0xx_hal_i2c.h"
        "${CUBE_ROOT}/Drivers/STM32F0xx_HAL_Driver/Inc/stm32f0xx_hal_uart.h"
        "${CUBE_ROOT}/Drivers/STM32F0xx_HAL_Driver/Inc/stm32f0xx_hal_smartcard.h"
        "${CUBE_ROOT}/Drivers/STM32F0xx_HAL_Driver/Inc/stm32f0xx_hal_i2c_ex.h"
        "${CUBE_ROOT}/Drivers/STM32F0xx_HAL_Driver/Inc/stm32f0xx_hal_pcd.h"
        "${CUBE_ROOT}/Drivers/STM32F0xx_HAL_Driver/Inc/stm32f0xx_hal_wwdg.h"
        "${CUBE_ROOT}/Drivers/STM32F0xx_HAL_Driver/Inc/stm32f0xx_hal_i2s.h"
        "${CUBE_ROOT}/Drivers/STM32F0xx_HAL_Driver/Inc/stm32f0xx_hal_tsc.h"
        "${CUBE_ROOT}/Drivers/STM32F0xx_HAL_Driver/Inc/stm32f0xx_hal_usart.h"
        "${CUBE_ROOT}/Drivers/STM32F0xx_HAL_Driver/Inc/stm32f0xx_hal_rtc.h"
        "${CUBE_ROOT}/Drivers/STM32F0xx_HAL_Driver/Inc/stm32f0xx_hal_adc_ex.h"
        "${CUBE_ROOT}/Drivers/STM32F0xx_HAL_Driver/Inc/stm32f0xx_hal_crc.h"
        "${CUBE_ROOT}/Drivers/STM32F0xx_HAL_Driver/Inc/stm32f0xx_hal_pwr.h"
        "${CUBE_ROOT}/Drivers/STM32F0xx_HAL_Driver/Inc/stm32f0xx_hal_dac_ex.h"
        "${CUBE_ROOT}/Drivers/STM32F0xx_HAL_Driver/Inc/stm32f0xx_hal_irda_ex.h"
        "${CUBE_ROOT}/Drivers/STM32F0xx_HAL_Driver/Inc/stm32f0xx_hal_flash.h"
        "${CUBE_ROOT}/Drivers/STM32F0xx_HAL_Driver/Inc/stm32f0xx_hal_tim_ex.h"
        "${CUBE_ROOT}/Drivers/STM32F0xx_HAL_Driver/Inc/stm32f0xx_hal_dma.h"
        "${CUBE_ROOT}/Drivers/STM32F0xx_HAL_Driver/Inc/stm32f0xx_hal_irda.h"
        "${CUBE_ROOT}/Drivers/STM32F0xx_HAL_Driver/Inc/stm32f0xx_hal_smbus.h"
        "${CUBE_ROOT}/Drivers/STM32F0xx_HAL_Driver/Inc/stm32f0xx_hal.h"
        "${CUBE_ROOT}/Drivers/STM32F0xx_HAL_Driver/Inc/stm32f0xx_hal_adc.h"
        "${CUBE_ROOT}/Drivers/STM32F0xx_HAL_Driver/Inc/stm32f0xx_hal_spi_ex.h"
        "${CUBE_ROOT}/Drivers/STM32F0xx_HAL_Driver/Inc/stm32f0xx_hal_tim.h"
        "${CUBE_ROOT}/Drivers/STM32F0xx_HAL_Driver/Inc/stm32f0xx_hal_can.h"
        "${CUBE_ROOT}/Drivers/STM32F0xx_HAL_Driver/Inc/stm32f0xx_hal_cec.h"
        "${CUBE_ROOT}/Drivers/STM32F0xx_HAL_Driver/Inc/stm32f0xx_hal_rcc_ex.h"
        "${CUBE_ROOT}/Drivers/STM32F0xx_HAL_Driver/Inc/stm32f0xx_hal_pcd_ex.h"
        "${CUBE_ROOT}/Drivers/STM32F0xx_HAL_Driver/Inc/stm32f0xx_hal_gpio.h"
        "${CUBE_ROOT}/Drivers/STM32F0xx_HAL_Driver/Inc/stm32f0xx_hal_flash_ex.h"
        "${CUBE_ROOT}/Drivers/STM32F0xx_HAL_Driver/Inc/stm32f0xx_hal_comp.h"
        "${CUBE_ROOT}/Drivers/STM32F0xx_HAL_Driver/Inc/stm32f0xx_hal_dac.h"
        "${CUBE_ROOT}/Drivers/STM32F0xx_HAL_Driver/Inc/stm32f0xx_hal_conf_template.h"
        "${CUBE_ROOT}/Drivers/STM32F0xx_HAL_Driver/Inc/stm32f0xx_hal_dma_ex.h"
        "${CUBE_ROOT}/Drivers/STM32F0xx_HAL_Driver/Inc/stm32f0xx_hal_usart_ex.h"
        "${CUBE_ROOT}/Drivers/STM32F0xx_HAL_Driver/Inc/stm32f0xx_hal_spi.h"
        "${CUBE_ROOT}/Drivers/STM32F0xx_HAL_Driver/Inc/stm32f0xx_hal_smartcard_ex.h"
        "${CUBE_ROOT}/Drivers/STM32F0xx_HAL_Driver/Inc/stm32f0xx_hal_gpio_ex.h"
        "${CUBE_ROOT}/Drivers/STM32F0xx_HAL_Driver/Inc/stm32f0xx_hal_crc_ex.h"
        "${CUBE_ROOT}/Drivers/STM32F0xx_HAL_Driver/Inc/stm32f0xx_hal_rtc_ex.h"
        "${CUBE_ROOT}/Drivers/STM32F0xx_HAL_Driver/Inc/stm32f0xx_hal_cortex.h"
        "${STARTUP_CODE}"
)

#set_target_properties (${CMAKE_PROJECT_NAME}.elf PROPERTIES COMPILE_FLAGS "-save-temps")
TARGET_LINK_LIBRARIES (${CMAKE_PROJECT_NAME}.elf stdc++)
TARGET_LINK_LIBRARIES (${CMAKE_PROJECT_NAME}.elf -Wl,--whole-archive stm -Wl,--no-whole-archive)

ADD_CUSTOM_TARGET(${CMAKE_PROJECT_NAME}.bin ALL DEPENDS ${CMAKE_PROJECT_NAME}.elf COMMAND ${CMAKE_OBJCOPY} -Obinary ${CMAKE_PROJECT_NAME}.elf ${CMAKE_PROJECT_NAME}.bin)

FIND_PROGRAM (ST_FLASH st-flash)
ADD_CUSTOM_TARGET("upload" DEPENDS ${CMAKE_PROJECT_NAME}.elf COMMAND ${ST_FLASH} write ${CMAKE_PROJECT_NAME}.bin 0x8000000)
