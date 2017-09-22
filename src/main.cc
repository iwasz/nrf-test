#include <Debug.h>
#include <ErrorHandler.h>
#include <Gpio.h>
#include <cstdbool>
#include <cstring>
#include <functional>
#include <stm32f0xx_hal.h>

namespace __gnu_cxx {
void __verbose_terminate_handler ()
{
        while (1)
                ;
}
}

static void SystemClock_Config (void);

/*****************************************************************************/

class Spi {
public:
        Spi (SPI_TypeDef *spi, GPIO_TypeDef *port, uint32_t pins, uint32_t alternate);
        ~Spi ();

        uint8_t transmit (uint8_t byte);

        void setCallback (std::function<void(uint8_t)> const &f) { fff = f; }

private:
        SPI_TypeDef *spi;
        SPI_HandleTypeDef accelSpiHandle;
        GPIO_InitTypeDef gpioInitStructure;
        std::function<void(uint8_t)> fff;
};

Spi::Spi (SPI_TypeDef *spi, GPIO_TypeDef *port, uint32_t pins, uint32_t alternate)
{
        __HAL_RCC_SPI2_CLK_ENABLE ();
        Gpio::clkEnable (port);
        gpioInitStructure.Pin = pins;
        gpioInitStructure.Mode = GPIO_MODE_AF_PP;
        gpioInitStructure.Pull = GPIO_NOPULL;
        gpioInitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
        gpioInitStructure.Alternate = alternate;
        HAL_GPIO_Init (port, &gpioInitStructure);

        SPI_HandleTypeDef accelSpiHandle;

        accelSpiHandle.Instance = spi;
        accelSpiHandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
        accelSpiHandle.Init.Direction = SPI_DIRECTION_2LINES;
        accelSpiHandle.Init.CLKPhase = SPI_PHASE_1EDGE;
        accelSpiHandle.Init.CLKPolarity = SPI_POLARITY_LOW;
        accelSpiHandle.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
        accelSpiHandle.Init.CRCPolynomial = 7;
        accelSpiHandle.Init.DataSize = SPI_DATASIZE_8BIT;
        accelSpiHandle.Init.FirstBit = SPI_FIRSTBIT_MSB;
        accelSpiHandle.Init.NSS = SPI_NSS_HARD_OUTPUT;
        accelSpiHandle.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
        accelSpiHandle.Init.TIMode = SPI_TIMODE_DISABLE;
        accelSpiHandle.Init.Mode = SPI_MODE_MASTER;

        HAL_SPI_Init (&accelSpiHandle);
}

uint8_t Spi::transmit (uint8_t byte) { return 0; }

/*****************************************************************************/

class Nrf24L0P {
public:
        Nrf24L0P (Spi *spi, Gpio *cePin, Gpio *irqPin) : spi (spi), cePin (cePin), irqPin (irqPin) {}
        ~Nrf24L0P () {}

private:
        Spi *spi;
        Gpio *cePin;
        Gpio *irqPin;
};

/*****************************************************************************/

int main (void)
{
        HAL_Init ();
        SystemClock_Config ();

        Debug *d = Debug::singleton ();
        d->init (115200);
        d->print ("nRF24L01+ test here\n");

        for (;;) {
        }

        //        Gpio ce1 (GPIOA, GPIO_PIN_2);
        //        ce1.set (false);

        Gpio irq1 (GPIOA, GPIO_PIN_2, GPIO_MODE_IT_FALLING);

        irq1.setOnToggle ([] {});

        //        Spi spi1 (SPI1, GPIOA, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, GPIO_AF0_SPI1);

        //        uint8_t buff[5];
        //        int i = 0;
        //        spi1.setCallback ([&i, &buff](uint8_t c) { buff[i++] = c; });

        //        Nrf24L0P nrfTx (&spi1, &ce1, &irq1);

        //        Nrf24L0P nrfRx (&spi1, &ce1, &irq1);

        while (1) {
                //                screen->refresh ();
                //                buzzer->run ();
                //                button->run ();
        }
}

/*****************************************************************************/

void SystemClock_Config (void)
{
        RCC_OscInitTypeDef RCC_OscInitStruct;
        RCC_ClkInitTypeDef RCC_ClkInitStruct;
        RCC_PeriphCLKInitTypeDef PeriphClkInit;

        /**Initializes the CPU, AHB and APB busses clocks
        */
        RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSI14 | RCC_OSCILLATORTYPE_HSI48;
        RCC_OscInitStruct.HSIState = RCC_HSI_ON;
        RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
        RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
        RCC_OscInitStruct.HSICalibrationValue = 16;
        RCC_OscInitStruct.HSI14CalibrationValue = 16;
        RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
        if (HAL_RCC_OscConfig (&RCC_OscInitStruct) != HAL_OK) {
                Error_Handler ();
        }

        /**Initializes the CPU, AHB and APB busses clocks
        */
        RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
        RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
        RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
        RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

        if (HAL_RCC_ClockConfig (&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
                Error_Handler ();
        }

        PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB | RCC_PERIPHCLK_USART1 | RCC_PERIPHCLK_I2C1;
        PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
        PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
        PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;

        if (HAL_RCCEx_PeriphCLKConfig (&PeriphClkInit) != HAL_OK) {
                Error_Handler ();
        }

        /**Configure the Systick interrupt time
        */
        HAL_SYSTICK_Config (HAL_RCC_GetHCLKFreq () / 1000);

        /**Configure the Systick
        */
        HAL_SYSTICK_CLKSourceConfig (SYSTICK_CLKSOURCE_HCLK);

        /* SysTick_IRQn interrupt configuration */
        HAL_NVIC_SetPriority (SysTick_IRQn, 0, 0);
}
