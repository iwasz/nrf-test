#include <Debug.h>
#include <ErrorHandler.h>
#include <Gpio.h>
#include <Spi.h>
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

        Gpio button (GPIOA, GPIO_PIN_0, GPIO_MODE_IT_RISING);
        HAL_NVIC_SetPriority (EXTI0_1_IRQn, 3, 0);
        HAL_NVIC_EnableIRQ (EXTI0_1_IRQn);
        button.setOnToggle ([d] { d->print ("#"); });

        Gpio ce1 (GPIOA, GPIO_PIN_2);
        ce1.set (false);

        Gpio irq1 (GPIOA, GPIO_PIN_3, GPIO_MODE_IT_FALLING);
        HAL_NVIC_SetPriority (EXTI2_3_IRQn, 3, 0);
        HAL_NVIC_EnableIRQ (EXTI2_3_IRQn);
        irq1.setOnToggle ([d] { d->print ("IRQ\n"); });

        Gpio spiGpios (GPIOA, GPIO_PIN_4 | GPIO_PIN_5, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH, GPIO_AF0_SPI1);
        Gpio spiGpios2 (GPIOB, GPIO_PIN_4 | GPIO_PIN_5, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH, GPIO_AF0_SPI1);
        Spi spi1 (SPI1);

        //        uint16_t ret = spi1.transmit (0xff);
        //        d->print (ret);
        //        d->print ("\n");
        //        ret = spi1.transmit (0xff);
        //        d->print (ret);
        //        d->print ("\n");
        //        ret = spi1.transmit (0xff);
        //        d->print (ret);
        //        d->print ("\n");
        //        ret = spi1.transmit (0xff);

        enum Nrf24L01PCommands { NOP = 0xff };
        uint8_t bufferTx[16];
        uint8_t bufferRx[16];
        bufferTx[0] = NOP;
        spi1.transmit (bufferTx, bufferRx, 1);

        d->print ("Received [");
        d->print (bufferRx[0]);
        d->print ("]\n");

        Gpio spi2Gpios (GPIOB, GPIO_PIN_9 | GPIO_PIN_10, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH, GPIO_AF5_SPI2);
        // MOSI (15), MISO (14)
        Gpio spi2Gpios3 (GPIOB, GPIO_PIN_15 | GPIO_PIN_14, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH, GPIO_AF0_SPI2);
        Spi spi2 (SPI2);
        spi2.transmit (bufferTx, bufferRx, 1);

        d->print ("Received2 [");
        d->print (bufferRx[0]);
        d->print ("]\n");

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
