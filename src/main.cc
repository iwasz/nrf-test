#include <Debug.h>
#include <ErrorHandler.h>
#include <Gpio.h>
#include <Nrf24L01P.h>
#include <Spi.h>
#include <Timer.h>
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
} // namespace __gnu_cxx

static void SystemClock_Config (void);

/*****************************************************************************/

int main (void)
{
        HAL_Init ();
        SystemClock_Config ();

        // PB10 == TX, PB11 == RX
        Gpio debugGpios (GPIOC, GPIO_PIN_4, GPIO_MODE_AF_OD, GPIO_PULLUP, GPIO_SPEED_FREQ_HIGH, GPIO_AF1_USART3);
        Debug *d = Debug::singleton ();
        d->init (115200);
        d->print ("nRF24L01+ test here\n");

        Gpio button (GPIOA, GPIO_PIN_0, GPIO_MODE_IT_RISING);
        HAL_NVIC_SetPriority (EXTI0_1_IRQn, 3, 0);
        HAL_NVIC_EnableIRQ (EXTI0_1_IRQn);
        button.setOnToggle ([d] { d->print ("#"); });

        /*---------------------------------------------------------------------------*/

        const uint8_t CX10_ADDRESS[] = { 0xcc, 0xcc, 0xcc, 0xcc, 0xcc };
#define CX10_PACKET_SIZE 15
// CX10 blue board packets have 19-byte payload
#define CX10A_PACKET_SIZE 19
#define Q282_PACKET_SIZE 21
#define PACKET_SIZE CX10A_PACKET_SIZE
#define CHANNEL 0x02

        Gpio ceTx (GPIOA, GPIO_PIN_2);
        ceTx.set (false);

        //        Gpio irqTx (GPIOA, GPIO_PIN_3, GPIO_MODE_IT_FALLING, GPIO_PULLUP);
        //        HAL_NVIC_SetPriority (EXTI2_3_IRQn, 3, 0);
        //        HAL_NVIC_EnableIRQ (EXTI2_3_IRQn);
        //        irqTx.setOnToggle ([d] { d->print ("IRQ TX\n"); });

        Gpio spiTxGpiosNss (GPIOA, GPIO_PIN_4, GPIO_MODE_OUTPUT_OD, GPIO_PULLUP);
        Gpio spiTxGpiosSck (GPIOA, GPIO_PIN_5, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH, GPIO_AF0_SPI1);
        Gpio spiTxGpiosMosiMiso (GPIOB, GPIO_PIN_4 | GPIO_PIN_5, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH, GPIO_AF0_SPI1);
        Spi spiTx (SPI1);
        spiTx.setNssPin (&spiTxGpiosNss);

#if 1
        Nrf24L01P nrfTx (&spiTx, &ceTx, nullptr);
//        nrfTx.writeRegister (Nrf24L01P::STATUS, 0x70); // Clear IRQS
        nrfTx.setConfig (Nrf24L01P::MASK_NO_IRQ, true, Nrf24L01P::CRC_LEN_2);
        nrfTx.setTxAddress (CX10_ADDRESS, 5);
        nrfTx.setRxAddress (0, CX10_ADDRESS, 5);
//        nrfTx.flushTx ();
        nrfTx.setAutoAck (0x00);
        nrfTx.setEnableDataPipe (Nrf24L01P::ERX_P0);
        nrfTx.setAdressWidth (Nrf24L01P::WIDTH_5);
        nrfTx.setChannel (CHANNEL);
        nrfTx.setAutoRetransmit (Nrf24L01P::WAIT_250, Nrf24L01P::RETRANSMIT_0);
        nrfTx.setPayloadLength (0, PACKET_SIZE);
        nrfTx.setDataRate (Nrf24L01P::MBPS_1, Nrf24L01P::DBM_0);
        nrfTx.setEnableDynamicPayload (0x00);
        nrfTx.setFeature (0x00);
        nrfTx.powerUp (Nrf24L01P::TX);
#endif

        /*---------------------------------------------------------------------------*/

        static uint8_t bufTx[PACKET_SIZE]
                = { 0xaa, 0x2b, 0x59, 0x26, 0xb9, 0xff, 0xff, 0xff, 0xff, 0x01, 0x05, 0xdc, 0x05, 0xe8, 0x03, 0xdc, 0x05, 0x00, 0x00 };

        static uint8_t bufRx[PACKET_SIZE] = {
                0x00,
        };

        /*---------------------------------------------------------------------------*/

        Gpio ceRx (GPIOB, GPIO_PIN_8);
        ceRx.set (false);

        Gpio irqRx (GPIOB, GPIO_PIN_7, GPIO_MODE_IT_FALLING, GPIO_PULLUP);
        HAL_NVIC_SetPriority (EXTI4_15_IRQn, 3, 0);
        HAL_NVIC_EnableIRQ (EXTI4_15_IRQn);

        Gpio spiRxGpiosNss (GPIOB, GPIO_PIN_9, GPIO_MODE_OUTPUT_OD, GPIO_PULLUP);
        Gpio spiRxGpiosSck (GPIOB, GPIO_PIN_10, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH, GPIO_AF5_SPI2);
        Gpio spiRxGpiosMisoMosi (GPIOB, GPIO_PIN_15 | GPIO_PIN_14, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH, GPIO_AF0_SPI2); // MOSI (15),MISO (14)
        Spi spiRx (SPI2);
        spiRx.setNssPin (&spiRxGpiosNss);

#if 1
        Nrf24L01P nrfRx (&spiRx, &ceRx, &irqRx);
        nrfRx.setConfig (Nrf24L01P::MASK_NO_IRQ, true, Nrf24L01P::CRC_LEN_2);
        nrfRx.setTxAddress (CX10_ADDRESS, 5);
        nrfRx.setRxAddress (0, CX10_ADDRESS, 5);
        nrfRx.setAutoAck (0x00);
        nrfRx.setEnableDataPipe (Nrf24L01P::ERX_P0);
        nrfRx.setAdressWidth (Nrf24L01P::WIDTH_5);
        nrfRx.setChannel (CHANNEL);
        nrfRx.setAutoRetransmit (Nrf24L01P::WAIT_250, Nrf24L01P::RETRANSMIT_0);
        nrfRx.setPayloadLength (0, PACKET_SIZE);
        nrfRx.setDataRate (Nrf24L01P::MBPS_1, Nrf24L01P::DBM_0);
        nrfRx.setEnableDynamicPayload (0x00);
        nrfRx.setFeature (0x00);
        nrfRx.powerUp (Nrf24L01P::RX);
#endif

        //        nrfRx.setOnData ([d, &nrfRx, &bufRx] {
        //                d->print ("RX : ");
        //                for (int i = 0; i < PACKET_SIZE; ++i) {
        //                        d->print (bufRx[i]);
        //                }
        //                d->print ("\n");
        //        });

        /*****************************************************************************/
        Timer tim;

        while (1) {
#if 1
                nrfTx.writeRegister (Nrf24L01P::CONFIG, 0x0e);
                nrfTx.writeRegister (Nrf24L01P::RF_CH, 0x02);
                nrfTx.writeRegister (Nrf24L01P::STATUS, 0x70);
                nrfTx.flushTx ();

                nrfTx.transmit (bufTx, PACKET_SIZE);
                ++bufTx[1];

//                d->print ("TX : ");
//                for (int i = 0; i < PACKET_SIZE; ++i) {
//                        d->print (bufTx[i]);
//                        d->print (",");
//                }
//                d->print ("\n");
#endif

                HAL_Delay (500);

#if 1
                if (tim.isExpired ()) {
#if 1
                        // TODO czemu to musi tu być!? Czemu IRQ się nie zgłąsza samo!!!
                        uint8_t *out = nrfRx.receive (bufRx, PACKET_SIZE);
                        d->print ("RX : ");
                        for (int i = 0; i < PACKET_SIZE; ++i) {
                                d->print (out[i]);
                                d->print (",");
                        }
                        d->print ("\n");
#else
                        d->print (".\n");
#endif
                        tim.start (1000);
                }
#endif
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
