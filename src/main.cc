#include <Console.h>
#include <Debug.h>
#include <ErrorHandler.h>
#include <Gpio.h>
#include <Spi.h>
#include <Timer.h>
#include <Usart.h>
#include <cstdbool>
#include <cstring>
#include <functional>
#include <rf/Nrf24L01P.h>
#include <rf/SymaX5HWRxProtocol.h>
#include <stm32f0xx_hal.h>

/*****************************************************************************/

const uint8_t CX10_ADDRESS[] = { 0xcc, 0xcc, 0xcc, 0xcc, 0xcc };
#define PACKET_SIZE 5
#define CHANNEL 100

const uint8_t SYMA_ADDR[] = { 0xab, 0xac, 0xad, 0xae, 0xaf }; // bind addr
const uint8_t SYMA_CHANNELS[] = { 0x4b, 0x30, 0x40, 0x20 };   // bind chan
//#define RX_PACKET_SIZE 10
//#define RX_CHANNEL 8

/*****************************************************************************/

namespace __gnu_cxx {
void __verbose_terminate_handler ()
{
        while (1)
                ;
}
} // namespace __gnu_cxx

static void SystemClock_Config (void);

/*****************************************************************************/

class MyConsole : public Console {
public:
        MyConsole (Nrf24L01P *n) : Console (false, nullptr), nrf (n) {}
        virtual ~MyConsole () {}
        void onNewLine (const char *str, size_t len);

private:
        Nrf24L01P *nrf;
};

/*****************************************************************************/

void MyConsole::onNewLine (const char *str, size_t len)
{
        uint8_t buffer[5];
        buffer[0] = str[0]; // command

        if (str[1] != ' ') {
                Debug::singleton ()->print ("Sytax : c int\n");
                Debug::singleton ()->print (" c : one letter command\n");
                Debug::singleton ()->print (" int : 32 bit integer\n");
                return;
        }

        int i = atoi (str + 2);
        memcpy (buffer + 1, &i, sizeof (i));
        //        nrf->transmit (buffer, 5);
        nrf->setAckPayload (1, buffer, 5);
}

/*****************************************************************************/

int main (void)
{
        HAL_Init ();
        SystemClock_Config ();

        // C4 = TX, C5 = RX
        Gpio usartGpios (GPIOC, GPIO_PIN_4 | GPIO_PIN_5, GPIO_MODE_AF_OD, GPIO_PULLUP, GPIO_SPEED_FREQ_HIGH, GPIO_AF1_USART3);

        HAL_NVIC_SetPriority (USART3_4_IRQn, 6, 0);
        HAL_NVIC_EnableIRQ (USART3_4_IRQn);
        Usart uart (USART3, 115200);

        Debug debug (&uart);
        Debug::singleton () = &debug;
        Debug *d = Debug::singleton ();
        d->print ("nRF24L01+ test here\n");

        // Gpio button (GPIOA, GPIO_PIN_0, GPIO_MODE_IT_RISING);
        // HAL_NVIC_SetPriority (EXTI0_1_IRQn, 3, 0);
        // HAL_NVIC_EnableIRQ (EXTI0_1_IRQn);
        // button.setOnToggle ([d] { d->print ("#"); });

        /*---------------------------------------------------------------------------*/

        Gpio ceTx (GPIOC, GPIO_PIN_0);
        ceTx.set (false);

        Gpio irqTx (GPIOA, GPIO_PIN_1, GPIO_MODE_IT_FALLING, GPIO_PULLUP);
        // TODO This one is critical for bufferRx to be consistent
        HAL_NVIC_SetPriority (EXTI0_1_IRQn, 3, 0);
        HAL_NVIC_EnableIRQ (EXTI0_1_IRQn);

        Gpio spiTxGpiosNss (GPIOA, GPIO_PIN_4, GPIO_MODE_OUTPUT_OD, GPIO_PULLUP);
        Gpio spiTxGpiosSck (GPIOA, GPIO_PIN_5, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH, GPIO_AF0_SPI1);
        Gpio spiTxGpiosMosiMiso (GPIOB, GPIO_PIN_4 | GPIO_PIN_5, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH, GPIO_AF0_SPI1);
        Spi spiTx (SPI1);
        spiTx.setNssGpio (&spiTxGpiosNss);

#if 0
        // Dla ułatwienia : TO JEST ROBOT. ROBOT wysyła telemetrię.
        Nrf24L01P nrfTx (&spiTx, &ceTx, &irqTx, 10);
        nrfTx.setConfig (Nrf24L01P::MASK_TX_DS, true, Nrf24L01P::CRC_LEN_2);
        //        nrfTx.setTxAddress (CX10_ADDRESS, 5);
        //        nrfTx.setRxAddress (0, CX10_ADDRESS, 5);
        nrfTx.setAutoAck (Nrf24L01P::ENAA_P0);
        nrfTx.setEnableDataPipe (Nrf24L01P::ERX_P0);
        nrfTx.setAdressWidth (Nrf24L01P::WIDTH_5);
        nrfTx.setChannel (CHANNEL);
        nrfTx.setPayloadLength (0, PACKET_SIZE);
        nrfTx.setDataRate (Nrf24L01P::MBPS_1, Nrf24L01P::DBM_0);
        nrfTx.setEnableDynamicPayload (Nrf24L01P::DPL_P0);
        nrfTx.setFeature (Nrf24L01P::EN_DPL | Nrf24L01P::EN_ACK_PAY);
        HAL_Delay (100);
        nrfTx.powerUp (Nrf24L01P::TX);
        HAL_Delay (100);

        //        uint8_t bufRx2[33] = { 0x00 };

        class TxCallback : public Nrf24L01PCallback {
        public:
                virtual ~TxCallback () {}

                virtual void onRx (uint8_t *data, size_t len)
                {
                        Debug *d = Debug::singleton ();
                        d->print ("nrfTx received : ");
                        d->printArray (data, len);
                        d->print ("\n");
                }

                virtual void onTx () {}

                virtual void onMaxRt ()
                {
                        Debug *d = Debug::singleton ();
                        d->print ("nrfTx MAX_RT! Unable to send packet!");
                }
        } txCallback;

        nrfTx.setCallback (&txCallback);
#endif

        /*---------------------------------------------------------------------------*/

        //        static uint8_t bufTx[64] = { 1, 2, 3, 4, 5 };

#if 1
        //        uint8_t bufRx[SymaX5HWRxProtocol::RX_PACKET_SIZE + 1] = { 0x00 };

        /*---------------------------------------------------------------------------*/
        Gpio ceRx (GPIOB, GPIO_PIN_8);
        ceRx.set (false);

        Gpio irqRx (GPIOB, GPIO_PIN_7, GPIO_MODE_IT_FALLING, GPIO_PULLUP);
        HAL_NVIC_SetPriority (EXTI4_15_IRQn, 3, 0);
        HAL_NVIC_EnableIRQ (EXTI4_15_IRQn);

        Gpio spiRxGpiosNss (GPIOB, GPIO_PIN_9, GPIO_MODE_OUTPUT_OD, GPIO_PULLUP);
        Gpio spiRxGpiosSck (GPIOB, GPIO_PIN_10, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH, GPIO_AF5_SPI2);
        Gpio spiRxGpiosMisoMosi (GPIOB, GPIO_PIN_15 | GPIO_PIN_14, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH,
                                 GPIO_AF0_SPI2); // MOSI (15),MISO (14)
        Spi spiRx (SPI2);
        spiRx.setNssGpio (&spiRxGpiosNss);

        // A to jest na PC-cie. ODBIERA telemetrię i WYSYŁA KOMENDY w ACK.
        Nrf24L01P nrfRx (&spiRx, &ceRx, &irqRx, 10);
        nrfRx.setConfig (Nrf24L01P::MASK_TX_DS, true, Nrf24L01P::CRC_LEN_2);
        //        nrfRx.setTxAddress (CX10_ADDRESS, 5);
        //        nrfRx.setRxAddress (0, CX10_ADDRESS, 5);
        nrfRx.setAutoAck (Nrf24L01P::ENAA_P0 | Nrf24L01P::ENAA_P1);
        nrfRx.setEnableDataPipe (Nrf24L01P::ERX_P0 | Nrf24L01P::ERX_P1);
        nrfRx.setAdressWidth (Nrf24L01P::WIDTH_5);
        nrfRx.setChannel (CHANNEL);
        //        nrfRx.setPayloadLength (0, PACKET_SIZE);
        //        nrfRx.setPayloadLength (1, PACKET_SIZE);
        nrfRx.setDataRate (Nrf24L01P::MBPS_1, Nrf24L01P::DBM_0);
        nrfRx.setEnableDynamicPayload (Nrf24L01P::DPL_P0 | Nrf24L01P::DPL_P1);
        nrfRx.setFeature (Nrf24L01P::EN_DPL | Nrf24L01P::EN_ACK_PAY);
        HAL_Delay (100);
        nrfRx.powerUp (Nrf24L01P::RX);
        HAL_Delay (100);

        //        uint8_t ackPayload[] = { 7, 8, 9, 10, 11 };

        HAL_Delay (100);
        nrfRx.powerUp (Nrf24L01P::RX);
        HAL_Delay (100);

        //        SymaX5HWRxProtocol syma (&nrfRx);

        //        nrfRx.setOnData ([&syma, &nrfRx, &bufRx] {
        //                uint8_t *out = nrfRx.receive (bufRx, SymaX5HWRxProtocol::RX_PACKET_SIZE);
        //                syma.onPacket (out);
        //        });

        class RxCallback : public Nrf24L01PCallback {
        public:
                virtual ~RxCallback () {}

                virtual void onRx (uint8_t *data, size_t len)
                {
                        Debug *d = Debug::singleton ();

                        int index = *reinterpret_cast<int *> (data);

                        int pitchI = *reinterpret_cast<int *> (data + 4);
                        d->print (pitchI);
                        d->print (",");

                        int errorI = *reinterpret_cast<int *> (data + 8);
                        d->print (errorI);
                        d->print (",");

                        int integralI = *reinterpret_cast<int *> (data + 12);
                        d->print (integralI);
                        d->print (",");

                        int derivativeI = *reinterpret_cast<int *> (data + 16);
                        d->print (derivativeI);
                        d->print (",");

                        int outI = *reinterpret_cast<int *> (data + 20);
                        d->print (outI);
                        d->print (",");

                        if (index % 100 == 0) {
                                d->print ("\n");
                        }
                }

                virtual void onTx () {}

                virtual void onMaxRt ()
                {
                        Debug *d = Debug::singleton ();
                        d->print ("nrfRx MAX_RT");
                }
        } rxCallback;

        nrfRx.setCallback (&rxCallback);

#endif
        /*---------------------------------------------------------------------------*/

        MyConsole console (&nrfRx);
        console.setInput (&uart);
        console.setOutput (&uart);
        uart.startReceive ();

        /*---------------------------------------------------------------------------*/

        Timer tim;

        //        int cnt = 0;
        while (1) {

#if 0
                if (tim.isExpired ()) {
                        nrfTx.transmit (bufTx, PACKET_SIZE);
                        ++bufTx[0];

//                        d->print ("TX : ");
//                        for (int i = 0; i < PACKET_SIZE; ++i) {
//                                d->print (bufTx[i]);
//                                d->print (",");
//                        }
//                        d->print ("\n");

//                        uint8_t statusTx = nrfTx.getStatus ();
//                        d->print (statusTx);
//                        d->print ("\n");

//                        d->print ("TX : ");
//                        for (int i = 0; i < PACKET_SIZE; ++i) {
//                                d->print (bufTx[i]);
//                                d->print (",");
//                        }
//                        d->print ("\n");

//                        uint8_t *out = nrfRx.receive (bufRx, PACKET_SIZE);

//                        d->print ("RX : ");
//                        for (int i = 0; i < PACKET_SIZE; ++i) {
//                                d->print (out[i]);
//                                d->print (",");
//                        }
//                        d->print ("\n");

                        tim.start (500);
                }
#endif
                // nrfTx.poorMansScanner (200);
                console.run ();

                if (tim.isExpired ()) {
                        //                        nrfRx.setAckPayload (1, ackPayload, 5);
                        //                        ++(ackPayload[2]);

                        // Fake telemetry data
                        //                        nrfTx.transmit (bufTx, PACKET_SIZE);
                        //                        ++(bufTx[4]);
                        tim.start (500);
                }
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
