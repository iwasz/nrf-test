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
        enum Register {
                CONFIG = 0x00,
                EN_AA = 0x01,
                EN_RXADDR = 0x02,
                SETUP_AW = 0x03,
                SETUP_RETR = 0x04,
                RF_CH = 0x05,
                RF_SETUP = 0x06,
                STATUS = 0x07,
                OBSERVE_TX = 0x08,
                RPD = 0x09,
                RX_ADDR_P0 = 0x0A,
                RX_ADDR_P1 = 0x0B,
                RX_ADDR_P2 = 0x0C,
                RX_ADDR_P3 = 0x0D,
                RX_ADDR_P4 = 0x0E,
                RX_ADDR_P5 = 0x0F,
                TX_ADDR = 0x10,
                RX_PW_P0 = 0x11,
                RX_PW_P1 = 0x12,
                RX_PW_P2 = 0x13,
                RX_PW_P3 = 0x14,
                RX_PW_P4 = 0x15,
                RX_PW_P5 = 0x16,
                FIFO_STATUS = 0x17,
                DYNPD = 0x1C,
                FEATURE = 0x1D
        };

        enum Nrf24L01PCommands {
                R_REGISTER = 0x00, // last 4 bits will indicate reg. address
                W_REGISTER = 0x20, // last 4 bits will indicate reg. address
                REGISTER_MASK = 0x1F,
                R_RX_PAYLOAD = 0x61,
                W_TX_PAYLOAD = 0xA0,
                FLUSH_TX = 0xE1,
                FLUSH_RX = 0xE2,
                REUSE_TX_PL = 0xE3,
                ACTIVATE = 0x50,
                R_RX_PL_WID = 0x60,
                NOP = 0xFF
        };

        enum ConfigBit { EN_CRC = 3, CRCO = 2, PWR_UP = 1, PRIM_RX = 0 };
        enum IrqSource { RX_DR = 1 << 6, TX_DS = 1 << 5, MAX_RT = 1 << 4, ALL_IRQ = RX_DR | TX_DS | MAX_RT };
        enum CrcLength { CRC_LEN_1 = 0, CRC_LEN_2 = 1 << EN_CRC };
        enum Mode { TX, RX };
        enum EnAa { ENAA_P5 = 1 << 5, ENAA_P4 = 1 << 4, ENAA_P3 = 1 << 3, ENAA_P2 = 1 << 2, ENAA_P1 = 1 << 1, ENAA_P0 = 1 << 0 };
        enum EnRxAddr { ERX_P5 = 1 << 5, ERX_P4 = 1 << 4, ERX_P3 = 1 << 3, ERX_P2 = 1 << 2, ERX_P1 = 1 << 1, ERX_P0 = 1 << 0 };
        enum AddressWidth { WIDTH_3 = 0x01, WIDH_4 = 0x02, WIDTH_5 = 0x03 };

        enum RetransmitDelay {
                WAIT_250,
                WAIT_500,
                WAIT_750,
                WAIT_1000,
                WAIT_1250,
                WAIT_1500,
                WAIT_1750,
                WAIT_2000,
                WAIT_2250,
                WAIT_2500,
                WAIT_2750,
                WAIT_3000,
                WAIT_3250,
                WAIT_3500,
                WAIT_3750,
                WAIT_4000
        };

        enum RetransmitCount {
                RETRANSMIT_0,
                RETRANSMIT_1,
                RETRANSMIT_2,
                RETRANSMIT_3,
                RETRANSMIT_4,
                RETRANSMIT_5,
                RETRANSMIT_6,
                RETRANSMIT_7,
                RETRANSMIT_8,
                RETRANSMIT_9,
                RETRANSMIT_10,
                RETRANSMIT_11,
                RETRANSMIT_12,
                RETRANSMIT_13,
                RETRANSMIT_14,
                RETRANSMIT_15
        };

        enum DataRate { MBPS_1 = 0 << 3, MBPS_2 = 1 << 3, KBPS_250 = 2 << 3 };
        enum Gain { DBM_18 = 0 << 1, DBM_12 = 1 << 1, DBM_6 = 2 << 1, DBM_0 = 3 << 1 };
        enum DynamicPayloadLength { DPL_P5 = 1 << 5, DPL_p4 = 1 << 4, DPL_P3 = 1 << 3, DPL_P2 = 1 << 2, DPL_P1 = 1 << 1, DPL_P0 = 1 << 0 };

        /*---------------------------------------------------------------------------*/

        Nrf24L0P (Spi *spi, Gpio *cePin, Gpio *irqPin);

        void setConfig (uint8_t irqSource, bool crcEnable, CrcLength crcLength)
        {
                configRegisterCopy &= (1 << PWR_UP | 1 << PRIM_RX);
                configRegisterCopy |= irqSource | (uint8_t (crcEnable) << 3) | crcLength;
                writeRegister (CONFIG, configRegisterCopy);
        }

        void powerUp (Mode mode);
        void setAutoAck (uint8_t enAa) { writeRegister (EN_AA, enAa); }
        void setEnableDataPipe (uint8_t pipes) { writeRegister (EN_RXADDR, pipes); }
        void setAdressWidth (AddressWidth aw) { writeRegister (SETUP_AW, aw); }
        void setAutoRetransmit (RetransmitDelay d, RetransmitCount c) { writeRegister (SETUP_RETR, d | c); }
        void setChannel (uint8_t channel) { writeRegister (RF_CH, channel); }
        void setDataRate (DataRate dr, Gain g) { writeRegister (RF_SETUP, dr | g); }
        // Status
        void getObserve (uint8_t *lostPackets, uint8_t *retransmittedPackets) const
        {
                *lostPackets = readRegister (OBSERVE_TX);
                *retransmittedPackets = *lostPackets & 0x0f;
                *lostPackets >>= 4;
        }

        bool getReceivedPowerDetector () const { return readRegister (RPD) & 1; }
        void setRxAddress (uint8_t dataPipeNo, uint8_t address[5], uint8_t addressLen) { writeRegister (RX_ADDR_P0 + dataPipeNo, address, addressLen); }
        void setTxAddress (uint8_t address[5], uint8_t addressLen) { writeRegister (TX_ADDR, address, addressLen); }
        void setPayloadLength (uint8_t dataPipeNo, uint8_t len) { writeRegister (RX_PW_P0 + dataPipeNo, len); }

        /// Mask with something
        uint8_t getFifoStatus () const { return readRegister (FIFO_STATUS); }

        /// DynamicPayloadLength
        void setEnableDynamicPayload (uint8_t dpl) { writeRegister (DYNPD, dpl); }

        /// Feature
        void setFeature (uint8_t f) { writeRegister (FEATURE, f); }

        void transmit (uint8_t *data, size_t len);
        void receive (uint8_t *data, size_t len);

private:
        void writeRegister (uint8_t reg, uint8_t value);
        void writeRegister (uint8_t reg, uint8_t *data, uint8_t len);
        uint8_t readRegister (uint8_t reg) const;
        void setCe (bool b) { cePin->set (b); }

private:
        Spi *spi;
        Gpio *cePin;
        Gpio *irqPin;
        uint8_t configRegisterCopy = 0x08;
};

/*****************************************************************************/

Nrf24L0P::Nrf24L0P (Spi *spi, Gpio *cePin, Gpio *irqPin) : spi (spi), cePin (cePin), irqPin (irqPin) {}

/*****************************************************************************/

void Nrf24L0P::writeRegister (uint8_t reg, uint8_t value)
{
        // TODO delete this
        uint8_t dummy[2];
        uint8_t buf[2];
        buf[0] = reg | W_REGISTER;
        buf[1] = value;
        spi->transmit (buf, dummy, 2);

        // TODO use 2 spi->trasmit calls. First with reg-address, and second with value.
}

void Nrf24L0P::writeRegister (uint8_t reg, uint8_t *data, uint8_t len)
{
        // TODO delete this
        uint8_t dummy[6];
        dummy[0] = reg | W_REGISTER;
        memcpy (dummy + 1, data, len);
        spi->transmit (data, dummy, len + 1);
}

/*****************************************************************************/

uint8_t Nrf24L0P::readRegister (uint8_t reg) const
{
        uint8_t bufRx[2];
        uint8_t bufTx[2];
        bufTx[0] = reg | R_REGISTER;
        bufTx[1] = NOP;
        spi->transmit (bufTx, bufRx, 2);
        return bufRx[1];
}

/*****************************************************************************/

void Nrf24L0P::powerUp (Mode mode)
{
        configRegisterCopy |= (1 << PWR_UP) | mode;
        writeRegister (CONFIG, configRegisterCopy);

        // Start listenig right away
        if (mode == RX) {
                setCe (true);
        }
}

/*****************************************************************************/

void Nrf24L0P::transmit (uint8_t *data, size_t len)
{
        setCe (true);
        uint8_t dummy[6];
        uint8_t dummyRx[6];
        dummy[0] = W_TX_PAYLOAD;
        memcpy (dummy + 1, data, len);
        spi->transmit (dummy, dummyRx, len + 1);
        HAL_Delay (1);
        setCe (false);
}

/*****************************************************************************/

void Nrf24L0P::receive (uint8_t *data, size_t len)
{
        uint8_t dummy[2];
        uint8_t tmp[2];
        tmp[0] = R_RX_PAYLOAD;
        spi->transmit (tmp, dummy, 2);
        data[0] = dummy[1];
}

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

        /*---------------------------------------------------------------------------*/

        Gpio ceTx (GPIOA, GPIO_PIN_2);
        ceTx.set (false);

        Gpio irqTx (GPIOA, GPIO_PIN_3, GPIO_MODE_IT_FALLING, GPIO_PULLUP);
        HAL_NVIC_SetPriority (EXTI2_3_IRQn, 3, 0);
        HAL_NVIC_EnableIRQ (EXTI2_3_IRQn);
        irqTx.setOnToggle ([d] { d->print ("IRQ TX\n"); });

        Gpio spiTxGpiosNss (GPIOA, GPIO_PIN_4, GPIO_MODE_OUTPUT_OD, GPIO_PULLUP);
        Gpio spiTxGpiosSck (GPIOA, GPIO_PIN_5, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH, GPIO_AF0_SPI1);
        Gpio spiTxGpiosMosiMiso (GPIOB, GPIO_PIN_4 | GPIO_PIN_5, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH, GPIO_AF0_SPI1);
        Spi spiTx (SPI1);
        spiTx.setNssPin (&spiTxGpiosNss);

        Nrf24L0P nrfTx (&spiTx, &ceTx, &irqTx);

        nrfTx.setConfig (Nrf24L0P::ALL_IRQ, true, Nrf24L0P::CRC_LEN_1);
        nrfTx.setAutoAck (Nrf24L0P::ENAA_P1 | Nrf24L0P::ENAA_P0);      // Redundant
        nrfTx.setEnableDataPipe (Nrf24L0P::ERX_P1 | Nrf24L0P::ERX_P0); // Redundant
        nrfTx.setAdressWidth (Nrf24L0P::WIDTH_5);                      // Redundant
        nrfTx.setAutoRetransmit (Nrf24L0P::WAIT_1000, Nrf24L0P::RETRANSMIT_15);
        nrfTx.setChannel (100);
        nrfTx.setPayloadLength (0, 1);
        nrfTx.setPayloadLength (1, 1);
        nrfTx.setDataRate (Nrf24L0P::MBPS_1, Nrf24L0P::DBM_0);
        // Status
        nrfTx.powerUp (Nrf24L0P::TX);

        //        uint8_t buff[5];
        //        int i = 0;
        //        spi1.setCallback ([&i, &buff](uint8_t c) { buff[i++] = c; });
        /*---------------------------------------------------------------------------*/

        Gpio ceRx (GPIOB, GPIO_PIN_8);
        ceRx.set (false);

        Gpio irqRx (GPIOB, GPIO_PIN_7, GPIO_MODE_IT_FALLING, GPIO_PULLUP);
        HAL_NVIC_SetPriority (EXTI4_15_IRQn, 3, 0);
        HAL_NVIC_EnableIRQ (EXTI4_15_IRQn);
        irqRx.setOnToggle ([d] { d->print ("IRQ RX\n"); });

        Gpio spiRxGpiosNss (GPIOB, GPIO_PIN_9, GPIO_MODE_OUTPUT_OD, GPIO_PULLUP);
        Gpio spiRxGpiosSck (GPIOB, GPIO_PIN_10, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH, GPIO_AF5_SPI2);
        Gpio spiRxGpiosMisoMosi (GPIOB, GPIO_PIN_15 | GPIO_PIN_14, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH, GPIO_AF0_SPI2); // MOSI (15), MISO (14)
        Spi spiRx (SPI2);
        spiRx.setNssPin (&spiRxGpiosNss);

        Nrf24L0P nrfRx (&spiRx, &ceRx, &irqRx);

        nrfRx.setConfig (Nrf24L0P::ALL_IRQ, true, Nrf24L0P::CRC_LEN_1);
        nrfRx.setAutoAck (Nrf24L0P::ENAA_P1 | Nrf24L0P::ENAA_P0);      // Redundant
        nrfRx.setEnableDataPipe (Nrf24L0P::ERX_P1 | Nrf24L0P::ERX_P0); // Redundant
        nrfRx.setAdressWidth (Nrf24L0P::WIDTH_5);                      // Redundant
        nrfRx.setAutoRetransmit (Nrf24L0P::WAIT_1000, Nrf24L0P::RETRANSMIT_15);
        nrfRx.setChannel (100);
        nrfRx.setDataRate (Nrf24L0P::MBPS_1, Nrf24L0P::DBM_0);
        nrfRx.setPayloadLength (0, 1);
        nrfRx.setPayloadLength (1, 1);
        nrfRx.powerUp (Nrf24L0P::RX);

        /*****************************************************************************/

        uint8_t bufTx[4];
        bufTx[0] = 0x66;

        uint8_t bufRx[4] = {
                0,
        };

        while (1) {
                nrfTx.transmit (bufTx, 1);
                HAL_Delay (500);

                nrfRx.receive (bufRx, 1);
                d->print (bufRx[0]);
                d->print ("\n");
                HAL_Delay (500);
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
