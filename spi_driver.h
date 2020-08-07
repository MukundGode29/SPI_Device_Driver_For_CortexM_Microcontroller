#ifndef _HAL_SPI_H
#define _HAL_SPI_H

#include "stm32f446xx.h"


/*           Register Bit Defintion                                                  */


//   CR1 register configuration

#define SPI_REG_CR1_BIDI_MODE            ( (uint32_t) 1 << 15 )
#define SPI_ENABLE_2_LINE_UNI_DIR        0
#define SPI_ENABLE_1_LINE_BI_DIR         1

#define SPI_REG_CR1_DFF                  ( (uint32_t) 1 << 11 )
#define SPI_ENABLE_8_BIT_DF              0
#define SPI_ENABLE_16_BIT_DF             1

#define SPI_REG_CR1_SSM                  ( (uint32_t) 1 << 9 )
#define SPI_DISENABLE_SSM                 0
#define SPI_ENABLE_SSM                    1

#define SPI_REG_CR1_SSI                   ( (uint32_t) 1 << 8 )

#define SPI_REG_CR1_LSBFIRST              ( (uint32_t) 1 << 7 )
#define SPI_TX_MSB_FIRST                    0
#define SPI_TX_LSB_FIRST                    1

#define SPI_REG_CR1_SPE                    ( (uint32_t) 1 << 6 )
#define SPI_DISABLE_PERIPHERAL              0
#define SPI_ENABLE_PERIPHERAL               1

#define SPI_REG_CR1_BR_FCLK_DIV_2          ( (uint32_t) 0 << 3)                    
#define SPI_REG_CR1_BR_FCLK_DIV_4          ( (uint32_t) 1 << 3)
#define SPI_REG_CR1_BR_FCLK_DIV_8          ( (uint32_t) 2 << 3)
#define SPI_REG_CR1_BR_FCLK_DIV_16         ( (uint32_t) 3 << 3)
#define SPI_REG_CR1_BR_FCLK_DIV_32         ( (uint32_t) 4 << 3)
#define SPI_REG_CR1_BR_FCLK_DIV_64         ( (uint32_t) 5 << 3)
#define SPI_REG_CR1_BR_FCLK_DIV_128        ( (uint32_t) 6 << 3)
#define SPI_REG_CR1_BR_FCLK_DIV_256        ( (uint32_t) 7 << 3)

#define SPI_REG_CR1_MASTER                 ( (uint32_t) 1 << 2)
#define SPI_SLAVE_MODE_SEL                 0
#define SPI_MASTER_MODE_SEL                1

#define SPI_REG_CR1_CPOL                   ( (uint32_t) 1 << 1)
#define SPI_CPOL_LOW                       0
#define SPI_CPOL_HIGH                      1


#define SPI_REG_CR1_CPHA                   ( (uint32_t) 1 << 0)
#define SPI_FIRST_CLK_TRANS                 0
#define SPI_SECOND_CLK_TRANS                1


//   CR2 register configuration

#define SPI_REG_CR2_TXEIE_ENABLE          ( (uint32_t) 1 << 7 )
#define SPI_REG_CR2_RXNEIE_ENABLE         ( (uint32_t) 1 << 6 )
#define SPI_REG_CR2_ERRIE_ENABLE          ( (uint32_t) 1 << 5 )

#define SPI_REG_CR2_FRF                   ( (uint32_t) 1 << 4 )
#define SPI_MOTOROLA_MODE                 0
#define SPI_TI_MODE                       1

#define SPI_REG_CR2_SSOE_ENABLE           ( (uint32_t) 1 << 2 )


//  SPI Status Register COnfiguration

#define SPI_REG_SR_FRE_ERROR              ( (uint32_t) 1 << 8 )
#define SPI_REG_SR_BSY_FLAG               ( (uint32_t) 1 << 7 )
#define SPI_REG_SR_TXE_FLAG               ( (uint32_t) 1 << 1 )
#define SPI_REG_SR_RXNE_FLAG              ( (uint32_t) 1 << 0 )

#define RESET 0
#define SET !RESET

//SPI Device BAse Address

#define SPI_1 SPI1
#define SPI_2 SPI2
#define SPI_3 SPI3

#define SPI_IS_BUSY  1
#define SPI_IS_NOT_BUSY 0

//MACRO Enabled Clock for different SPI Devices



#define _HAL_RCC_SPI1_CLCK_ENABLE()       ( RCC->APB2ENR |= ( 1 << 12 ) )
#define _HAL_RCC_SPI2_CLCK_ENABLE()       ( RCC->APB1ENR |= ( 1 << 14 ) )
#define _HAL_RCC_SPI3_CLCK_ENABLE()       ( RCC->APB1ENR |= ( 1 << 15 ) )

/***************************************************************/
/*                      Data Structure by SPI                  */
/***************************************************************/
typedef enum
{
	HAL_SPI_STATE_RESET  = 0x00,
	HAL_SPI_STATE_READY  = 0x01,
	HAL_SPI_STATE_BUSY   = 0x02,
	HAL_SPI_STATE_BUSY_TX = 0x12,
	HAL_SPI_STATE_BUSY_RX = 0x22,
	HAL_SPI_STATE_BUSY_TX_RX = 0x32,
	HAL_SPI_STATE_ERROR =  0x03
}hal_spi_state_t;


typedef struct 
{
	uint32_t Mode;
	
	uint32_t Direction;
	
	uint32_t DataSize;
	
	uint32_t CLKPolarity;
	
	uint32_t CLKPhase;
	
	uint32_t NSS;
	
	uint32_t BaudratePreScaler; 
	
	uint32_t FirstBit;
	
}spi_init_t;

typedef struct
{
	SPI_TypeDef *Instance;
	
	spi_init_t  Init;
	
	uint8_t     *pTxBuffPtr;
	
	uint16_t    TxXferSize;

  uint16_t    TxXferCount;

  uint8_t     *pRxBuffPtr;	
	
  uint16_t    RxXferSize;

  uint16_t    RxXferCount;
	
	hal_spi_state_t State;
	 
}__spi_handle_t;



//     DRIVER Exposed APIs

void hal_spi_init(__spi_handle_t *spi_handle);

void hal_spi_master_tx(__spi_handle_t *spi_handle, uint8_t *buffer, uint16_t len);

void hal_spi_slave_tx(__spi_handle_t *spi_handle, uint8_t *rcvbuffer, uint16_t len);

void hal_spi_master_rx(__spi_handle_t *spi_handle, uint8_t *buffer, uint16_t len);

void hal_spi_slave_rx(__spi_handle_t *spi_handle, uint8_t *rcvbuffer, uint16_t len);

void hal_i2c_spi_irq_handler(__spi_handle_t *hspi);

void hal_spi_enable_txe_interrupt(SPI_TypeDef *SPIx);

void hal_spi_disable_txe_interrupt(SPI_TypeDef *SPIx);

void hal_spi_enable_rxe_interrupt(SPI_TypeDef *SPIx);

void hal_spi_disable_rxe_interrupt(SPI_TypeDef *SPIx);

void hal_i2c_spi_irq_handler(__spi_handle_t *hspi );

void hal_spi_handle_tx_interrupt(__spi_handle_t *hspi); 

void hal_spi_close_tx_interrupt(__spi_handle_t *hspi);

void hal_spi_handle_rx_interrupt(__spi_handle_t *hspi);

void hal_spi_close_tx_interrupt(__spi_handle_t *hspi);


//Static helper Function

static void hal_spi_configure_device_mode(SPI_TypeDef *SPIx, uint32_t master);

static void hal_spi_configure_phase_polarity(SPI_TypeDef *SPIx, uint32_t phase_value, uint32_t polarity_value);

static void hal_spi_configure_datasize_direction(SPI_TypeDef *SPIx, uint32_t datasize_16, uint32_t lsbfirst);

static void hal_spi_configure_nss_master(SPI_TypeDef *SPIx, uint32_t ssm_enable);

static void hal_spi_configure_nss_slave(SPI_TypeDef *SPIx, uint32_t ssm_enable);

static void hal_spi_enable(SPI_TypeDef *SPIx);

static void hal_spi_disable(SPI_TypeDef *SPIx);

#endif