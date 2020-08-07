
#include "spi_driver.h"
#include <stdint.h>


static void hal_spi_configure_device_mode(SPI_TypeDef *SPIx, uint32_t master)
{
	if(master)
	{
		SPI1->CR1 |= SPI_REG_CR1_MASTER;
	}
	else
	{
		SPI1->CR1 &= ~SPI_REG_CR1_MASTER;
	}
}

static void hal_spi_configure_phase_polarity(SPI_TypeDef *SPIx, uint32_t phase_value, uint32_t polarity_value)
{
	if(phase_value)
	{
		SPI1->CR1 |= SPI_REG_CR1_CPHA;
	}
	else
	{
		SPI1->CR1 &= ~SPI_REG_CR1_CPHA;
	}
	
	if(polarity_value)
	{
		SPI1->CR1 |= SPI_REG_CR1_CPOL;
	}
	else
	{
		SPI1->CR1 &= ~SPI_REG_CR1_CPOL;
	}
}

static void hal_spi_configure_datasize_direction(SPI_TypeDef *SPIx, uint32_t datasize_16, uint32_t lsbfirst)
{
 if(datasize_16)
 {
	 SPI1->CR1 |= SPI_REG_CR1_DFF;
 }
 else
 {
	 SPI1->CR1 &= ~SPI_REG_CR1_DFF;
 }
 
 if(lsbfirst)
 {
	 SPI1->CR1 |= SPI_REG_CR1_LSBFIRST; 
 }
 else
 {
	 SPI1->CR1 &= ~SPI_REG_CR1_LSBFIRST;
 }
}

static void hal_spi_configure_nss_master(SPI_TypeDef *SPIx, uint32_t ssm_enable)
{
	if(ssm_enable)
	{
	SPI1->CR1 |= SPI_REG_CR1_SSM;
	SPI1->CR1 |= SPI_REG_CR1_SSI;
	}
	else
	{
		SPI1->CR1 &= ~SPI_REG_CR1_SSM;
	}
}

static void hal_spi_configure_nss_slave(SPI_TypeDef *SPIx, uint32_t ssm_enable)
{
 	if(ssm_enable)
	{
	SPI1->CR1 |= SPI_REG_CR1_SSM;
	}
	else
	{
		SPI1->CR1 &= ~SPI_REG_CR1_SSM;
	}
}

static void hal_spi_enable(SPI_TypeDef *SPIx)
{
 if(!( SPI1->CR1 & SPI_REG_CR1_SPE ) )
 {
	 SPI1->CR1 |= SPI_REG_CR1_SPE;
 }	 
}

static void hal_spi_disable(SPI_TypeDef *SPIx)
{
	SPI1->CR1 &= ~SPI_REG_CR1_SPE;
}


// ------------------------------------------------------------------

void hal_spi_enable_txe_interrupt(SPI_TypeDef *SPIx)
{
	SPIx->CR2 |= SPI_REG_CR2_TXEIE_ENABLE;

}

void hal_spi_disable_txe_interrupt(SPI_TypeDef *SPIx)
{
	SPIx->CR2 &= ~SPI_REG_CR2_TXEIE_ENABLE;

}

void hal_spi_enable_rxe_interrupt(SPI_TypeDef *SPIx)
{
 SPIx->CR2 |= SPI_REG_CR2_RXNEIE_ENABLE;
}

void hal_spi_disable_rxe_interrupt(SPI_TypeDef *SPIx)
{
 SPIx->CR2 &= ~SPI_REG_CR2_RXNEIE_ENABLE;
}

void hal_spi_master_tx(__spi_handle_t *spi_handle, uint8_t *buffer, uint16_t len)
{
	spi_handle->pTxBuffPtr = buffer;
	spi_handle->TxXferSize = len;
	spi_handle->TxXferCount = len;
	
	spi_handle->State = HAL_SPI_STATE_BUSY_TX;
	
	hal_spi_enable(spi_handle->Instance);
	
	hal_spi_enable_txe_interrupt(spi_handle->Instance);

}


void hal_i2c_spi_irq_handler(__spi_handle_t *hspi )
{
	uint32_t tmp1 = 0, tmp2 = 0;
	
	
	tmp1 = (hspi->Instance->SR & SPI_REG_SR_TXE_FLAG);   //Check to see TXE flag is set in the status Register
	tmp2 = (hspi->Instance->SR & SPI_REG_CR2_TXEIE_ENABLE);  // Check whether TXEIE bit is enabled in CR2 register
	
	if ( tmp1 != RESET && tmp2 != RESET )
	{
		hal_spi_handle_tx_interrupt(hspi);
	}

	tmp1 = (hspi->Instance->SR & SPI_REG_SR_RXNE_FLAG);  //Check to see RXNE flag is set in the status Register
	tmp2 = (hspi->Instance->SR & SPI_REG_CR2_RXNEIE_ENABLE);  // Check whether RXNEIE bit is enabled in CR2 register
	
	if ( tmp1 != RESET && tmp2 != RESET )
	{
		hal_spi_handle_rx_interrupt(hspi);
	}	

}


void hal_spi_master_rx(__spi_handle_t *spi_handle, uint8_t *buffer, uint16_t len)
{
	uint32_t i = 0, val;
	
	//this is dummy tx
	spi_handle->pTxBuffPtr = buffer;
	spi_handle->TxXferSize = len;
	spi_handle->TxXferCount = len;
	
	//data will be read to rx buffer
	spi_handle->pRxBuffPtr = buffer;
	spi_handle->RxXferSize = len;
	spi_handle->RxXferCount = len;
	
	//driver is busy in rx
	spi_handle->State = HAL_SPI_STATE_BUSY_RX;
	
	/* REad data register once before enabling
	RXNE interrput to make sure DR is empty*/
	
	val = spi_handle->Instance->DR;
	
	//Now Enable both TXE and RXNE interrupt
	
	hal_spi_enable_txe_interrupt(spi_handle->Instance);
	hal_spi_enable_rxe_interrupt(spi_handle->Instance);
	
	
}

void hal_spi_slave_tx(__spi_handle_t *spi_handle, uint8_t *tx_buffer, uint16_t len)
{
	
	//populate the pointer and the kength information to TX the data
	
	spi_handle->pTxBuffPtr = tx_buffer;
	spi_handle->TxXferSize = len;
	spi_handle->TxXferCount = len;
	
	//Pointer to handle dummy rx, you can reuse the same pointer
	
	spi_handle->pRxBuffPtr = tx_buffer;
	spi_handle->RxXferSize = len;
	spi_handle->RxXferCount = len;
	
	//driver is busy in tx
	
	spi_handle->State = HAL_SPI_STATE_BUSY_TX;
	
	hal_spi_enable(spi_handle->Instance);
	
	//Now ENable both TXE and RXNE interrupt
	
  hal_spi_enable_txe_interrupt(spi_handle->Instance);
	hal_spi_enable_rxe_interrupt(spi_handle->Instance);
	
	
}

void hal_spi_slave_rx(__spi_handle_t *spi_handle, uint8_t *rx_buffer, uint16_t len)
{

	
	
	//populate the pointer and the length information to RX the data
	spi_handle->pRxBuffPtr = rx_buffer;
	spi_handle->RxXferSize = len;
	spi_handle->RxXferCount = len;   
	
	//driver is busy in tx
	
	spi_handle->State = HAL_SPI_STATE_BUSY_RX;
	
  hal_spi_enable(spi_handle->Instance);
	
	//Now Enable  RXNE interrupt

	hal_spi_enable_rxe_interrupt(spi_handle->Instance);
	
	
}

void hal_spi_handle_tx_interrupt(__spi_handle_t *hspi)
{
	//transmit data 8 bit mode
	if(hspi->Init.DataSize == SPI_ENABLE_8_BIT_DF )
	{
		hspi->Instance->DR = (*hspi->pTxBuffPtr);
		hspi->TxXferCount--; //we sent 1 bit
	}
	
	//transmit data 16 bit mode
	else if(hspi->Init.DataSize == SPI_ENABLE_16_BIT_DF )
	{
		hspi->Instance->DR = (*(uint16_t*)hspi->pTxBuffPtr);
		hspi->pTxBuffPtr += 2;
		hspi->TxXferCount -= 2; //we sent 2 bits
	}
	
	if(hspi->TxXferCount == 0)
	{
		hal_spi_close_tx_interrupt(hspi);
	}

}

void hal_spi_handle_rx_interrupt(__spi_handle_t *hspi)
{
	if(hspi->Init.DataSize == SPI_ENABLE_8_BIT_DF )
	{
	 if(hspi->pRxBuffPtr++)  //Null Check
	 (*hspi->pRxBuffPtr++) = hspi->Instance->DR;
	 hspi->RxXferCount--;
	}
	else 
	{
		*((uint16_t*)hspi->pRxBuffPtr++) = hspi->Instance->DR;
		hspi->pRxBuffPtr+= 2;
		hspi->RxXferCount-= 2;
	}
	
	if(hspi->RxXferCount == 0)
	{
		hal_spi_close_rx_interrupt(hspi);
	}
}


void hal_spi_close_tx_interrupt(__spi_handle_t *hspi)
{
	//Disable TXE interrupt
	hal_spi_disable_txe_interrupt(hspi->Instance);
	
	if (hspi->Init.Mode && (hspi->State != HAL_SPI_STATE_BUSY_TX ) )
	{
		hspi->State = HAL_SPI_STATE_READY;
	}
}

void hal_spi_close_rx_interrupt(__spi_handle_t *hspi)
{
	while (hal_spi_is_bus_busy(hspi->Instance));
	//DIsable RXNE Interrupt
	
	hal_spi_disable_rxe_interrupt(hspi->Instance);
	 hspi->State = HAL_SPI_STATE_READY;
	
	
}

uint8_t hal_spi_is_bus_busy(SPI_TypeDef *SPIx)
{
	

}
