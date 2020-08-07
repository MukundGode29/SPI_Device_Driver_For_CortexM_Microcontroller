
#include<stdint.h>

#include "main.h"

int TestReady = 0;

uint8_t master_write_data[] = {0xa, 0xb, 0xc, 0xd};
uint8_t master_read_data[];

int main(void)
{

	
	spi_gpio_init();
	
  led_init(); //to make use of led
	
	//Configure user button interrupt
	_HAL_RCC_GPIOC_CLK_ENABLE();
	hal_gpio_configure_interrupt(GPIO_BUTTON_PIN, INT_FALLING_EDGE);
	hal_gpio_interrupt_enable(GPIO_BUTTON_PIN, EXTI15_10_IRQn);
	
	//Enable the clock for SPI12
	_HAL_RCC_SPI2_CLCK_ENABLE();
	
	SpiHandle.Instance                    = SPI_2;
  SpiHandle.Init.BaudratePreScaler      = SPI_REG_CR1_BR_FCLK_DIV_32;
	SpiHandle.Init.Direction              = SPI_ENABLE_2_LINE_UNI_DIR;
	SpiHandle.Init.CLKPhase               = SPI_SECOND_CLK_TRANS;
	SpiHandle.Init.CLKPolarity            = SPI_CPOL_LOW;
	SpiHandle.Init.DataSize               = SPI_ENABLE_8_BIT_DF;
	SpiHandle.Init.FirstBit               = SPI_TX_MSB_FIRST;
	SpiHandle.Init.Mode                   = SPI_MASTER_MODE_SEL;
	SpiHandle.Init.NSS                    = SPI_ENABLE_SSM;
	
	SpiHandle.State                       = HAL_SPI_STATE_READY;
	
	
	hal_spi_init(&SpiHandle); //Call driver API to initialise the API
	
	//Enable the IRQ in NVIC
	NVIC_EnableIRQ (SPI2_IRQn);
	
	
	/****************************************************************************
	*                   MAster command sending code
	*                   read and Write Command
	*
	****************************************************************************/
	
	while(1)
	{
		//Check for State Ready
		while(SpiHandle.State != HAL_SPI_STATE_READY);
		
		//Write Master Command
		addrcmd[0] = (uint8_t) CMD_MASTER_WRITE;
		addrcmd[1] = (uint8_t) (CMD_MASTER_WRITE >> 8);
		
		//first send master write cmd to slave
		hal_spi_master_tx(&SpiHandle, addrcmd, CMD_LENGTH);
		
		//application can wait here or can do other task untill above TX function is completed
		while(SpiHandle.State != HAL_SPI_STATE_READY);
		
		//this delay help salve to be ready with acknowledge bytes
		delay_gen();
		
		//Read back the Acknowledgement from Slave
		hal_spi_master_rx(&SpiHandle, ack_buf, ACK_LEN);
		
		//wait until ACK reception finish
		while(SpiHandle.State != HAL_SPI_STATE_READY);
		
		//Did we recive valid ack from slave
		if(ack_buf[0] == 0xE5 && ack_buf[1] == 0xD5)
		{
			//if Correct
			led_toggle(GPIOA, GPIO_LED_PIN);
			//mem_set(ack_buf, 0, 2);
		
		}
		else
		{
			//if Invalid
			//assert_error();
			//mem_set(ack_buf, 0, 2);
		}
		
		//Now Send the Data Stream
		hal_spi_master_tx(&SpiHandle, master_write_data, DATA_LENGTH);
		while(SpiHandle.State != HAL_SPI_STATE_READY);
		delay_gen();
		
		
		
	
	}
	
	return 0;
}

void SPI2_IRQHandler(void)
{
	//Call the driver spi to process this interrupt
	hal_i2c_spi_irq_handler(&SpiHandle);
}

void EXTI15_10_IRQHandler(void)
{
	hal_gpio_clear_interrupt(GPIO_BUTTON_PIN); //Clear the present interrupt from PR register
	
	//DO your interrupt handling
	TestReady = SET;
}

void delay_gen(void)
{
	uint32_t i;
	for(i=0; i <= 5000; i++);
}



