#include "master_spi.h"



int main(void)
{
	
	uint8_t addrcmd[CMD_LENGTH];
  uint8_t ack_buf[2];

	
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
		
		//read from slave
		
		//Read Master Command
		addrcmd[0] = (uint8_t) CMD_MASTER_READ;
		addrcmd[1] = (uint8_t) (CMD_MASTER_READ >> 8);
		
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
		
		//Start Receving the data Stream
		hal_spi_master_rx(&SpiHandle, master_read_buffer, DATA_LENGTH);
		
	  while(SpiHandle.State != HAL_SPI_STATE_READY);
		
		if(Buffer_cmp(master_read_buffer, slave_reply_data, DATA_LENGTH))
		{
		 //We didn't received what is expected
			led_toggle(GPIOA, GPIO_LED_PIN);
		}
		else
   {
	  //We received Correctly
		 hal_gpio_write_pin(GPIOA, GPIO_LED_PIN, 1);
	 }
		
	
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

void delay_ms(void)
{
  uint32_t i;
	for(i = 0; i <= 5000; i++);
}


void led_init(void)
{
	_HAL_RCC_GPIOA_CLK_ENABLE();
	
	gpio_pin_conf_t  led_conf;
	
	led_conf.pin = GPIO_LED_PIN;
	led_conf.mode = GPIO_PIN_OUTPUT_MODE ;
	led_conf.op_type = GPIO_PIN_OUTPUT_PULLUP;
	led_conf.pull = GPIO_PIN_PULL_UP;
	led_conf.speed = GPIO_SPEED_MEDIUM;


}

void led_toggle(GPIO_TypeDef *GPIOx, uint16_t pin_no)
{
	hal_gpio_write_pin(GPIOx, pin_no, 1);
	delay_ms();
	hal_gpio_write_pin(GPIOx, pin_no, 0);
}

void spi_gpio_init(void)
{
	
	gpio_pin_conf_t spi_conf;
	
	_HAL_RCC_GPIOB_CLK_ENABLE();
	
	//Configure GPOIO_PIN_13 For SPI_CLK Functionality
	spi_conf.pin = SPI_CLK_PIN;
	spi_conf.mode = GPIO_PIN_ALTERNATE_FUNCTION_MODE;
	spi_conf.op_type = GPIO_PIN_OUTPUT_PULLUP;
	spi_conf.pull = GPIO_PIN_PULL_DOWN;
	spi_conf.speed = GPIO_SPEED_MEDIUM;
	
	hal_gpio_configure_pin_alternate_function(GPIOB, SPI_CLK_PIN, GPIO_PIN_AF_SPI2);
	
	//Configure GPIO_PIN_14 For SPI_MISO Functionality
	spi_conf.pin = SPI_MISO_PIN;
	spi_conf.pull = GPIO_PIN_PULL_DOWN ;
  hal_gpio_configure_pin_alternate_function(GPIOB, SPI_MISO_PIN, GPIO_PIN_AF_SPI2);	
	
	//Configure GPIO_PIN_14 For SPI_MOSI Functionality
	spi_conf.pin = SPI_MOSI_PIN;
	spi_conf.pull = GPIO_PIN_PULL_DOWN ;
  hal_gpio_configure_pin_alternate_function(GPIOB, SPI_MOSI_PIN, GPIO_PIN_AF_SPI2);	
	
	
}


uint8_t Buffer_cmp( uint8_t arr1[], uint8_t arr2[], uint8_t len)
{
	for(uint8_t i = 0; i < len; i++)
	{
	  if(arr1[i] == arr2[i])
		{
		  break;
		}
		else
		{
			return 1;
		
		}
	
	}
		return 0;
}
