

#include "slave_spi.h"


int main(void)
{
	uint16_t ack_bytes = SPI_ACK_BYTES;
	uint16_t master_cmd;
	uint8_t rcv_cmd[2];
	
	
	
	
	spi_gpio_init();
	
	//to use led
	led_init();
	
	//Enable the clock for SPI12
	_HAL_RCC_SPI2_CLCK_ENABLE();
	
	SpiHandle.Instance                    = SPI_2;
  SpiHandle.Init.BaudratePreScaler      = SPI_REG_CR1_BR_FCLK_DIV_32;
	SpiHandle.Init.Direction              = SPI_ENABLE_2_LINE_UNI_DIR;
	SpiHandle.Init.CLKPhase               = SPI_SECOND_CLK_TRANS;
	SpiHandle.Init.CLKPolarity            = SPI_CPOL_LOW;
	SpiHandle.Init.DataSize               = SPI_ENABLE_8_BIT_DF;
	SpiHandle.Init.FirstBit               = SPI_TX_MSB_FIRST;
	SpiHandle.Init.Mode                   = SPI_SLAVE_MODE_SEL;
	SpiHandle.Init.NSS                    = SPI_ENABLE_SSM;
	
	SpiHandle.State                       = HAL_SPI_STATE_READY;
	
	
	hal_spi_init(&SpiHandle); //Call driver API to initialise the API
	
	//Enable the IRQ in NVIC
	NVIC_EnableIRQ (SPI2_IRQn);
	
	
	while(1)
	{
		
		while(SpiHandle.State != HAL_SPI_STATE_READY);
		
		//Receive the master command first
		hal_spi_slave_rx(&SpiHandle, rcv_cmd, CMD_LENGTH);

		while(SpiHandle.State != HAL_SPI_STATE_READY);
		
		//This is the command Slave got
		master_cmd = (uint16_t)(rcv_cmd[1] << 8 | rcv_cmd[0]);
		
		//it is a Valid Command
		if(master_cmd == CMD_MASTER_WRITE || master_cmd == CMD_MASTER_READ )
		{
			//Yes, send out the ACK Bytes
			hal_spi_slave_tx(&SpiHandle, (uint8_t*)&ack_bytes, ACK_LEN);
			while(SpiHandle.State != HAL_SPI_STATE_READY);
		}
		else
		{
		 //Error
			led_toggle(GPIOA, GPIO_LED_PIN);
		}
		
		
		//Is it a write Command
		if(master_cmd == CMD_MASTER_WRITE)
		{
			//master wants to write, so get read to write te byte
			hal_spi_slave_rx(&SpiHandle, slave_rx_buffer, DATA_LENGTH);
			
			
			while(SpiHandle.State != HAL_SPI_STATE_READY);
			
			//Compare the recive data with expected data
			if(Buffer_cmp(master_write_data, slave_rx_buffer, 4))
			{
			  //Doesnt Match Error
				led_toggle(GPIOA, GPIO_LED_PIN);
			}
			else
      {
				//MAtches
				hal_gpio_write_pin(GPIOA, GPIO_LED_PIN, 1);
			
			}
		 
		}
		
		if(master_cmd == CMD_MASTER_READ)
		{
			//Master wants to read so transmit the data to master
			hal_spi_slave_tx(&SpiHandle, slave_tx_buffer, DATA_LENGTH);
			
			//hang on little while, till TXing Finishes
			while(SpiHandle.State != HAL_SPI_STATE_READY);
		
		}
	
	}
	


 return 0;
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

