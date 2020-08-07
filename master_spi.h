#ifndef MASTER_SPI_H
#define MASTER_SPI_H

#include "gpio_header.h"

#include <stdint.h>

#include "master_spi.h"
#include "gpio_header.h"
#include "spi_driver.h"


#define CMD_LENGTH      2
#define ACK_LEN         4

#define DATA_LENGTH     4

#define CMD_MASTER_WRITE  0x5678
#define CMD_MASTER_READ  0x1234



#define GPIO_PIN_13   13
#define GPIO_PIN_14   14
#define GPIO_PIN_15   15

#define GPIO_PIN_5    5
#define GPIO_LED_PIN GPIO_PIN_5

#define GPIOC_PIN_13  13

#define GPIO_BUTTON_PIN GPIOC_PIN_13

#define SPI_CLK_PIN    GPIO_PIN_13
#define SPI_MISO_PIN   GPIO_PIN_14
#define SPI_MOSI_PIN   GPIO_PIN_15

//SPI Alternate Functionality value
#define GPIO_PIN_AF_SPI2   0x05

int TestReady = 0;

//slave will reply this data when master issue read command
uint8_t slave_reply_data[] = {0x55, 0xaa, 0x55, 0xaa};


//Master read/write Buffer
uint8_t master_write_data[] = {0xa, 0xb, 0xc, 0xd};
uint8_t master_read_buffer[4];



__spi_handle_t SpiHandle;


void SPI2_IRQHandler(void);

void EXTI15_10_IRQHandler(void);

void delay_gen(void);

void spi_gpio_init(void);

void led_init(void);

void led_toggle(GPIO_TypeDef *GPIOx, uint16_t pin_no);

void delay_ms(void);

uint8_t Buffer_cmp( uint8_t arr1[], uint8_t arr2[], uint8_t len);




#endif