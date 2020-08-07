#ifndef SLAVE_SPI_H
#define SLAVE_SPI_H


#include <stdint.h>


#include "gpio_header.h"
#include "spi_driver.h"


//Define used for transfer Communication
#define CMD_MASTER_READ           ((uint16_t)0x1234)
#define CMD_MASTER_WRITE          ((uint16_t)0x5678)
#define CMD_LENGTH                 2
#define DATA_LENGTH                4
#define ACK_LEN                    2
#define SPI_ACK_BYTES              0xD5E5


//Defination for SPIx NVIC
#define SPIx_IRQn                SPI2_IRQn
#define SPIx_IRQHandler          SPI2_IRQHandler

#define EXTIx_IRQn               EXTI15_10_IRQn
#define EXTIx_IRQHandler         EXTI15_10_IRQHandler

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


__spi_handle_t SpiHandle;


uint8_t master_write_data[] = {0xa, 0xb, 0xc, 0xd};
uint8_t master_read_buffer[4];


uint8_t slave_tx_buffer[DATA_LENGTH] = {0x55, 0xaa, 0x55, 0xaa};
uint8_t slave_rx_buffer[DATA_LENGTH];

void spi_gpio_init(void);

void led_init(void);

void led_toggle(GPIO_TypeDef *GPIOx, uint16_t pin_no);

void delay_gen(void);

void delay_ms(void);

uint8_t Buffer_cmp( uint8_t arr1[], uint8_t arr2[], uint8_t len);



#endif