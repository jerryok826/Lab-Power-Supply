#include "DEV_Config.h"
#include <bcm2835.h>
#include <stdio.h>		//printf()
/********************************************************************************
function:	System Init and exit
note:
	Initialize the communication method
********************************************************************************/
uint8_t System_Init(void)
{
    if(!bcm2835_init()) {
        printf("bcm2835 init failed   !!! \r\n");
        return 1;
    } else {
        printf("bcm2835 init success  !!! \r\n");
    }

    bcm2835_gpio_fsel(OLED_RST,      BCM2835_GPIO_FSEL_OUTP);
    bcm2835_gpio_fsel(OLED_DC,       BCM2835_GPIO_FSEL_OUTP);
    bcm2835_gpio_fsel(OLED_CS,       BCM2835_GPIO_FSEL_OUTP);

#if USE_SPI_4W
	printf("USE_SPI\r\n");
    bcm2835_spi_begin(); 										 //Start spi interface, set spi pin for the reuse function
    bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);     //High first transmission
    bcm2835_spi_setDataMode(BCM2835_SPI_MODE0);                  //spi mode 1
    bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_2048);   //Frequency
    bcm2835_spi_chipSelect(BCM2835_SPI_CS0);                     //set CE0
    bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS0, LOW);     //enable cs0
#elif USE_SPI_3W	
#elif USE_IIC
    OLED_DC_1;
    OLED_CS_0;
    printf("USE_IIC\r\n");
    bcm2835_i2c_begin();
    bcm2835_i2c_setSlaveAddress(0x3d);  //i2c address
	
	/*
	BCM2835_I2C_CLOCK_DIVIDER_2500 ：2500 = 10us = 100 kHz 
	BCM2835_I2C_CLOCK_DIVIDER_626  ：622 = 2.504us = 399.3610 kHz
	150 = 60ns = 1.666 MHz (default at reset)
	148 = 59ns = 1.689 MHz
	*/	
	bcm2835_i2c_setClockDivider(BCM2835_I2C_CLOCK_DIVIDER_626);
//  bcm2835_i2c_set_baudrate(100000);    //100k baudrate

#endif

    return 0;
}

void System_Exit(void)
{
#if USE_SPI_4W
    bcm2835_spi_end();
#elif USE_SPI_3W	

#elif USE_IIC
    bcm2835_i2c_end();
#endif
    bcm2835_close();
}

/********************************************************************************
function:	Hardware interface
note:
	SPI4W_Write_Byte(value) : 
		hardware SPI
	SPI3W_Write_Byte(Value, Cmd) : 
		Gpio analog SPI
	I2C_Write_Byte(value, cmd):
		hardware I2C
********************************************************************************/
void SPI4W_Write_Byte(uint8_t value)
{
	char buf[1] = {0XFF};
	buf[0] = (char)value;
    bcm2835_spi_transfern(buf, 1);
}

void I2C_Write_Byte(uint8_t value, uint8_t Cmd)
{
    char buf[2] = {IIC_RAM,0XFF};
    int ref;
    buf[1] = (char)value;
	
    if(Cmd == IIC_RAM)
        buf[0] = IIC_RAM;
    else
        buf[0] = IIC_CMD;
	
    ref = bcm2835_i2c_write(buf, 2);
    while(ref != 0) {
        ref = bcm2835_i2c_write(buf, 2);
        if(ref == 0)
            break;
    }	
}

/********************************************************************************
function:	Delay function
note:
	Driver_Delay_ms(xms) : Delay x ms
	Driver_Delay_us(xus) : Delay x us
********************************************************************************/
void Driver_Delay_ms(uint32_t xms)
{
    bcm2835_delay(xms);
}

void Driver_Delay_us(uint32_t xus)
{
    int j;
    for(j=xus; j > 0; j--);
}
