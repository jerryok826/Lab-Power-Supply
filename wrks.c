/* ------------------------------------------------------------
------------------------------------------------------------ */

void ESP8266_SSD1322::begin(uint8_t i2caddr, bool reset) {
	_i2caddr = i2caddr;

	// set pin directions
	if (sid != -1) {
		pinMode(dc, OUTPUT);
		pinMode(cs, OUTPUT);
		if (hwSPI) {
			SPI.begin();
			SPI.setClockDivider (SPI_CLOCK_DIV2); // 26/2 = 13 MHz (freq ESP8266 26 MHz)
		}
	}

	if (reset && rst)
	{
		// Setup reset pin direction (used by both SPI and I2C)
		pinMode(rst, OUTPUT);
		// bring out of reset
		digitalWrite(rst, HIGH);
		delay(100);
		// bring reset low
		digitalWrite(rst, LOW);
		delay(400);
		// bring out of reset
		digitalWrite(rst, HIGH);
	}

//#ifdef SSD1322_256_64

	ssd1322_command(SSD1322_SETCOMMANDLOCK);// 0xFD
	ssd1322_data(0x12);// Unlock OLED driver IC

	ssd1322_command(SSD1322_DISPLAYOFF);// 0xAE

	ssd1322_command(SSD1322_SETCLOCKDIVIDER);// 0xB3
	ssd1322_data(0x91);// 0xB3

	ssd1322_command(SSD1322_SETMUXRATIO);// 0xCA
	ssd1322_data(0x3F);// duty = 1/64

	ssd1322_command(SSD1322_SETDISPLAYOFFSET);// 0xA2
	ssd1322_data(0x00);

	ssd1322_command(SSD1322_SETSTARTLINE);// 0xA1
	ssd1322_data(0x00);

	ssd1322_command(SSD1322_SETREMAP);// 0xA0
	ssd1322_data(0x14);//Horizontal address increment,Disable Column Address Re-map,Enable Nibble Re-map,Scan from COM[N-1] to COM0,Disable COM Split Odd Even
	ssd1322_data(0x11);//Enable Dual COM mode

	ssd1322_command(SSD1322_SETGPIO);// 0xB5
	ssd1322_data(0x00);// Disable GPIO Pins Input

	ssd1322_command(SSD1322_FUNCTIONSEL);// 0xAB
	ssd1322_data(0x01);// selection external vdd

	ssd1322_command(SSD1322_DISPLAYENHANCE);// 0xB4
	ssd1322_data(0xA0);// enables the external VSL
	ssd1322_data(0xFD);// 0xfFD,Enhanced low GS display quality;default is 0xb5(normal),

	ssd1322_command(SSD1322_SETCONTRASTCURRENT);// 0xC1
	ssd1322_data(0xFF);// 0xFF - default is 0x7f

	ssd1322_command(SSD1322_MASTERCURRENTCONTROL);// 0xC7
	ssd1322_data(0x0F);// default is 0x0F

	// Set grayscale
	ssd1322_command(SSD1322_SELECTDEFAULTGRAYSCALE); // 0xB9

 	ssd1322_command(SSD1322_SETPHASELENGTH);// 0xB1
	ssd1322_data(0xE2);// default is 0x74

	ssd1322_command(SSD1322_DISPLAYENHANCEB);// 0xD1
	ssd1322_data(0x82);// Reserved;default is 0xa2(normal)
	ssd1322_data(0x20);//

	ssd1322_command(SSD1322_SETPRECHARGEVOLTAGE);// 0xBB
	ssd1322_data(0x1F);// 0.6xVcc

	ssd1322_command(SSD1322_SETSECONDPRECHARGEPERIOD);// 0xB6
	ssd1322_data(0x08);// default

	ssd1322_command(SSD1322_SETVCOMH);// 0xBE
	ssd1322_data(0x07);// 0.86xVcc;default is 0x04

	ssd1322_command(SSD1322_NORMALDISPLAY);// 0xA6

	ssd1322_command(SSD1322_EXITPARTIALDISPLAY);// 0xA9

//#endif
	//Clear down image ram before opening display
	fill(0x00);

	ssd1322_command(SSD1322_DISPLAYON);// 0xAF
}

void ESP8266_SSD1322::invertDisplay(uint8_t i) {
	if (i) {
		ssd1322_command(SSD1322_INVERSEDISPLAY);
	} else {
		ssd1322_command(SSD1322_NORMALDISPLAY);
	}
}
