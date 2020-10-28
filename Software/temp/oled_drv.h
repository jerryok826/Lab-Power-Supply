//////////////////////////////////////////////////////////////////////
// meter.hpp -- Base class for analog meter
// Date: Wed Dec  6 22:50:23 2017   (C) Warren W. Gay ve3wwg@gmail.com
///////////////////////////////////////////////////////////////////////

#ifndef OLED_HPP
#define OLED_HPP

struct Meter {
	float		range;
	float		value;		// Meter value (volts)
	short		cx, cy;		// Center
	short		rd;		// Radius difference
	short		cr;		// Circle radius
	short		ocr, icr;	// Outer and inner radius
	short		dx;		// Tick delta x
	short		dy;		// Tick delta y
	short		tw;		// Label text width
};

void oled_init_real(void); 
void meter_init(struct Meter *m,float range);
void oled_update(void);
void oled_init (void);

int spi_1_init(void);
int spi_2_init(void);

#endif // OLED_HPP

// End oled.hpp
