// SSD1322 driver
// https://github.com/littlevgl/lvgl/issues/441


void Write_Command(unsigned char command)
{
PORTA=command;
CLR_CS;
CLR_CD;
SET_RD;
CLR_WR;
_delay_us(5);
SET_WR;
_delay_us(5);
SET_CS;
}

void Write_Data(unsigned char date)
{
PORTA=date;
CLR_CS;
SET_CD;
SET_RD;
CLR_WR;
_delay_us(5);
SET_WR;
_delay_us(5);
SET_CS;
}

void Command_Lock(unsigned char lock)
{
unsigned char value;
value = lock;
Write_Command(0xfd);
Write_Data(value);
}

void Sleep_Mode(unsigned char mode)
{
unsigned char value;
value = mode;
Write_Command(value);
}
void Column_Address(unsigned char start , unsigned char end)
{
unsigned char value_1,value_2;
value_1 = start ;
value_2 = end ;
Write_Command(0x15);
Write_Data(value_1);
Write_Data(value_2);
}

void Row_Address(unsigned char start , unsigned char end)
{
unsigned char value_1,value_2;
value_1 = start ;
value_2 = end ;
Write_Command(0x75);
Write_Data(value_1);
Write_Data(value_2);
}
void Dual_COM_Line(unsigned char mode_1, unsigned char mode_2)
{
unsigned char value_1,value_2;
value_1 = mode_1;
value_2 = mode_2;
Write_Command(0xA0);
Write_Data(mode_1);
Write_Data(mode_2);
}
void Display_Start_Line(unsigned char line)
{
unsigned char value;
value = line;
Write_Command(0xA1);
Write_Data(value);
}

void Display_Offset(unsigned char offset)
{
unsigned char value;
value = offset;
Write_Command(0xA2);
Write_Data(value);
}
void Fun_Selection(unsigned char selection)
{
unsigned char value;
value = selection;
Write_Command(0xAB);
Write_Data(value);
}

void Phase_Length(unsigned char length)
{
unsigned char value;
value = length;
Write_Command(0xB1);
Write_Data(value);
}

void Clock_Divider(unsigned char divider)
{
unsigned char value;
value = divider;
Write_Command(0xB3);
Write_Data(value);
}

void Display_En_A(unsigned char enhance_1, unsigned char enhance_2)
{
unsigned char value_1,value_2;
value_1 = enhance_1;
value_2 = enhance_2;
Write_Command(0xB4);
Write_Data(enhance_1);
Write_Data(enhance_2);
}

void Set_GPIO(unsigned char gpio)
{
unsigned char value;
value = gpio;
Write_Command(0xB5);
Write_Data(value);
}
void Sec_Pre_Period(unsigned char period)
{
unsigned char value;
value = period;
Write_Command(0xB6);
Write_Data(value);
}

void Default_Gray(unsigned char gray)

{
unsigned char value;
value = gray;
Write_Command(value);
}

void Pre_Voltage(unsigned char voltage)
{
unsigned char value;
value = voltage;
Write_Command(0xBB);
Write_Data(value);
}

void Set_Vcomh(unsigned char vcomh)
{
unsigned char value;
value = vcomh;
Write_Command(0xBE);
Write_Data(value);
}
void Set_Contrast(unsigned char contrast)
{
unsigned char value;
value = contrast;
Write_Command(0xC1);
Write_Data(value);
}

void Master_Contrast(unsigned char current)
{
unsigned char value;
value = current;
Write_Command(0xC7);
Write_Data(value);
}
void MUX_Ratio(unsigned char ratio)
{
unsigned char value;
value = ratio;
Write_Command(0xCA);
Write_Data(value);
}

void Display_En_B(unsigned char enhance_1, unsigned char enhance_2)
{
unsigned char value_1,value_2;
value_1 = enhance_1;
value_2 = enhance_2;
Write_Command(0xD1);
// (enhance_1);
Write_Data(enhance_2);
}
void Display_Mode(unsigned char mode)
{
unsigned char value;
value = mode;
Write_Command(value);
}
void Reset_IC()
{
_delay_ms(10);
CLR_RESET;
_delay_ms(1);
SET_RESET;
}

void fill(void)
{
  unsigned char x,y;
  unsigned char dat=0;
  
  Column_Address(0x1C,0x5B); //Set Column Address
  Row_Address(0x00,0x3F); //Set Row Address
  Write_Command(0x5c);
  for(y=0;y<64;y++) {
     dat=0;
     for(x=0;x<64;x++) {
        Write_Data(dat);
        Write_Data(dat);
        if((x%8==0)&&(x>0)) {
           dat+=0x22;
	}
     }
  }
}
void fill2(void)
{
unsigned char x,y;
unsigned char dat=0;
Column_Address(0x1C,0x5B); //Set Column Address
Row_Address(0x00,0x3F); //Set Row Address
Write_Command(0x5c);
for(y=0;y<64;y++)
{
dat=0xFF;
for(x=64;x>0;x--)
{
Write_Data(dat);
Write_Data(dat);
if((x%8==0)&&(x>0))
dat-=0x22;
}
}
}
`
