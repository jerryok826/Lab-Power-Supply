void
oled_init_ssd1322 (void) // SSD1322 256X64
{

  gpio_clear (GPIOC, GPIO12);
  oled_reset ();
  sleep_us(200);           //wait minimum 200μs before sending commands
//  oled_command (cmds[ux]);
  oled_command2(0xFD, 0x12)        # Unlock IC
  oled_command(0xA4)	      # Display off (all pixels off)
  oled_command2(0xB3, 0xF2)        # Display divide clockratio/freq
  oled_command2(0xCA, 0x3F)        # Set MUX ratio
  oled_command2(0xA2, 0x00)        # Display offset
  oled_command2(0xA1, 0x00)        # Display start Line
  oled_command3(0xA0, 0x14, 0x11)  # Set remap & dual COM Line
  oled_command2(0xB5, 0x00)        # Set GPIO (disabled)
  oled_command2(0xAB, 0x01)        # Function select (internal Vdd)
  oled_command3(0xB4, 0xA0, 0xFD)  # Display enhancement A (External VSL)
  oled_command2(0xC7, 0x0F)        # Master contrast (reset)
  oled_command(0xB9)	      # Set default greyscale table
  oled_command2(0xB1, 0xF0)        # Phase length
  oled_command3(0xD1, 0x82, 0x20)  # Display enhancement B (reset)
  oled_command2(0xBB, 0x0D)        # Pre-charge voltage
  oled_command2(0xB6, 0x08)        # 2nd precharge period
  oled_command2(0xBE, 0x00)        # Set VcomH
  oled_command(0xA6)	      # Normal display (reset)
  oled_command(0xA9)	      # Exit partial display

  oled_command(0x7F)             # Reset
  gpio_set (GPIOC, GPIO12);
}
