/* main.c : CAN
 * Tue May  9 20:58:59 2017	Warren W. Gay VE3WWG
 * Uses CAN on PB8/PB9, UART1 115200,"8N1","rw",1,1
 *
 * GPIO:
 * ----
 * TX	A9  ====> RX of TTL serial adapter
 * RX	A10 <==== TX of TTL serial adapter
 * CTS	A11 ====> RTS of TTL serial
 * RTS	A12 <==== CTS of TTL serial
 * PB8	CAN_RX (NOTE: Differs from front/rear.c)
 * PB9	CAN_TX (NOTE: Differs from front/rear.c)
 */
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <ctype.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/spi.h>

#include "FreeRTOS.h"
#include "semphr.h"
#include "mcuio.h"
#include "miniprintf.h"
#include "canmsgs.h"
//#include "monitor.h"

#include "ugui.h"
#include "oled_drv.h"
#include "oled.h"

// indent -gnu -br -cli2 -lp -nut -l100 main.c

#define FLASH_MS		400     // Signal flash time in ms
#define GPIO_PORT_LED		GPIOC   // Builtin LED port
#define GPIO_LED		GPIO12  // Builtin LED

static SemaphoreHandle_t mutex; // Handle to mutex
struct s_lamp_status lamp_status = { 0, 0, 0, 0, 0, 0 };

static volatile bool show_rx = true;    // false;

//int my_spi = SPI1;

void show_console (void);
int rot_position = 37;

/*********************************************************************
 * CAN Receive Callback
 *********************************************************************/
void
can_recv (struct s_canmsg *msg)
{

  if (show_rx) {
    std_printf ("[%4u(%d/%u):%c,$%02X]\n",
                (unsigned) msg->msgid,
                msg->fifo, (unsigned) msg->fmi, msg->rtrf ? 'R' : 'D', msg->data[0]);
  }

  if (!msg->rtrf) {
    switch (msg->msgid) {
      case ID_Temp:
        break;
      default:
        break;
    }
  }
}

/*********************************************************************
 * Signal Flash task
 ************************************************************#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/spi.h>
*********/

static void
flash_task (void *arg __attribute__ ((unused)))
{
  for (;;) {
    gpio_toggle (GPIOC, GPIO12);
    vTaskDelay (pdMS_TO_TICKS (100));
  }
}

static void
oled_task (void *arg __attribute__ ((unused)))
{
  char buf[20];
  static int cntr = 0;

  oled_init ();
  oled_init_real ();

//      UG_FontSelect(&FONT_8X12);
//      UG_FontSelect(&FONT_12X16);
  UG_FontSelect (&FONT_8X14);
  UG_SetBackcolor (C_BLACK);
  UG_SetForecolor (C_WHITE);

  for (;;) {
    UG_FillScreen (C_BLACK);
    mini_snprintf (buf, sizeof buf, "Trim1: %d", rot_position++);
    UG_PutString (0, 0, buf);
    mini_snprintf (buf, sizeof buf, "Trim2: %d", rot_position++);
    UG_PutString (0, 15, buf);
    mini_snprintf (buf, sizeof buf, "Trim3: %d", rot_position++);
    UG_PutString (0, 30, buf);
    mini_snprintf (buf, sizeof buf, "Trim4: %d", cntr++);
    UG_PutString (0, 45, buf);
    oled_update ();
    vTaskDelay (500);
  }
}


/*********************************************************************
 * Console:
 *********************************************************************/
static void
console_task (void *arg __attribute__ ((unused)))
{
  static bool lockedf, flashf = false;
  struct s_lamp_en msg;
  char ch;

  xSemaphoreTake (mutex, portMAX_DELAY);        // Initialize this as locked
  lockedf = true;

  std_printf ("Car simulation begun.\n");
//      show_menu();
  std_printf ("CAN Console Ready:\n");


  for (;;) {
    std_printf ("> ");
    ch = std_getc ();
    std_printf ("%c\n", ch);

    switch (ch) {
      case 'F':
      case 'f':
        msg.enable = false;     // Not used here
        msg.reserved = 0;
        can_xmit (ID_Flash, false, false, sizeof msg, &msg);
        break;
      case 'L':
      case 'l':
        lamp_status.left = ch == 'L';
//                      lamp_enable(ID_LeftEn,lamp_status.left);
        flashf = true;
        break;
      case 'R':
      case 'r':
        lamp_status.right = ch == 'R';
//                      lamp_enable(ID_RightEn,lamp_status.right);
        flashf = true;
        break;
      case 'P':
      case 'p':
        lamp_status.park = ch == 'P';
//                      lamp_enable(ID_ParkEn,lamp_status.park);
        break;
      case 'b':
      case 'B':
        lamp_status.brake = ch == 'B';
//                      lamp_enable(ID_BrakeEn,lamp_status.brake);
        break;
      case 'V':
      case 'v':
        // Toggle show messages received (verbose mode)
        show_rx ^= true;
        break;
      case '\r':
        break;
//              default:
    }


    if (flashf) {
      if ((lamp_status.left || lamp_status.right) && lockedf) {
        xSemaphoreGive (mutex); // Start flasher
        lockedf = false;
      }
      else if (!lockedf && !lamp_status.left && !lamp_status.right) {
        xSemaphoreTake (mutex, portMAX_DELAY);  // Stop flasher
        lockedf = true;
      }
      flashf = false;
    }
  }
}

/*********************************************************************
 * Main program: Device initialization etc.
 *********************************************************************/
int
main (void)
{
  rcc_clock_setup_in_hse_8mhz_out_72mhz ();     // Use this for "blue pill"

  rcc_periph_clock_enable (RCC_GPIOC);
  gpio_set_mode (GPIO_PORT_LED, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO_LED);

  rcc_periph_clock_enable (RCC_GPIOA);
  rcc_periph_clock_enable (RCC_GPIOB);
  rcc_periph_clock_enable (RCC_AFIO);

  gpio_set_mode (GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART2_TX);
  std_set_device (mcu_uart2);   // Use UART1 for std I/O
  open_uart (2, 115200, "8N1", "rw", 0, 0);

  /* setup the OLED SPi PORT */
//  spi_1_init();
  spi_2_init();

  initialize_can (false, true, true);   // !nart, locked, altcfg=true PB8/PB9

  xTaskCreate (console_task, "console", 200, NULL, configMAX_PRIORITIES - 1, NULL);

  mutex = xSemaphoreCreateMutex ();
  xTaskCreate (flash_task, "flash", 100, NULL, configMAX_PRIORITIES - 1, NULL);

  xTaskCreate (oled_task, "oled", 300, NULL, configMAX_PRIORITIES - 1, NULL);

  vTaskStartScheduler ();
  for (;;);
}

// End main.c
