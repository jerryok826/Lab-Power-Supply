/* flash_block_rd_wrt.c : Lab Power Supply 
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <ctype.h>
#include <stdint.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/flash.h>

#include "main.h"
#include "mcuio.h"     // for std_printf

#define FLASH_OPERATION_ADDRESS ((uint32_t)0x0800f000)
#define FLASH_PAGE_NUM_MAX 127
#define FLASH_PAGE_SIZE 0x800
#define FLASH_WRONG_DATA_WRITTEN 0x80
#define FLASH_BLOCK_SIZE 256

// indent -gnu -br -cli2 -lp -nut -l100 flash_block_rd_wrt.c

// char local_buf[]={"012345667890 hello"__DATE__ __TIME__};

int
flash_bytes_used (void)
{
#define FLASH_BASE_ADDRESS ((uint32_t)0x08000000)
#define FLASH_OPERATION_ADDRESS ((uint32_t)0x0800f000)

  uint32_t *ptr = (uint32_t *) FLASH_OPERATION_ADDRESS;
  // FLASH_BASE_ADDRESS

  while (*ptr == 0xFFFFFFFF) {
    ptr -= 4;
  }

  return (ptr - ((uint32_t *) FLASH_BASE_ADDRESS));
}

// read a 256 byte block of flash
void
flash_block_rd (uint8_t * output_data)
{
  uint32_t *memory_ptr = (uint32_t *) FLASH_OPERATION_ADDRESS;
  uint16_t iter;

  for (iter = 0; iter < FLASH_BLOCK_SIZE / 4; iter++) {
    *(uint32_t *) output_data = *(memory_ptr + iter);
    output_data += 4;
  }
}

int
flash_block_wrt (uint8_t * input_data)
{
  uint16_t iter;
  uint32_t current_address = FLASH_OPERATION_ADDRESS;
  uint32_t page_address = FLASH_OPERATION_ADDRESS;
  uint32_t flash_status = 0;
  uint16_t num_elements = FLASH_BLOCK_SIZE / 4;

  /* calculate page address */
  page_address -= (FLASH_OPERATION_ADDRESS % FLASH_PAGE_SIZE);

  flash_unlock ();

  /*Erasing page */
  flash_erase_page (page_address);
  flash_status = flash_get_status_flags ();
  if (flash_status != FLASH_SR_EOP) {
    return flash_status;
  }

  /*programming flash memory */
  for (iter = 0; iter < num_elements; iter += 4) {
    /*programming word data */
    flash_program_word (current_address + iter, *((uint32_t *) (input_data + iter)));
    flash_status = flash_get_status_flags ();
    if (flash_status != FLASH_SR_EOP) {
      return flash_status;
    }

    /*verify if correct data is programmed */
    if (*((uint32_t *) (current_address + iter)) != *((uint32_t *) (input_data + iter))) {
      return FLASH_WRONG_DATA_WRITTEN;
    }
  }
  return flash_status;
}
