// From /home/jerryo/stm32f103/stm32f103c8t6/rtos/can_enc/main.c

uint32_t get_mac_adr (void);
int flash_rd_int32(int indx);
int flash_wr_int32(int indx, uint32_t val32);
int flash_bytes_used(void);

uint32_t
get_mac_adr (void)
{
  uint32_t my_mac;
  my_mac = *((unsigned int *) 0x1FFFF7F0);
  return my_mac;
}

  std_printf ("Car simulation begun.\n");
  std_printf ("My MAC: 0x%08lX\n", get_mac_adr ());
  std_printf ("EEPROM: 0x%08lX\n", flash_rd_int32(0));
  bytes = flash_bytes_used();
  std_printf ("Bytes used: %d 0x%08lX\n", bytes, bytes);
    
//  rc = flash_wr_int32(0, 0x7777);
//  std_printf ("EEPROM: wrt rc: %d\n", rc);
  vTaskDelay (3000);  


#define FLASH_BASE_ADDRESS ((uint32_t)0x08000000)
#define FLASH_OPERATION_ADDRESS ((uint32_t)0x0800f000)
#define FLASH_PAGE_NUM_MAX 127
#define FLASH_PAGE_SIZE 0x800   // 2048 page size
#define FLASH_WRONG_DATA_WRITTEN 0x80

// result = flash_program_data(FLASH_OPERATION_ADDRESS, str_send, SEND_BUFFER_SIZE);
// flash_read_data(FLASH_OPERATION_ADDRESS, SEND_BUFFER_SIZE, str_verify);


static uint32_t
flash_program_data (uint32_t start_address, uint8_t * input_data, uint16_t num_elements)
{
  uint16_t iter;
  uint32_t current_address = start_address;
  uint32_t page_address = start_address;
  uint32_t flash_status = 0;

  /*check if start_address is in proper range */
  if ((start_address - FLASH_BASE) >= (FLASH_PAGE_SIZE * (FLASH_PAGE_NUM_MAX + 1))) {
    return 1;
  }

  /*calculate current page address */
  if (start_address % FLASH_PAGE_SIZE) {
    page_address -= (start_address % FLASH_PAGE_SIZE);
  }

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

  return 0;
}

static void
flash_read_data (uint32_t start_address, uint16_t num_elements, uint8_t * output_data)
{
  uint16_t iter;
  uint32_t *memory_ptr = (uint32_t *) start_address;

  for (iter = 0; iter < num_elements / 4; iter++) {
    *(uint32_t *) output_data = *(memory_ptr + iter);
    output_data += 4;
  }
}

int flash_rd_int32(int indx)
{
     uint32_t *current_address = (FLASH_OPERATION_ADDRESS) + indx*4;
     return *current_address;
}

int flash_wr_int32(int indx, uint32_t val32)
{
  uint32_t flash_status = 0; 

  flash_unlock();  
 
  /* Erasing page */
  flash_erase_page (FLASH_OPERATION_ADDRESS);
  flash_status = flash_get_status_flags ();
  if (flash_status != FLASH_SR_EOP) {
      return flash_status;
  }
  
  /* programming word data */
  flash_program_word ((FLASH_OPERATION_ADDRESS) + indx*4, val32);
  flash_status = flash_get_status_flags ();
  if (flash_status != FLASH_SR_EOP) {
    return -1;
  }  
  
  flash_lock();
  
  /* verify if correct data is programmed */
  if (*((uint32_t *) (FLASH_OPERATION_ADDRESS + indx*4)) != val32) {
     return -2;
  }

  return 0;
}

int flash_bytes_used(void)
{
 int i=0;
 uint32_t *ptr = (uint32_t *)FLASH_OPERATION_ADDRESS;
 // FLASH_BASE_ADDRESS
 
 ptr--; // skip EEPROM section
 while ((*ptr) == 0xFFFFFFFF) {
     ptr--;;
 }
 
return ((ptr - ((uint32_t *)FLASH_BASE_ADDRESS))*4);
// return (uint32_t)ptr;
}

int flash_bytes_used2(void)
{
 int i=0;
 uint32_t *ptr = (uint32_t *)FLASH_OPERATION_ADDRESS;
 // FLASH_BASE_ADDRESS
 
 while (*ptr == 0xFFFFFFFF) {
     ptr -= 4;
 }
 
 return (ptr - ((uint32_t *)FLASH_BASE_ADDRESS));
}

