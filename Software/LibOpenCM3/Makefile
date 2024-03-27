######################################################################
#  Project Makefile
######################################################################

BINARY		= main
# SRCFILES	= main.c ugui.c oled_drv.c canmsgs.c rtos/heap_4.c rtos/list.c rtos/port.c rtos/queue.c rtos/tasks.c rtos/opencm3.c
SRCFILES	= main.c gcvt_2.c ugui.c oled_drv.c led_drv.c mcp23017.c i2c.c\
                  at42qt1070.c cmd_line.c cmdproc.c mcp9808.c \
		  hex_dump.c flash_block_rd_wrt.c modbus_drv.c\
		  rtos/port.c rtos/queue.c rtos/tasks.c rtos/opencm3.c rtos/heap_4.c rtos/list.c
#		  rtos/port.c rtos/queue.c rtos/tasks.c rtos/opencm3.c rtos/heap_4.c rtos/list.c
LDSCRIPT	= stm32f103c8t6.ld

# DEPS		= 	# Any additional dependencies for your build
# CLOBBER	+= 	# Any additional files to be removed with "make clobber"

include ../../Makefile.incl
include ../Makefile.rtos

.PSEUDO: all front

all: elf front

front:
	$(MAKE) -f Makefile.front 

front: Makefile.front

######################################################################
#  NOTES:
#	1. remove any modules you don't need from SRCFILES
#	2. "make clean" will remove *.o etc., but leaves *.elf, *.bin
#	3. "make clobber" will "clean" and remove *.elf, *.bin etc.
#	4. "make flash" will perform:
#	   st-flash write main.bin 0x8000000
######################################################################
