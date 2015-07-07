/* Memory regions for STM32F103RB, 128K flash, 20K RAM. */

MEMORY
{
    rom (rx) : ORIGIN = 0x08000000, LENGTH = 128K
    ram (rwx) : ORIGIN = 0x20000000, LENGTH = 20K
}

/* Include main opencm3 linker script */
INCLUDE libopencm3_stm32f1.ld
