/* Memory regions for STM32F072RB, 128K flash, 16K RAM. */

MEMORY
{
    rom (rx) : ORIGIN = 0x08000000, LENGTH = 128K
    ram (rwx) : ORIGIN = 0x20000000, LENGTH = 16K
}

/* Include main opencm3 linker script */
INCLUDE libopencm3_stm32f0.ld
