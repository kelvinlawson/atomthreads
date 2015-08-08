/* Memory regions for TI Stellaris EX-LM4F120XL evaluation board. */

MEMORY
{
    rom (rx) : ORIGIN = 0x00000000, LENGTH = 256K
    ram (rwx) : ORIGIN = 0x20000000, LENGTH = 32K
}

/* Include main opencm3 linker script */
INCLUDE libopencm3_lm4f.ld
