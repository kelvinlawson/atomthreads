/*
 * AtomThreads project - Copyright (c) 2010, Kelvin Lawson. All rights reserved.
 * 
 * Please refer to the README file for further details and License information.
 *
 * x86 port by: ido@mm.st
 */

ENTRY(_start)

SECTIONS
{
	/* load us at 1 MiB */ 
	. = 1M;

	.text BLOCK(4K) : ALIGN(4K)
	{
		*(.multiboot)
		*(.text)
	}

	/* RO data. */
	.rodata BLOCK(4K) : ALIGN(4K)
	{
		*(.rodata)
	}

	/* RW (initialized) */
	.data BLOCK(4K) : ALIGN(4K)
	{
		*(.data)
	}

	/* RW (uninitialized) data and stack */
	.bss BLOCK(4K) : ALIGN(4K)
	{
		*(COMMON)
		*(.bss)
	}
}
