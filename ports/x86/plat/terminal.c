/*
 * AtomThreads project - Copyright (c) 2010, Kelvin Lawson. All rights reserved.
 * 
 * Please refer to the README file for further details and License information.
 *
 * x86 port by: ido@mm.st
 */

#include <stddef.h>
#include <stdint.h>
#include "string.h"
#include "plat.h"
#include "print.h"

#define PC_VGA_BUFFER 0xB8000

static const size_t VGA_WIDTH = 80;
static const size_t VGA_HEIGHT = 25;

static size_t terminal_row;
static size_t terminal_column;
static uint8_t terminal_color;
static uint16_t* terminal_buffer;

static uint8_t make_color(enum vga_color fg, enum vga_color bg) {
	return fg | bg << 4;
}

static uint16_t vga_entry(char c, uint8_t color) {
	uint16_t c16 = c;
	uint16_t color16 = color;
	return c16 | color16 << 8;
}
void terminal_init() {
	terminal_row = 0;
	terminal_column = 0;
	terminal_color = make_color(COLOR_LIGHT_RED, COLOR_WHITE);
	terminal_buffer = (uint16_t*) PC_VGA_BUFFER;
	for (size_t y = 0; y < VGA_HEIGHT; y++) {
		for (size_t x = 0; x < VGA_WIDTH; x++) {
			const size_t index = y * VGA_WIDTH + x;
			terminal_buffer[index] = vga_entry(' ', terminal_color);
		}
	}
	plat_hide_cursor();
}

static void scroll(void)
{
	memcpyw( terminal_buffer, terminal_buffer + VGA_WIDTH, (VGA_HEIGHT-1) * VGA_WIDTH *2);
	uint16_t c = vga_entry(' ', terminal_color);
        memsetw(terminal_buffer + (VGA_HEIGHT-1)*VGA_WIDTH , c , VGA_WIDTH); 	
	--terminal_row;
}

void terminal_putchar_color(char c, uint8_t text_color, uint8_t background_color) {
	
	uint8_t color = terminal_color;
	
	if(VGA_HEIGHT <= terminal_row)
		scroll();

	if( COLOR_DEFAULT != text_color && COLOR_DEFAULT != background_color)
		color = make_color(text_color, background_color);
			
	if('\n' == c) {
		terminal_column = 0;
		++terminal_row;
	}else{
		const size_t index = terminal_row * VGA_WIDTH + terminal_column;
		terminal_buffer[index] = vga_entry(c, color);
	}

	if (++terminal_column == VGA_WIDTH) {
		terminal_column = 0;
		if (++terminal_row >= VGA_HEIGHT) 
			terminal_row = 0;
	}
}

void terminal_putchar(char c){
	terminal_putchar_color(c,COLOR_DEFAULT, COLOR_DEFAULT);
}	
