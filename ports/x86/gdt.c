/*
 * AtomThreads project - Copyright (c) 2010, Kelvin Lawson. All rights reserved.
 * 
 * Please refer to the README file for further details and License information.
 *
 * x86 port by: ido@mm.st
 * */

struct gdt_entry
{
    unsigned short limit_low;
    unsigned short base_low;
    unsigned char base_middle;
    unsigned char access;
    unsigned char granularity;
    unsigned char base_high;
} __attribute__((packed));

struct gdt_ptr
{
    unsigned short limit;
    unsigned int base;
} __attribute__((packed));

struct gdt_entry gdt[3];
struct gdt_ptr gdt_p;

extern void gdt_flush();

void init_gdt_entry(int index, unsigned long base, unsigned long limit, unsigned char access, unsigned char granularity)
{
    gdt[index].base_low = (base & 0xFFFF);
    gdt[index].base_middle = (base >> 16) & 0xFF;
    gdt[index].base_high = (base >> 24) & 0xFF;
    gdt[index].limit_low = (limit & 0xFFFF);
    gdt[index].granularity = ((limit >> 16) & 0x0F);
    gdt[index].granularity |= (granularity & 0xF0);
    gdt[index].access = access;
}

void gdt_install_flat()
{
    gdt_p.limit = (sizeof(struct gdt_entry) * 3) - 1;
    gdt_p.base = (unsigned int) gdt;

    init_gdt_entry(0, 0, 0, 0, 0); // Null
    init_gdt_entry(1, 0, 0xFFFFFFFF, 0x9A, 0xCF); // Code 
    init_gdt_entry(2, 0, 0xFFFFFFFF, 0x92, 0xCF); // Data
    gdt_flush();    
}
