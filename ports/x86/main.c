/*
 * AtomThreads project - Copyright (c) 2010, Kelvin Lawson. All rights reserved.
 * 
 * Please refer to the README file for further details and License information.
 *
 * x86 port by: ido@mm.st
 */

#include "atom.h"
#include "atomtests.h"
#include "atomport.h"
#include "x86.h"
#include "plat.h"
#include "print.h"

#define APP_PRIORITY 16

#define IDLE_STACK_SIZE_BYTES 1024*16
#define APP_STACK_SIZE 1024*16 

static uint8_t idle_thread_stack[IDLE_STACK_SIZE_BYTES];
static uint8_t app_stack[APP_STACK_SIZE];

static ATOM_TCB app_tcb;

int x86_pc_init(void)
{
    	gdt_install_flat();
	print("GDT installed\n");

	setup_idt();
	print("IDT installed\n");

	pit_init(SYSTEM_TICKS_PER_SEC);
	print("i8253 (PIT) initialized @%d hz\n", SYSTEM_TICKS_PER_SEC);

    	pic_init();
	print("i8259 (PIC) initialized\n");	
	
	irq_register_handler(0, sys_tick_handler);
	irq_register_handler(1, sys_key_handler);
	
	return ATOM_OK;
}

static void app_entry(uint32_t data);

void kernel_main (void)
{
    terminal_init();

    print("kernel_main()\n");

    if(ATOM_OK != x86_pc_init()) goto failure;
    if(ATOM_OK != atomOSInit(&idle_thread_stack[0], IDLE_STACK_SIZE_BYTES, TRUE)) goto failure;
    if(ATOM_OK != atomThreadCreate(
	           &app_tcb, 
		   APP_PRIORITY, 
		   app_entry, 
		   1,
                   &app_stack[0],
                   APP_STACK_SIZE,
                   TRUE)) goto failure;

    atomOSStart();

failure:  

    print("[FAILURE]\n");
    x86_halt();
}

static void app_entry (uint32_t data)
{
    uint32_t test_status;

    print("\nstarting test..\n");

    test_status = test_start();

    /* Check main thread stack usage (if enabled) */
#ifdef ATOM_STACK_CHECKING
    if (test_status == 0)
    {
        uint32_t used_bytes, free_bytes;

        /* Check idle thread stack usage */
        if (atomThreadStackCheck (&main_tcb, &used_bytes, &free_bytes) == ATOM_OK)
        {
            /* Check the thread did not use up to the end of stack */
            if (free_bytes == 0)
            {
                print ("Main stack overflow\n");
                test_status++;
            }

            /* Log the stack usage */
#ifdef TESTS_LOG_STACK_USAGE
            print ("MainUse:%d\n", (int)used_bytes);
#endif
        }

    }
#endif

    /* Log final status */
    if (test_status == 0)
    {
	print_color("\n Test Pass!\n", COLOR_WHITE, COLOR_GREEN);
    }
    else
    {
        print_color("\nTets Fail\n", COLOR_WHITE, COLOR_RED );
	kprintf("Total Failures: %d\n", test_status);
    }

    print("\n\nPress Q to reboot\n");
}
