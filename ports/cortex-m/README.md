# ARM Cortex-M Port

Author: Tido Klaassen <tido@4gh.eu>

License: BSD Revised. Needs libopencm3, which is LGPLv3.

## Summary and Prerequisites
This port should run on any Cortex-M0/3/4/4F (M0+ not tested). 
It uses RedHat's [newlib](https://sourceware.org/newlib/) and [libopencm3]
(https://github.com/libopencm3/libopencm3). You will also need an ARM compiler
toolchain. If you want to flash or debug your target, [openocd](http://openocd.org)
is also a must. For STM32 targets [stlink](https://github.com/texane/stlink)
might be helpful.

On Debian systems, compiler, newlib and openocd can easily be installed by the
package manager (untested, not going to set up a new system just to test this):

``` 
apt-get install gcc-arm-none-eabi binutils-arm-none-eabi
apt-get install libnewlib-arm-none-eabi libnewlib-dev
apt-get install openocd
```
**N.B.** Usually you will want to compile and link against the size optimised
"nano" version of newlib. This is done by default. If your version
of newlib does not support this (Debian's libnewlib package version before
2.1.0+git20141201.db59ff3-2) you will have to comment out the line
`USE_NANO := true` in the Makefile or pass `USE_NANO=` as
a command line option to make.

**N.B.** Debian's libnewlib-arm-none-eabi version 2.2.0+git20150830.5a3d536-1
ships with broken nano support. To enable necessary workarounds, uncomment
the line `#FIX_DEBIAN := true` in the Makefile or pass `FIX_DEBIAN=true`
as a command line option to make.
If you are using this fix, be advised that when switching between nano
and regular builds, you will have to do a `make realclean` first.

## Code Layout
The "classic" port components (code needed for task set-up and context
switching and the atomport{-private}.h headers) are residing in the
top level port directory.

There are additional subdirectories:

* **boards** contains subdirectories for specific hardware. Each
board needs at least a Makefile fragment, which defines certain variables 
describing the hardware used, as well as a list of extra object files needed.
There will usually be at least a `board_setup.c`, which contains code to 
initialise the hardware properly (clocks, UART, systick timer, GPIOs, etc.).

* **common** contains code needed by multiple boards, such as
stub functions for newlib or routines shared by multiple boards using the
same family of target MCUs.

* **linker** contains linker script fragments for specific target MCUs. These
just define the target's memory areas (RAM, Flash) and include libopencm3's
main linker script for the appropriate chip family.

* **libopencm3** unless you compile and link against an external installation
of libopencm3, this will be a Git submodule containing, surprise!, libopencm3.

* **build** this is a temporary directory where all object files end up in and
which will be removed by `make clean`.

## Build Preparation
Unless you decide to use an external installation of libopencm3, you will have
to set up the libopencm3 sub-module:
```
git submodule init
git submodule update
```

## Building and Flashing
To build the test suite, run
```
make BOARD=*yourboard* all
```
where *yourboard* is the name of a directory in **boards**. If no BOARD is
given, it defaults to nucleo-f103rb.

To build with an external libopencm3, run
```
make BOARD=*yourboard* OPENCM3_DIR=*/path/to/libopencm3* all
```

Instead of building the whole test suite you can also just build a specific
application image by giving its base name. If, for example, you only want
to build the `queue2` test app, you can do that like this:
```
make BOARD=*yourboard* queue2
```
 
If your board Makefile also sets up the necessary openocd variables, you can
use it to flash the application image by appending `.flash` to its base name.
```
make BOARD=*yourboard* *yourimage*.flash
```

N.B.: with the ek-lm4f120xl board openocd will report an error after flashing.
I think it is because it can not handle the changed core clock after the 
application starts, but I simply can't be bothered to further investigate this.
The application gets flashed and you can ignore the error.

## Adding New Boards
To add support for a new board, you will have to provide at least two parts. 
First, a `Makefile.include` to be pulled in by the main Makefile. Second,
some set-up code for your specific hardware. If there is no linker script for
your MCU, you will have to add one, too.

### Board Makefile Fragment
The main Makefile will include the Makefile.include located in the chosen
board's sub-directory. This is where you set compile time options and additional
source files to be used. Here is the one for the nucleo-f103rb:

```
TARGET          ?= stm32f103rb
LIBNAME         ?= opencm3_stm32f1
DEFS            ?= -DSTM32F1
DEFS            += -DSTD_CON=USART2
DEFS            += -DMST_SIZE=0x400

FP_FLAGS        ?= -msoft-float
ARCH_FLAGS      ?= -mthumb -mcpu=cortex-m3 $(FP_FLAGS) -mfix-cortex-m3-ldrd

OOCD            ?= openocd
OOCD_INTERFACE  ?= stlink-v2-1
OOCD_BOARD      ?= st_nucleo_f103rb

objs            += board_setup.o
objs            += stubs.o stm32_con.o
```

* **TARGET** is the name for the actual MCU used on your board. It will be used
to locate the linker script file in the `linker` directory by appending
`.ld` to it.

* **LIBNAME** is the name of the opencm3 library to link your object files
against.

* **DEFS** are flags that will be appended to the CPPFLAGS. You will at least
have to define the opencm3 target family (STM32F1 here), so the build process
will find the right header files to include. If you are using the stub and
console functions provided in the common directory, you will also have to
define the reserved main stack size (MST_SIZE) and which UART to use for stdio
(STD_CON).

* **FP_FLAGS** which floating point format to use. For MCUs without hardware
support for floating point (M0/3, sometimes 4), use `-msoft-float`,
otherwise use `-mfloat-abi=hard -mfpu=fpv4-sp-d16`. You could add these 
directly to `ARCH_FLAGS`, but this way it is easily overridden from the
make command line.

* **ARCH_FLAGS** specify the instruction (sub)set and CPU type to compile for,
as well as any quirk tunings needed and the `FP_FLAGS`. These flags are
handed to preprocessor, compiler, assembler and linker.

The following flags are only used for flashing the target with openocd:

* **OOCD** binary to call. Give full path if it is not in your PATH environment
variable. 

* **OOCD_INTERFACE** tells open which interface configuration to use

* **OOCD_BOARD** tells openocd which board configuration file to use.

* **objs** here you _append_ object files to include into _all_ binaries
built for this board. The main Makefile will search for matching source files
(*.c and *.S) in the main port directory, the board directory and the common
directory. You will usually have at least a `board_setup.c` specific to
your hardware and pull in the system and stdio stubs provided in the 
`common` directory.

* **aobjs** the build system will build one application for each object you 
add to this variable. It will do so by linking each of these objects with all
of the kernel-, port- and board-objects (but none of the testsuite-objects). 
The source for the app-object should be located in either the board or
common directory and must contain the application's `main()` function.
The `helloworld.c` file in the common directory is provided as an example.

As you will probably know, variables assigned with the `?=` operator are
only altered if they have not been already set. This way you can easily test
different options for a build by providing the variables on the make command
line.

### Board-Specific Set-up
All hardware needs to be initialised to some degree before it is ready to run
atomthread's kernel. At the very least you will have to

1. mask interrupts
2. set up the main clock and configure the systick timer
3. configure console UART  
4. configure GPIOs

(Steps 3. and 4. might be optional if you do not plan to let your board
interact with the outside world in any way...)

The test suite programs expect your board set-up code to provide a function
named `int board_setup(void)` to perform this tasks.

### Linker Script
If you are using an hitherto unknown target MCU, you will have to provide a
linker script for it in the `linker` directory. The script's name must
be the same as given to the `TARGET` variable in your Makefile.include,
with the extension `.ld` added. It is recommended that you just define
your MCU's memory regions and include libopencm3's linker file for your
target's product family. Please look at the provided linker files for examples
on how to do this.

## Port Internals
The Cortex-M port is different from the other architectures insofar as it makes
use of two particular features. First, it uses separate stacks for thread and
exception context. When the core enters exception mode, it first pushes xPSR,
PC, LR, r0-r3 and r12 on the currently active stack (probably the thread stack
in PSP) and then switches to the main stack stored in MSP. It also stores a
special EXC_RETURN code in LR which, when loaded into the PC, will determine
if on return program execution continues to use the MSP or switches over to 
the PSP and, on cores with an FPU, whether FPU registers need to be restored.
The Cortex-M also implements a nested vectored interrupt controller (NVIC),
which means that a running ISR may be pre-empted by an exception of higher
priority.

The use of separate stacks for thread and exception context has the nice
implication that you do not have to reserve space on every task's stack for 
possible use by ISRs. But it also means that it breaks atomthreads concept 
of simply swapping task stacks, regardless of if atomSched() was called from
thread or interrupt context. We would have to implement different 
archContextSwitch() functions called from thread or exception context and
also do messy stack manipulations depending on whether the task to be
scheduled in was scheduled out in in the same context or not. Yuck!
And don't get me started on nested exceptions calling atomIntExit()...   

This is where the second feature comes handy, the PendSV mechanism.
PendSV is an asynchronous exception with the lowest possible priority, which
means that, when triggered, it will be called by the NVIC if there are no
other exceptions pending or running. We use it by having archContextSwitch()
set up a pointer to the TCB that should be scheduled in and then trigger the
PendSv exception. As soon as program flow leaves the critical section or 
performs the outermost exception return, the pend_sv_handler() will be called
and the thread context switch takes place.
