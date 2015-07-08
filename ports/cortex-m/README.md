# ARM Cortex-M Port

## Summary and Prerequisites
This port should run on any Cortex-M3/4/4F (but not on M0, sorry). It uses
RedHat's [newlib](https://sourceware.org/newlib/) and [libopencm3]
(https://github.com/libopencm3/libopencm3). You will also need an ARM compiler
toolchain. If you want to flash or debug your target, [openocd](http://openocd.org)
 is also a must. For STM32 targets [stlink](https://github.com/texane/stlink)
might also be helpful.

On Debian systems, compiler, newlib and openocd can easily be installed by the
package manager (untested, not going to set up a new system just to test this):

``` 
apt-get install gcc-arm-none-eabi binutils-arm-none-eabi
apt-get install libnewlib-arm-none-eabi libnewlib-dev
apt-get install openocd
```

## Code Layout
The "classic" port components (code needed for task setup and context
switching and the atomport{-private}.h headers) are residing in the
top level port directory.

There are additional subdirectories:

* **boards** contains subdirectories for specific hardware. Each
board needs at least a Makefile, which defines certain variables describing
the hardware used, as well as a list of extra object files needed. There will
usually be at least a board_setup.c, which contains code to initialise the
hardware properly (clocks, uart, systick timer)

* **common** contains code needed by multiple boards, such as
stub functions for newlib or routines shared by multiple boards using the
same family of target MCUs.

* **linker** contains linker script fragments for specific target MCUs. These
just define the memory areas (RAM, Flash) of the chip and include libopencm3's
main linker script for the appropriate chip family

* **libopencm3** unless you compile and link against an external installation
of libopencm3, this will be a Git submodule containing, surprise!, libopencm3

* **build** this is a temporary directory where all object files end up in and
which will be removed by ``make clean``  

## Build Preparation
Unless you decide to use an external installation of libopencm3, you will have
to set up the libopencm3 sub-module:
```
git submodule add https://github.com/libopencm3/libopencm3.git
git submodule init
git submodule update
```
Optional: As of 2015-07-08 the libopencm3 API has not been declared stable. If
future changes break the build, you can check out the older revision used while
developing this port:
```
cd libopencm3
git checkout a4bb8f7e240c9f238384cf86d009002ba42a25ed
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
If your board Makefile also set up the necessary openocd variables, you can
use it to flash the application image:
```
make BOARD=*yourboard* BINARY=*yourimage* flash
```
N.B.: with the ek-lm4f120xl board openocd will report an error after flashing.
I think it is because it can not handle the changed core clock after the 
application starts, but I simply can't be bothered to further investigate this.
The application gets flashed and you can ignore the error.

## Adding New Boards
*TODO*

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
which means that a running ISR may be preempted by an exception of higher
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
