---------------------------------------------------------------------------

Library:      Atomvn
Author:       Natie van Rooyen <natie@navaro.nl>
License:      BSD Revised

---------------------------------------------------------------------------

Atomvm is a tiny virtual machine that can run on Windows inside an IDE with a 
debugger like Microsoft Visual C++ Express. The primary purpose of this VM is 
for the evaluation of Real Time Operating Systems (like atomthreads) and 
the development and testing of modules for this Real Time Operating System
in a user friendly environment.

---------------------------------------------------------------------------

BUILDING THE SOURCE

To test this project, just add all the files from the “atomthreads/kernel” 
directory and the “atomthreads/ports/atomvm” directory as well as the test 
program “atomthreads/ports/atomvm/test/main.c” to your project. Add both the 
before mentioned directories to the include paths of your project and compile.

Atomvm was designed for multi core systems but also runs well on single core 
systems.

---------------------------------------------------------------------------

RUNNING THE TESTS

The test, main.c, was designed to stress the vm as opposed to testing the RTOS. 
However, the test can also run the unit tests if desired by using the 
precompiled directive UNIT_TESTS and linking in the desired unit test.

---------------------------------------------------------------------------

FINALLY

Good luck, but most of all, have fun!
