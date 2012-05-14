---------------------------------------------------------------------------

Library:      Atomvn
Author:       Natie van Rooyen <natie@navaro.nl>
License:      BSD Revised

---------------------------------------------------------------------------

Atomvm is a tiny virtual machine that can run on Windows inside an IDE with a 
debugger like Microsoft Visual C++ Express. The primary purpose of this virtual 
machine is for the evaluation of Real Time Operating Systems (like atomthreads) 
and the development and testing of modules for this Real Time Operating System
in a user friendly environment.

---------------------------------------------------------------------------

BUILDING THE SOURCE

To test this project, just add all the files from the "atomthreads/kernel" 
directory and the "atomthreads/ports/atomvm" directory as well as the test 
program "atomthreads/ports/atomvm/test/main.c" to your project. Add both the 
before mentioned directories to the include paths of your project and compile.

Atomvm was designed for multi core systems but also runs fine on any single 
core system.

---------------------------------------------------------------------------

RUNNING THE TESTS

The test, main.c, is intentioned to stress the virtual  machine as opposed to 
testing the Real Time Operating System. However, this test can also run the 
unit tests of atomthreads by using the preprocessor directive "UNIT_TESTS" and 
linking in the desired unit test into the project.


