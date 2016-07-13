---------------------------------------------------------------------------

 * Library: Atomthreads
 * Author: Kelvin Lawson <info@atomthreads.com>
 * Website: http://atomthreads.com
 * License: BSD Revised

---------------------------------------------------------------------------

Atomthreads is a free RTOS for embedded systems, released under the
flexible, open source BSD license and is free to use for commercial or
educational purposes without restriction.

It is targeted at systems that need only a lightweight scheduler and the
usual RTOS primitives. No file system, IP stack or device drivers are
included, but developers can bolt on their own as required. Atomthreads
will always be a small number of C files which are easy to port to any
platforms that require threading by adding a simple
architecture-specific file.

---------------------------------------------------------------------------

DOCUMENTATION:

All documentation is contained within the source files, commented using
Doxygen markup. Pre-generated documentation can be accessed at
http://atomthreads.com.

See also the README file contained within each folder of the source tree.

---------------------------------------------------------------------------

GETTING STARTED:

Building of the sources is carried out from the ports tree. For example to 
make a software build for the AVR architecture see ports/avr/README.

---------------------------------------------------------------------------

SOURCE TREE:

 * kernel: Core kernel sources
 * tests: Automated test suite
 * ports: CPU architecture ports

---------------------------------------------------------------------------

AUTOMATED TESTS STATE:
 * Continuous Integration hosted at travis-ci.org: [![Build Status](https://travis-ci.org/kelvinlawson/atomthreads.svg?branch=master)](https://travis-ci.org/kelvinlawson/atomthreads)
