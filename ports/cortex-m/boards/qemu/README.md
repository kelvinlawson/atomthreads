This board is a quick n' dirty copy of nucleo-f103rb, stripped down to run
atomthread's test suite on qemu. Since it must be linked against newlib's 
librdimon to enable semi-hosting, the stubs provided in the common directory
can not be used. Do not be surprised if malloc etc. do not work as expected.

The [GNU ARM Eclipse](http://gnuarmeclipse.livius.net/blog/) project maintains
a fork of qemu that is able to emulate a Nucleo-F103RB board. Check out their
gnuarmeclipse-dev branch from [here]
(http://sourceforge.net/p/gnuarmeclipse/qemu/ci/gnuarmeclipse-dev/tree/).

At time of writing (2015-07-13), I had to run configure with 
`--disable-werror --target-list="gnuarmeclipse-softmmu"` to build
a usable target.

After installing you can use it to run the test binaries like this:

```
qemu-system-gnuarmeclipse -nographic -monitor null -serial stdio \
      -semihosting --semihosting-config target=native \
      --machine NUCLEO-F103RB --kernel build/kern1.elf --verbose
```
