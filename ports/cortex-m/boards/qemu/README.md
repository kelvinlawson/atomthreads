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
qemu-system-gnuarmeclipse -nographic -monitor null \
      -semihosting --machine NUCLEO-F103RB \
      --verbose --kernel build/kern1.elf
```

The whole test suite can be run in an automatic way from the build system by
using the tool `expect`.

```
make BOARD=qemu QEMU=/path/to/your/qemu/binary qemutests
```

If your qemu-binary is called `qemu-system-gnuarmeclipse` and is located
in your search path, you can omit the `QEMU=...` part.
