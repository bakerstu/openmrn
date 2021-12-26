# Using distcc with OpenMRN compilation framework

## Overview

### What is this?

This feature allows using a powerful remote computer to perform most of the
compilation steps when compiling OpenMRN-based projects.

### Why?

It is much faster.

OpenMRN compiles a large number of c++ files that can take a lot of CPU to
compile. If your workstation has constrained CPU (e.g. a laptop or a virtual
machine with limited number of CPUs), you can use a powerful remote machine to
offload most of the compilation steps.

### How does it work?

We use the opensource distcc package (git@github.com:distcc/distcc.git). We
create an SSH link to the remote machine, and start the distcc daemon. We wrap
the local compilation command in distcc. Distcc will preprocess the source file
locally, then transmit the preprocessed source to the remote machine over the
SSH link. The daemon running on the powerful machine will then invoke the
compiler, create the .o file, and transmit the .o file back over the SSH
link. The linking steps then happen locally.

## Prerequisites

- You have to have the same compiler that OpenMRN uses available on the remote
  machine. We have a [specific step](#configuring-the-compilers) for telling
  distcc and OpenMRN what the matching compilers are.
- You need to start the SSH link before trying to compile things. If you forget
  this, local compilation will be performed instead.
- A small change needs to be made to the distcc sources, because it does not
  support a compiler flag that we routinely pass to armgcc. For this reason you
  need to compile it from source.

### Installing

#### Installing distcc

1. `sudo apt-get install distcc` on both the local and remote machine.
2. Clone the distcc sources from git@github.com:distcc/distcc.git. Check the
   INSTALL file for an apt-get commandline near the very top for compile-time
   dependencies to be installed.
3. edit src/arg.c. Find the place where it checks for "-specs="
   argument. Comment out the early return. (Not doing anything in that if is
   what you want.)
4. edit src/serve.c. Find where it checks for "-specs=", and disable that
   check.
5. run through the compile steps of distcc, see the INSTALL file. These are
   normally:
   ```
   ./autogen.sh
   ./configure
   make
   ```
6. Copy `distcc` to `~/bin`, and copy `distccd` to the remote machine at `~/bin`

#### Configuring the compilers

When setting up compilers, we need to ensure that we can determine what the
remote compiler will be called for any given local compiler that we have. In
addition to this, we need to be able to call the same compiler underthe same
command on both machines, because the local machine will be calling the
compiler for preprocessing and the remote machine will be calling it for
compilation.

For any given compiler, you need to make a symlink on both the local machine
and the remote machine:

```bash
cd ~/bin
ln -sf $(realpath /opt/armgcc/gcc-arm-none-eabi-8-2018-q4-major/bin/arm-none-eabi-gcc) armgcc-2018-q4-gcc
ln -sf $(realpath /opt/armgcc/gcc-arm-none-eabi-8-2018-q4-major/bin/arm-none-eabi-g++) armgcc-2018-q4-g++
```

Do this on BOTH the local and remote machine. The real path may differ but the
name must be exactly the same.

Only on the remote machine, the compiler also needs to be added to the
masquerade directory, otherwise distccd server will refuse to execute it.

```bash
cd /usr/lib/distccd
sudo ln -sf ../../bin/distcc armgcc-2018-q4-g++
sudo ln -sf ../../bin/distcc armgcc-2018-q4-gcc
```

#### Setting parameters

For the OpenMRN compilation scripts to find distcc and the remote machine, we
store a few parameters in files under `~/.distcc`.

```bash
mkdir -p ~/.distcc
echo 3434 > ~/.distcc/port
echo my-fast-remote-machine.myplace.com > ~/.distcc/ssh_hosts
echo '127.0.0.1:3434/20,lzo' > ~/.distcc/hosts
```

The port number will be used with SSH port forwarding. The remote machine is
used by the ssh tunnel setup script.

The "/20" part of the hosts line is the capacity of the remote machine. This
one says it's good for 20 parallel jobs, which I picked for a 6-core (12
hyperthread) machine with lots of RAM.

You can configure a lot of other things in the hosts file. `man distcc` for the
full documentation. It is also possible to configure the local CPU to run some
number of jobs while the remote CPU runs other builds. It is possible to use
multiple remote machines as well.

## Using

After every restart of your machine you need to start the ssh tunnel:

```bash
~/openmrn/bin/start-distcc.sh
```

This will set up the SSH tunnel to the remote host and start the distccd server
on the far end of the tunnel.

Then compile OpenMRN stuff with very high parallelism:

```bash
~/openmrn/applications/io_board/target/freertos.armv7m.ek-tm4c123gxl$ make -j21
```

### How do I know if it worked?

OpenMRN build system will automatically detect that you have the SSH tunnel
running to distcc and use distcc.

If it uses distcc, then you will see the compilation done remotely. Check the
beginning of most lines:

```
distcc armgcc-2018-q4-gcc -c -Os -fno-strict-aliasing [...] STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_usart.c -o stm32l4xx_hal_usart.o
```

If it is done locally, then you don't see distcc but see the execution of the
compiler directly:

```
/opt/armgcc/gcc-arm-none-eabi-8-2018-q4-major/bin/arm-none-eabi-gcc -c -Os -fno-strict-aliasing [...] STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_usart.c -o stm32l4xx_hal_usart.o
```

### How much does it help?

With distcc (parallelism of 21, all remotely; this is on a W-2135 Xeon CPU @
3.70GHz):

```bash
   text	   data	    bss	    dec	    hex	filename
 113808	    312	   3768	 117888	  1cc80	io_board.elf
/opt/armgcc/default/bin/arm-none-eabi-objdump -C -d -h io_board.elf > io_board.lst

real    0m44.704s
user    2m41.551s
sys     0m29.235s
```

Without distcc (parallelism of 9, all locally; this is on an intel i7-8665U CPU
@ 1.90GHz -- this is a 15W TDP mobile CPU with 4 cores, 8 hyperthreads):

```bash
   text	   data	    bss	    dec	    hex	filename
 113808	    312	   3768	 117888	  1cc80	io_board.elf
/opt/armgcc/default/bin/arm-none-eabi-objdump -C -d -h io_board.elf > io_board.lst

real    2m8.602s
user    14m21.471s
sys     0m42.488s
```
