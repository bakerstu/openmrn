# OpenMRN

OpenMRN (Open Model Railroad Network) is a set of software libraries that are
designed to make it easier to implement support for the NMRA's LCC (Layout
Command Control) bus.

The work to define LCC was done by the [OpenLCB group](http://openlcb.org/),
and that's a good place to go in order to learn more about LCC. In particular,
[Introduction to OpenLCB](http://openlcb.org/about-openlcb/introduction-to-openlcb/).
Whenever you see _OpenLCB_, just think of this as a public working group that is
creating standards that, once approved by the NMRA, become LCC standards. The 
[OpenLCB & LCC Standards](http://openlcb.org/openlcb-and-lcc-documents/layout-command-control-lcc/)
page will give you a good idea of status of the specifications.

## Why OpenMRN

OpenMRN is a set of C++ code that is designed to make it easier to implement
support for LCC. This might be in accessory decoders, in a command station, in a
throttle, or any other device. The code is designed to be able to run on micro
controllers. There are currently a number of different 32-bit micro controllers
supported. To find the full list, you'll need to check the boards folder.

**Note:** The software license terms are 2-clause BSD in order to be commercial
friendly. This allows commercial applications to be written that use the open
source OpenMRN libraries without having to be open source themselves.

# Getting Started

Most of the documentation for OpenMRN is in
[doxygen](http://www.stack.nl/~dimitri/doxygen/) format. The best way to view
the documents is to build the HTML files (instructions below).

The existing OpenMRN stack makes heavy use of Linux build tools. As a result,
the best way to get up and running, and to build the documentation, is within
Linux or Mac OS X. If you have a Windows, we recommend that you create a Linux
virtual machine. Below are some high-level instructions on getting started.

## Create a Linux Virtual Machine

* Install Oracle [VirtualBox](https://www.virtualbox.org/) with is available for
free.
* Create a virtual machine
    * Ideally, this should be a 64-bit virtual machine
    * Assign it at least 4GB of RAM
    * Assign it multiple cores (improves build speed)
    * Create a _dynamically allocated_ virtual disk that is larger than the
      default. The default is 10GB, and we recommend at least 30GB. More is
      better because the virtual machine file itself will expand, but the
      maximum size is hard to change in the future
* Download a copy of Ubuntu desktop. At the time of this writing, 16.04 LTS is a
  good machine. Download this as an ISO disk image
* Start the virtual machine. It will ask you to select a startup disk. Select
  the ISO file that you downloaded for Ubuntu desktop

Once you've done this, you'll need to do a few more things before you can get
the source code and build the documentation.

## Installing Required Software (Linux or Mac)

All software installation will be done through the command line, known as a
_terminal_ in Ubuntu Linux. Most of these commands will start with **sudo** in
order to run them with administrator privileges. Building the documentation
requires the gcc/g++ or LLVM compiler to be installed. Ubuntu LTS comes with the
compilers already installed.

Open a terminal window (for Ubuntu, right click anywhere on the desktop and
click **Open Terminal**) and enter the following commands (these all require
access to the internet):

```
sudo apt-get install git
sudo apt-get install doxygen
cd ~
git clone https://github.com/bakerstu/openmrn/
cd openmrn/doc
make html
```
That should create the HTML files. Now open the **Files** application, navigate
to Home/openmrn/doc and double-click **index.html**. This will open the OpenMRN
documentation in your browser (which is Firefox by default).