# Release process

This document details how releases need to be built and published for OpenMRN.

## Purpose and content of releases

Releases fulfill two purposes:

- Precompiled binaries are made available for download. This is important for
  those that do not have facilities or the experience to compile OpenMRN
  applications from source.
  
- (At a later point) a packaged library is made available with include headers
  and `*.a` files that enable importing OpenMRN into a different compilation
  environment, such as various IDEs for embedded development.
  
The following is out of scope of this documentation:

- Releases for OpenMRNLite (Arduino compatible library) are documented in the
  [arduino/RELEASE.md](arduino/RELEASE.md) file.

## Requirements for building releases

- You need to be able to compile OpenMRN binaries. You need a linux host (or
  VM). The necessary packages have to be installed:
  
  - beyond standard `g++` you will need `sudo apt-get install
    libavahi-client-dev`
    
- You need both a linux.x86_64 and a raspberry pi host.

- On the linux host you should install node.js, npm, emscripten, and the
  packager:
  
  `sudo npm install -g pkg`
  
## Release numbering

Release numbers are marked as major.minor.patch, such as `v2.10.1`.

- Major release numbers are incremented once per year (2020 is 0, 2021 is 1,
  2022 is 2, ...).

- Minor release numbers are 10, 20, 30, 40 for given quarters, then incremented
  by one if there are multiple releases built within a quarter.

- Patch release numbers start at 1, and are only incremented if the same
  release needs to be re-built with a patch.

  
## How to build

All of the make commands here need to be run in the openmrn directory
(toplevel).

1. Start with a clean checkout. Run `make release-clean`.

2. On the linux.x86_64 host, run

   `make -j5 release-bin`
   
3. Copy `openmrn/bin/release/applications.linux.x86_64.zip` as one of the
   artifacts.
   
3. On the raspberry pi host, do the same:

   `make release-clean`
   
   `make -j4 release-bin`
   
4. Copy `openmrn/bin/release/applications.linux.armv7l.zip` as one of the
   artifacts. Rename it to `applications.linux.armv7l-raspberry-pi.zip`
   
5. On the linux.x86_64 host, build the javascript binaries. Run
   
   `make -j5 release-js`
   
6. Copy `openmrn/bin/release/applications.win.zip` and
   `openmrn/bin/release/applications.macos.zip`.
   
7. On the OpenMRN GitHub project, select Releases, create a new release, select
   create a new tag in the form of `release-v2.10.1`. Upload the release
   artifacts that you collected.
   
8. Publish the release.
