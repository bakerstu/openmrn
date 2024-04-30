# Releasing the OpenMRN Arduino libraries

Currently there is one arduino library generated from OpenMRN sources:

- http://github.com/openmrn/OpenMRNLite

This library is a packaged version of OpenMRN application-level core for
implementing the OpenLCB/LCC stack, and the arduino drivers. The FreeRTOS-based
POSIX emulation layer is not part of this library despite the code running on
microcontrollers; instead the native frameworks and device drivers of the
Arduino ecosystem (whichever incarnation) are used.

## Where to get the library

- by exporting directly from the OpenMRN source tree (under linux also symlinks
  are supported)
- by downloading a released source code ZIP file from GitHub
  (https://github.com/openmrn/OpenMRNLite/releases) and importing that ZIP file
  into the arduino UI.
- by using the Arduino Library Manager to download a released version from the
  global Arduino index.
  
## Release path

- [bakerstu/openmrn](https://github.com/bakerstu/openmrn) git repo
  - (export via libify.sh)
- [openmrn/OpenMRNLite](https://github.com/openmrn/OpenMRNLite) git repo
  - releases tagged via github release tool
- Arduino Library Manager global index
  - crawls library versions once an hour.

## Release process

1. Check out both bakerstu/openmrn and openmrn/OpenMRNLite.
1. Make sure the working tree of openmrn is clean, no dirty or untracked files
   are present.
1. Edit library.properties and library.json to increment the version number.
1. Run `arduino/libify.sh "/path/to/OpenMRNLite" . -f`
1. Go to OpenMRNLite, commit the results to git, then push it to master.
   To generate the commit note and later the release notes, use this command: `git log --oneline OpenMRNLite-v1.0.3..OpenMRNLite-v2.0.0 | sed -e 's/[(]#/(https:\/\/github.com\/bakerstu\/openmrn\/pull\//' | sed 's/^/bakerstu\/openmrn@/'` 
1. go to http://github.com/openmrn/OpenMRNLite, click the releases tab, then
   create a release called v1.2.3 if your library.properties said 1.2.3 as
   version.

### Executing libify.sh to create the OpenMRNLite library
The libify.sh script requires a bash like environment for execution, on Windows
the Git bash commandline will work. On Linux/MacOS the native bash shell will
work.

```bash
    sh libify.sh {path to OpenMRNLite creation directory} {path to OpenMRN} -f
```
    
The `-f` argument will clean the target library directory first. Make sure you
do not have any changes (including in the examples folder!). This is helpful in
case OpenMRN might have removed or renamed a file.

On Linux/Mac you can add the `-s` flag in order to create symlinks instead of
copying files. This allows changes in openmrn (including pulling from github) to
be immediately picked up by the arduino library. Warning: do not use `-s` when
releasing!

#### Arduino IDE library generation
On Windows the Arduino IDE stores the libraries under
"Documents\Arduino\libraries", this can be accessed via the Git bash
commandline as:
```bash
    sh arduino/libify.sh "$USERPROFILE/Documents/Arduino/libraries/OpenMRNLite" .
```
when executed from the OpenMRN repository root folder.

On Linux the location would normally be
/home/{user}/Documents/Arduino/libraries.

On macOS the location would normally be
/Users/{user}/Documents/Arduino/libraries.

#### PlatformIO IDE library generation
For PlatformIO IDE it would be recommended to put this into the project lib
folder instead. By default the PlatformIO build process will inspect the lib
folder for project specific libraries and will automatically include them in
the compilation.
```bash
    sh arduino/libify.sh "/path/to/project/lib/OpenMRNLite" .
```
when executed from the OpenMRN repository root folder.
