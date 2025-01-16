# ESP-IDF Build Instructions
The ESP-IDF has its own CMake based build environment for application developers. Using the standard ESP-IDF application build environment, The openmrn/etc/esp-idf directory can be linked into the ESP application "components" directory.
```
cd <application path>/components
ln -s <openmrn path>/etc/esp-idf/ openmrn
```
This will result in openmrn being compiled as a registered idf component with the name "openmrn".
