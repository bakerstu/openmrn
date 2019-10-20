# Adding SocketCAN support on the Raspberry Pi 4

These are my notes from adding SocketCAN support and building the **hub** application in a Raspberry Pi.

- Start with the pre-built JMRI - RPI disk image from: [M Steve Todd's - JMRI RaspberryPi as Access Point](https://mstevetodd.com/rpi)

- Install the MCP2517 CAN interface hardware from: [2-Channel CAN-BUS(FD) Shield for Raspberry Pi](https://www.seeedstudio.com/2-Channel-CAN-BUS-FD-Shield-for-Raspberry-Pi-p-4072.html) and followed their instructions for building the MCP2517 kernel device driver on their [Support Wiki](http://wiki.seeedstudio.com/2-Channel-CAN-BUS-FD-Shield-for-Raspberry-Pi/#software)  page.

- Install some needed packages on the RPi:

	`sudo apt install git doxygen libavahi-client-dev` 

- Download the OpenMRN source code to the RPi:

	`cd ~`
	
	`git clone https://github.com/bakerstu/openmrn.git`

- Build the **hub** application:

	`cd openmrn/applications/hub/targets/linux.rpi1/`

	`make` 
	
- Configure the **can0** interface for 125,000 bps and run the **hub** application at system at start-up by creating the file: `/etc/network/interfaces.d/can0` with the following lines:
	```
	allow-hotplug can0
	iface can0 can static
	  bitrate 125000
	  up /home/pi/openmrn/applications/hub/targets/linux.rpi1/hub -s $IFACE &
	```

- Configure the LCC Layout Connection in JMRI to use
	- System Connection: `CAN via GridConnect Network Interface`
	- IP Address/Host Name: `localhost`
	- TCP/UDP Port: `12021` 
