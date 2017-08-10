
## Notes


#### uEnv.txt

* There is something wrong with cape_universal and SPI on BBB
* Disable it

```
#cmdline=coherent_pool=1M net.ifnames=0 quiet cape_universal=enable
cmdline=coherent_pool=1M net.ifnames=0 quiet

```

* See /boot/uEnv.txt
  * Enable the SPI dts
  * then remove cape manager universal from the command line.
  * Example in uEnv.txt

* Also change file in 
  * /sys/devices/platform/bone_capemgr

### Reference

#### Where are the overlays?

* https://groups.google.com/forum/#!topic/beagleboard/vMd2b508wAE

* see /lib/firmware

```
root@beaglebone:/opt/source# find . -name \*dts -print | grep SPI
./adafruit-beaglebone-io-python/overlays/ADAFRUIT-SPI0-00A0.dts
./adafruit-beaglebone-io-python/overlays/ADAFRUIT-SPI1-00A0.dts
./bb.org-overlays/src/arm/BB-SPIDEV1A1-00A0.dts
./bb.org-overlays/src/arm/BB-SPIDEV0-00A0.dts
./bb.org-overlays/src/arm/BB-SPIDEV1-00A0.dts
./bb.org-overlays/src/arm/BB-SPI0-MCP3008-00A0.dts
./Userspace-Arduino/overlay/BB-SPI0-01-00A0.dts
root@beaglebone:/opt/source# 
```


#### Change file 'slots' to enable the SPIDEV0

```
root@beaglebone:/sys/devices/platform/bone_capemgr# ls
baseboard  driver  driver_override  modalias  of_node  power  slot-4  slots  subsystem	uevent
root@beaglebone:/sys/devices/platform/bone_capemgr# cat slot
cat: slot: No such file or directory
root@beaglebone:/sys/devices/platform/bone_capemgr# cat slots 
 0: PF----  -1 
 1: PF----  -1 
 2: PF----  -1 
 3: PF----  -1 
 4: P-O-L-   0 Override Board Name,00A0,Override Manuf,BB-SPIDEV0
root@beaglebone:/sys/devices/platform/bone_capemgr# 

```
