
## Notes


#### uEnv.txt

* There is something wrong with cape_universal on BBB
* Disable it by commenting out the following line in /boot/uEnv.txt

```
#enable_uboot_cape_universal=1

```

* Next, enable the SPIDEV0 and CAN1 pins on the BBB by uncommenting the following line and adding to the end of it the desired overlays

```
cape_enable=bone_capemgr.enable_partno=BB-CAN1,BB-SPIDEV0

```

* The uEnv.txt in this directory serves as an example. A simple diff should show the relevent changes between the stock image uEnv.txt and the desired configuration (excluding possible kernel version differences)


### Reference

#### Where are the overlays?

* https://groups.google.com/forum/#!topic/beagleboard/vMd2b508wAE

* see /lib/firmware

```
debian@beaglebone:/lib/firmware$ find . -iname \*SPI\*.dtbo -print            
./BB-SPI0-ADS8688-0A00.dtbo
./BB-SPI0-MCP3008-00A0.dtbo
./ADAFRUIT-SPI0-00A0.dtbo
./BB-SPIDEV1-00A0.dtbo
./BB-SPIDEV0-00A0.dtbo
./BB-SPIDEV1A1-00A0.dtbo
./ADAFRUIT-SPI1-00A0.dtbo
debian@beaglebone:/lib/firmware$
```


#### Enabled Overlays are shown via cat of bone_capemgr slots file


```
debian@beaglebone:/lib/firmware$ cat /sys/devices/platform/bone_capemgr/slots 
 0: PF----  -1 
 1: PF----  -1 
 2: PF----  -1 
 3: PF----  -1 
 4: P-O-L-   0 Override Board Name,00A0,Override Manuf,BB-CAN1
 5: P-O-L-   1 Override Board Name,00A0,Override Manuf,BB-SPIDEV0
debian@beaglebone:/lib/firmware$
```
