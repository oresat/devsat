# BeagleBone Black Setup

## Connecting

By default, the BeagleBone should set up an IP interface, and use dhcp to assign your computer the address ```192.168.7.1```, use ```ifconfig``` to check the configuration. Once this is working, you can ssh to the device:

```
$ ssh debian@192.168.7.2
```

The default password is "temppwd".


## IP over USB

In order to download packages, the BeagleBone needs to have an internet connection. You can grab a spare Ethernet cable, and plug it in to the network that way; however, it is usually easier to use the USB connection on your computer.

On the **BeagleBone**:

```
sudo route add default gw 192.168.7.1
```

On the **host** use ```ifconfig``` to find the interface that your computer uses to access the network. This is probably something like eth0 or wlan0, but it could be different. Then run the *BBBHostRouting.sh* script in the /devsat/scripts directory, with your interface as an argument. Example:

```
./BBBHostRouting.sh eth0
```

Go back to the **BeagleBone** and see if it worked:

```
sudo ping 8.8.8.8
```

**TODO: add DNS configuration intstructions.**

## CAN bus configuration