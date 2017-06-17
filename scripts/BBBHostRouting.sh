# Script that configures the routing needed for the BeagleBone Black to connect to the internet over USB
# Run it on the host PC

# Written by Evan Yand for the Portland State Aerospace Society
# 06/2017 licensed GPLv3

echo "BTW I'm not sure if this works quite yet...'"

# Find BBB interface
IFACE=ifconfig|grep 192.168.7.1 -B1|cut -d " " -f1  

sudo ifconfig $IFACE 192.168.7.1
sudo iptables --table nat --append POSTROUTING --out-interface $1 -j MASQUERADE
sudo iptables --append FORWARD --in-interface $IFACE -j ACCEPT

echo 1 > /proc/sys/net/ipv4/ip_forward
echo $?