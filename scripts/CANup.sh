# Shell script that brings up the can0 interface for the CANable transceiver
# See the CANable Getting Started page for details


# Written by Evan Yand for Portland State Aerospace Society
# 06/2017 licensed GPLv3


case $1 in
   "-10") sudo slcand -o -c -s0 /dev/serial/by-id/*CANtact*-if00 can0;;
   "-20") sudo slcand -o -c -s1 /dev/serial/by-id/*CANtact*-if00 can0;;
   "-50") sudo slcand -o -c -s2 /dev/serial/by-id/*CANtact*-if00 can0;;
   "-100") sudo slcand -o -c -s3 /dev/serial/by-id/*CANtact*-if00 can0;;
   "-125") sudo slcand -o -c -s4 /dev/serial/by-id/*CANtact*-if00 can0;;
   "-250") sudo slcand -o -c -s5 /dev/serial/by-id/*CANtact*-if00 can0;;
   "-500") sudo slcand -o -c -s6 /dev/serial/by-id/*CANtact*-if00 can0;;
   "-750") sudo slcand -o -c -s7 /dev/serial/by-id/*CANtact*-if00 can0;;
   "-1000") sudo slcand -o -c -s8 /dev/serial/by-id/*CANtact*-if00 can0;;
   "-h") 
        echo "Use the following options to set the bus speed in kbps:

        -10
        -20
        -50
        -100
        -125
        -250
        -500
        -750
        -1000
        
        Use the -h option to show this message.
        
        Example:
        $ ./CANup.sh -10
        
        Troubleshooting:
        - Double check that the CANable transciever is connected and powered on.
        - Restart the tranciever if necessary."
        exit 1;;
   *) 
        echo "Bus speed must be selected. Enter ./CANup.sh -h to see options"
        exit 1;;
esac


sudo ifconfig can0 up

if [ $? = "0" ] 
then
    echo "CAN interface is active on can0. Use the can-utils package to access."
else
    echo "Error configuring interface. Check connection and retry."
fi