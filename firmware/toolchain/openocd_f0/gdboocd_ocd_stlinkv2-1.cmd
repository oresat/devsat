target remote | openocd -s ../../toolchain/openocd -f stlinkv2-1_stm32.cfg -c "gdb_port pipe"
monitor stm32f0x.cpu configure -rtos ChibiOS
monitor reset halt
