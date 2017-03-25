# devsat
One step down from the flatsat: this cute little development board is for software development for OreSat. It's got an M0, M4, BBB, and Semtech radio all tied together via CAN.

Here's a block diagram:

![DevSat Block Diagram](blockdiagram-whiteboard.png)

And here are some details:

- M0 board
    - This is our "terminal node" or "sensor/actuator node" microcontroller. Small, low power, mostly off.
    - STM32F046K6 [NUCLEO-F042K6](https://www.digikey.com/products/en?keywords=497-15980-ND) development board: small, cheap, has ST-LINK, CAN, and low power. Also ChibiOS support.
- M4 board
    - This is our system controller (C3, T&DH, etc). Needs some horsepower, but not tons. Core of OreSat!
    - STM32F411 [NUCLEO-F411RE](https://www.digikey.com/product-detail/en/stmicroelectronics/NUCLEO-F411RE/497-14711-ND/4866485) development board: *Still* cheap, CAN, lots of MHzand ChibiOS support.
- BeagleBoneBlack
    - This is our flight computer analog. We want to fly the Octavo chip (we think?) but we don't need it right now. We also don't need or want WiFi (we'll use the ATH9K board)
    - Standard [BBB](https://www.digikey.com/product-detail/en/ghi-electronics-llc/BBB01-SC-505/BBB01-SC-505-ND/6210999)
- CAN transceivers
    - None of the boards have CAN transceivers build in, so we needs the transceivers.
    - Waveshare [SN65HVD230 CAN Board](http://www.waveshare.com/sn65hvd230-can-board.htm) fits the ticket well.
- USB to CAN adapters
    - Usually use use CANUSB, but they're expensive, so we found this one!
    - Protofusion [CANable](https://www.tindie.com/products/protofusion/canable-usb-to-can-bus-adapter/) USB to CAN bus adapter.

- Still need to spec:
   - A Semtech dev board
   - A >= 4 port powered USB hub
   - A box
   - A CAN "bus"
- User needs a laptop, and another Semtech dev board.

