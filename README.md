# devsat
One step down from the flatsat: this cute little development board is for software development for OreSat. It's got an M0, M4, BBB, and Semtech radio all tied together via CAN.

Here's a block diagram (bet you're gonna be so jelly of our sweet drawing skillz):

![DevSat Block Diagram](https://github.com/oresat/devsat/blob/master/doc/block_diagram.jpg)

Photo of the actual thing:

![DevSat Photo](https://github.com/oresat/devsat/blob/master/doc/devsat_photo.jpg)


And here are some details:

- M0 board
    - This is our "terminal node" or "sensor/actuator node" microcontroller. Small, low power, mostly off.
    - STM32F046K6 [NUCLEO-F042K6](https://www.digikey.com/products/en?keywords=497-15980-ND) development board: small, cheap, has ST-LINK, CAN, and low power. Also ChibiOS support.
- M4 board
    - This is our system controller (C3, T&DH, etc). Needs some horsepower, but not tons. Core of OreSat!
    - STM32F446 [NUCLEO-F446RE](https://www.digikey.com/short/3nvcdz) development board: It's a beast: *Still* cheap, CAN, lots of MHz and ChibiOS support.
- BeagleBoneBlack
    - This is our flight computer analog. We want to fly the Octavo chip (we think?) but we don't need it right now. We also don't need or want WiFi (we'll use the ATH9K board)
    - Standard [BBB](https://www.digikey.com/product-detail/en/ghi-electronics-llc/BBB01-SC-505/BBB01-SC-505-ND/6210999)
- CAN transceivers
    - None of the boards have CAN transceivers build in, so we needs the transceivers.
    - Waveshare [SN65HVD230 CAN Board](http://www.waveshare.com/sn65hvd230-can-board.htm) fits the ticket well.
- USB to CAN adapters
    - Usually use use CANUSB, but they're expensive, so we found this one!
    - Protofusion [CANable](https://www.tindie.com/products/protofusion/canable-usb-to-can-bus-adapter/) USB to CAN bus adapter.
- Semtech SX1236 development board
    - An engineering reference design that is similar to the Semtech versions for various parts but unavailable for the SX1236. this one was designed by our own Evan Yand.
    - Design details are in the [semtech-dev-board](https://github.com/oresat/semtech-dev-board) repo.

- 4 port powered USB hub
    - Yes...

- Still need to spec:
   - A box
   - A CAN "bus"

- User needs a laptop, and another Semtech dev board / M4 package.

