
## How CAN addresses are assigned for a board : Use Makefile Definition

### See makefile:

```
ORESAT_CAN_ADDRESS=0xA0
UDEFS += -DMY_CAN_ADDRESS=$(ORESAT_CAN_ADDRESS)
```

### If CAN address is undefined then the default  is:

```
#ifndef MY_CAN_ADDRESS
#define MY_CAN_ADDRESS 0xCAFE
#endif
```


