# Semtech Dev Board App

This app will drive the [development board for the SX1236](https://github.com/oresat/semtech-dev-board). Note: code originally copied from app_candev.

## Building

```
ln -s main_rx.c OR main_tx.c to main.c
ln -s semtech_rx.c OR semtech_tx.c to semtech.c
```


```
$ make write
```

## Additional Docs

- [SX1236 datasheet](http://www.semtech.com/images/datasheet/sx1236.pdf)
- [ChibiOS SPI Driver & HAL](http://chibios.sourceforge.net/docs/hal_stm32f4xx_rm/struct_s_p_i_driver.html)
