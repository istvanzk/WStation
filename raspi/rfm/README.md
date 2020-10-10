## RH_RF22 or RH_RF69 based server on Raspberry Pi with POSIX IPC Message Queue interface

Uses POSIX inter-process message queue to make available the received data packet and other info.

Requires the [bcm2835][1] library to be already installed to access the GPIO pins to drive the [RFM22B based radio module][3] or [RFM69HCW based radio module][4] via 
the [modified RadioHead library][2]. 

Requires *boot/config.txt* to contain the line: 

```cpp
dtoverlay=gpio-no-irqh
```

### Files

*server_mq* : RH_RF22 or RH_RF69 based server on Raspberry Pi with POSIX IPC Message Queue interface


[1]: http://www.airspayce.com/mikem/bcm2835/
[2]: https://github.com/istvanzk/RadioHead
[3]: https://www.sparkfun.com/products/retired/10154
[4]: https://www.sparkfun.com/products/13910
