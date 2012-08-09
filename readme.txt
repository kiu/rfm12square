rfm12square - A RFM12 LED display
http://schoar.de/tinkering/rfm12square

1. Flash the ATMega8 through ISP
    make fuse
    make program

2. Wait for data, e.g. from http://schoar.de/tinkering/rfm12trx

3. If no data is received within TIMEOUT, LEDs will show TIMEOUT_COLOR
