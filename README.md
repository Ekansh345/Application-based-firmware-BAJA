**Progress log**
- Set up STM32CubeIDE project with FDCAN1 enabled and pins PA11/PA12 configured.
- Learned the basics of CAN: arbitration, message IDs, filters, and transceivers.
- Understood FDCAN timing concepts: prescaler, Seg1/Seg2, SJW, nominal vs. data bitrate.
- Added the first CAN filter (accept‑all) and learned why filters are mandatory for FDCAN.
- Enabled RX interrupts and implemented a basic RX callback.
- Debugged reception using GPIO toggling.

- Add UART debug prints for received CAN frames.
- Add CAN transmit test.
- Implement loopback mode for self-testing.
- Create FreeRTOS‑based CAN architecture (Rx task, Tx task, queues).
- Start designing actual motor controller + BMS CAN map.
