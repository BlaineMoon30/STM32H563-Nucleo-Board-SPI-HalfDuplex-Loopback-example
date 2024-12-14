Description:
Configuration of GPIO and SPI peripherals to transmit bytes from an SPI Master device to an SPI Slave device in Interrupt mode. 
Additionally, the SPI Master is switched to RX mode and the SPI Slave to TX mode, 
demonstrating byte transmission from the Slave to the Master in Interrupt mode.
This example is based on the STM32H5xx SPI HAL API. 

Example execution:
- Master to Slave Communication:
  - SPI1 is set to Master TX mode, and SPI2 is set to Slave RX mode. (Half-Duplex)
  - Master transmits data (masterTxBuffer), and Slave receives it (slaveRxBuffer).
  - The received data in slaveRxBuffer is compared with the transmitted data in masterTxBuffer.
  - If the data matches, the slaveRxBuffer is cleared, and the loop ends; otherwise, an error handler is called.
  
- Slave to Master Communication:
  - SPI1 is switched to Master RX mode, and SPI2 is switched to Slave TX mode.
  - Slave transmits data (slaveTxBuffer), and Master receives it (masterRxBuffer).
  - The received data in masterRxBuffer is compared with the transmitted data in slaveTxBuffer.
  - If the data matches, the masterRxBuffer is cleared, and the loop ends; otherwise, an error handler is called.

Hardware
- NUCLEO-H563ZI
- https://www.st.com/en/evaluation-tools/nucleo-h563zi.html

< SPI1 GPIO Configuration >
<br>PA5     ------> SPI1_SCK</br>
PA7     ------> SPI1_MOSI

< SPI2 GPIO Configuration >
<br>PC2     ------> SPI2_MISO</br>
PB13    ------> SPI2_SCK

SPI1 Peripheral is configured in Master mode Half-Duplex Tx.
SPI2 Peripheral is configured in Slave mode Half-Duplex Rx.

- This example runs on STM32H563ZITx devices.

- This example has been tested with NUCLEO-H563ZI board and can be
  easily tailored to any other supported device and development board.

Relation with Board connector:
 <br>PA5  is connected to pin 11 of CN12 connector</br>
     PA7  is connected to pin 15 of CN12 connector
 <br>PB13 is connected to pin 30 of CN12 connector</br>
     PC2  is connected to pin 35 of CN11 connector

Note: 
- the SPI must be disabled when changing the direction of the communication.
- In bidirectional SPI communication,
  a critical issue can occur if the communication direction is not synchronized between nodes.
  This can lead to conflicting output levels on the shared data line,
  causing potential damage or excessive current flow.
  To mitigate this, it is recommended to <strong>**_insert a serial resistor between the MISO and MOSI pins_**</strong>
  to protect the outputs and limit current during conflicts.
