# SX127X-FSK-LoRa Library

This is a FSK library for interfacing the SX1276 radio module from Semtech with STM32 microcontrolers. Currently it only supports FSK but I will soon glue together the LoRa code I have laying around in my computer.

The SX127X module line is pretty complex, offering MANY configuration options, so this library is NOT 100% finished, use with caution, but at least all available functions have been fully tested on a 2AD66 G-NiceRF module, wich is enough to get transmitting and receiving up to 255 bytes in FSK mode.

Also, there are comments spread around the code explaining limitations of functions, such as the transmit and receive functions.


## How to use it:

The module uses SPI, and since this was designed using an STM32 in mind you might need to swap the STM32_SPI.c, STM32_SPI.h and STM32.h files.

In the end all you have to do is recreate the STM32's HAL_StatusTypeDef for the "error management" functionality and swap out the macros for the SPI.

------- Post-It -------

This is where I keep some notes
about the tasks I'm doing in
the library.

-----------------------

## To-Do:

I'll turn them into issues later.

* General:
- [ ] Bandwidths should be typedefs (or not? Think about it).
- [ ] Remove unecessary macros and turn them into regular Masks (some lost on OCP and others on LoRa config).
- [ ] Remove magic numbers from bandwidths functions
- [ ] Remove Magic Numbers from DIO function selection.
- [ ] Add AGC functions.

* LoRa:
- [ ] Finish the LoRaConfig_t struct.
- [ ] Go over the entire LoRa library to make it more like the FSK one.
- [ ] Add Implicit header mode.
- [ ] Add LowDataRate optimization bit.

* FSK:
- [ ] Optmize dynamic receive/transmitt functions for higher bitrates.
- [ ] Add remaining configuraton options.
- [ ] Create Enable/Disable functions for CRC Autoclear.
- [ ] Create Enable/Disable functions for Auto Restart RX.
- [ ] Create Enable/Disable functions for LNA Boost.
- [ ] Remove Magic Numbers from the PreambleDetector config.

* OOK:
- [ ] Add it for some weird reason.
