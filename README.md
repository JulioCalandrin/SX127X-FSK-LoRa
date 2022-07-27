# SX127X-FSK-LoRa Library

<!---Esses são exemplos. Veja https://shields.io para outras pessoas ou para personalizar este conjunto de escudos. Você pode querer incluir dependências, status do projeto e informações de licença aqui--->

![GitHub repo size](https://img.shields.io/github/repo-size/iuricode/README-template?style=for-the-badge)
![GitHub language count](https://img.shields.io/github/languages/count/iuricode/README-template?style=for-the-badge)

This is a *FSK* library for interfacing the SX1276 radio module from Semtech with STM32 microcontrolers. Currently it only supports FSK but I will soon glue together the LoRa code I have laying around in my computer.

This library is not finished, use with caution, but at least all available functions have been fully tested on a 2AD66 G-NiceRF module.

Also, there are comments spread around the code explaining limitations of functions, such as the transmit and receive functions.

## Future Adjustments:

As said before, I will add LoRa soon, among with other items:

* LoRa:
- [ ] Add LoRa functions

* FSK:
- [ ] Optmize dynamic receive/transmitt functions for higher bitrates.
- [ ] 

## How to use it:

The module uses SPI, and since this was designed using an STM32 in mind you might need to swap the STM32_SPI.c, STM32_SPI.h and STM32.h files.

In the end all you have to do is recreate the STM32's HAL_StatusTypeDef for the "error management" functionality and swap out the macros for the SPI.