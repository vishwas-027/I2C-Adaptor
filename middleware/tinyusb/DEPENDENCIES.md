# Vendored USB dependencies

- TinyUSB 0.21.0, commit `dae3f9a366bfcddbf9dcf1b48d7500286a849539`
- ARM CMSIS 5, commit `2b7495b8535bdcb306dac29b9ded4cfb679d7e5c`
- ST CMSIS Device F4, commit `3c77349ce04c8af401454cc51f85ea9a50e34fc1`

Only TinyUSB's device-side sources and the CMSIS headers needed by its STM32F4
DWC2 controller driver are compiled. The project remains HAL-free.
