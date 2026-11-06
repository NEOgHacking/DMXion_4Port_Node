# Ethernet library

This is Arduino Ethernet library 2.0.2, vendored for this board. The Ethernet
driver's SPI transactions use `SPI1`, which is wired to the onboard W5500:

- MISO: GPIO28
- MOSI: GPIO31
- SCK: GPIO30
- CS: GPIO29

The upstream library uses the global `SPI` object. All driver source references
to that object are changed to `SPI1`; the RP2350 core and other libraries are
not affected by a global preprocessor alias.
