# rookie_pico

Firmware for the RPi Pico that acts as the base controller for [Rookie](https://github.com/wesfletch/rookie.git).

## Prerequisites

Build and install `pico_interface` first — see [`src/pico_interface`](../pico_interface/README.md).

## Build

```bash
cmake --preset dev
cmake --build ../build/pico
```

Output is written to `src/build/pico/`. Flash `rookie_pico.uf2` to the Pico.

## Options

| CMake option | Default | Description |
|---|---|---|
| `OUTBOUND_USE_UART` | `OFF` | Route outbound messages over UART instead of USB CDC |
