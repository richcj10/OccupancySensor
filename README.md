# occupancysensor

## Library

ModBusBL is not vendored. It is pulled from `../ModBussLibrary_BL` via
`symlink://` in `platformio.ini`, so that repo must be checked out next to this one.

EEPROM 0x00–0x04 is reserved for ModBusBL and the bootloader. Application data
lives at 0x10 and above (see `src/EEPROMData.h`).

## Flashing

Use the canonical flasher in the bootloader repo:
`C:\Repo\ModBusBootlader\tools\mbbp_flash.py` (supports `--scan`, `--verify`,
`--new-address`). Run it with `--help` for usage.
