// Host stub for the ESP port layer. Models flash faithfully enough for the
// storage tests: writes may only clear bits, erase restores 0xFF. The legacy
// compatibility shim depends on that asymmetry, so a memcpy model would hide
// the bug it exists to prevent.
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

typedef uint8_t byte;

#define SPI_FLASH_SECTOR_SIZE 4096
#define SPI_FLASH_SEC_SIZE    4096

#ifdef __cplusplus
extern "C" {
#endif

extern uint32_t _EEPROM_start;
extern uint32_t _SPIFFS_start;

uint32_t fake_flash_base(void);
void     fake_flash_erase_all(void);
byte    *fake_flash_bytes(void);

int fake_spi_flash_read(uint32_t addr, void *buf, size_t size);
int fake_spi_flash_write(uint32_t addr, const void *buf, size_t size);
int fake_spi_flash_erase_sector(uint32_t sector);

#ifdef __cplusplus
}
#endif

#define spiflash_read(addr, buffer, size)  (fake_spi_flash_read((addr), (buffer), (size)) == 0)
#define spiflash_write(addr, data, size)   (fake_spi_flash_write((addr), (data), (size)) == 0)
#define spiflash_erase_sector(addr)        (fake_spi_flash_erase_sector((addr) / SPI_FLASH_SECTOR_SIZE) == 0)
