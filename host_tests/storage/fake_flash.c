#include "port.h"
#include <string.h>

// Storage lives in one sector. The base is derived exactly as storage.c derives
// it, so both agree on where PAIRINGS_ADDR lands regardless of where the linker
// put these symbols on the host.
uint32_t _EEPROM_start;
uint32_t _SPIFFS_start;

int hk_log_enabled = 0;

static byte sector[SPI_FLASH_SECTOR_SIZE];

uint32_t fake_flash_base(void) {
    return (uint32_t) (&_EEPROM_start) - 0x40200000;
}

void fake_flash_erase_all(void) {
    memset(sector, 0xff, sizeof(sector));
}

byte *fake_flash_bytes(void) {
    return sector;
}

static int in_range(uint32_t addr, size_t size, size_t *off) {
    uint32_t base = fake_flash_base();
    if (addr < base) return 0;
    uint32_t o = addr - base;
    if (o + size > sizeof(sector)) return 0;
    *off = o;
    return 1;
}

int fake_spi_flash_read(uint32_t addr, void *buf, size_t size) {
    size_t off;
    if (!in_range(addr, size, &off)) return -1;
    memcpy(buf, sector + off, size);
    return 0;
}

// Real NOR flash can only clear bits on write. Modelling that is the point of
// this harness: a legacy record's zeroed status byte can never be raised to
// 0xff in place, which is why the shim reads old records instead of stamping them.
int fake_spi_flash_write(uint32_t addr, const void *buf, size_t size) {
    size_t off;
    if (!in_range(addr, size, &off)) return -1;
    const byte *src = (const byte *) buf;
    for (size_t i = 0; i < size; i++) sector[off + i] &= src[i];
    return 0;
}

int fake_spi_flash_erase_sector(uint32_t sector_idx) {
    (void) sector_idx;
    fake_flash_erase_all();
    return 0;
}
