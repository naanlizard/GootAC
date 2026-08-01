// Host tests for the vendored HomeKit storage layer.
//
// Covers the upgrade path from the Mixiaoxiao record layout to dkerr64's. The
// two differ in the last reserved byte: dkerr64 repurposed it as `active` and
// gates every read on it, while the old layout wrote that byte as zero. Without
// a compatibility rule the entire pairing table reads as empty, which is what
// took Simples off HomeKit.
#include <stdio.h>
#include <string.h>
#include <stdint.h>

extern "C" {
#include "port.h"
#include "constants.h"
#include "pairing.h"
#include "storage.h"
}

static int checks = 0, failures = 0;

static void ok(bool cond, const char *what) {
    checks++;
    if (!cond) { failures++; printf("FAIL: %s\n", what); }
}

// --- Record layout, stated explicitly so the test does not inherit whatever
// --- the library struct happens to be today.
static const size_t REC_SIZE      = 80;
static const size_t OFF_MAGIC     = 0;
static const size_t OFF_PERMS     = 4;
static const size_t OFF_DEVICE_ID = 5;
static const size_t OFF_PUBKEY    = 41;
static const size_t PAIRINGS_OFF  = 128;

// The library memcmps a full DEVICE_ID_SIZE against stored ids, so every id
// handed to it must be exactly that wide. Short literals read out of bounds.
struct DevId {
    char v[DEVICE_ID_SIZE + 1];
    explicit DevId(const char *tag) {
        memset(v, 'x', DEVICE_ID_SIZE);
        v[DEVICE_ID_SIZE] = 0;
        size_t n = strlen(tag);
        memcpy(v, tag, n < DEVICE_ID_SIZE ? n : DEVICE_ID_SIZE);
    }
};

static byte *rec(int idx) { return fake_flash_bytes() + PAIRINGS_OFF + REC_SIZE * idx; }

static void format_storage_header() {
    memcpy(fake_flash_bytes() + 0, "HAP", 4);
    memcpy(fake_flash_bytes() + 4, "AA:BB:CC:DD:EE:FF", 17);
    memset(fake_flash_bytes() + 32, 0x5a, 64);
}

// A record exactly as the Mixiaoxiao library wrote it: full 80 bytes from a
// zeroed struct, so the trailing status byte is 0x00.
static void write_legacy_pairing(int idx, const DevId &id, byte perms) {
    byte r[REC_SIZE];
    memset(r, 0, sizeof(r));
    memcpy(r + OFF_MAGIC, "HAP", 4);
    r[OFF_PERMS] = perms;
    memcpy(r + OFF_DEVICE_ID, id.v, DEVICE_ID_SIZE);
    memset(r + OFF_PUBKEY, 0x11 + idx, 32);
    memcpy(rec(idx), r, REC_SIZE);
}

// How the old library marked a removal: zero the whole record, magic included.
static void legacy_delete(int idx) { memset(rec(idx), 0, REC_SIZE); }

static int count_pairings() {
    pairing_iterator_t it;
    homekit_storage_pairing_iterator_init(&it);
    pairing_t p;
    int n = 0;
    while (!homekit_storage_next_pairing(&it, &p)) n++;
    homekit_storage_pairing_iterator_done(&it);
    return n;
}

static bool has_pairing(const DevId &id) {
    pairing_t p;
    return homekit_storage_find_pairing(id.v, &p) == 0;
}

static DevId legacy_id(int i) {
    char tag[32];
    snprintf(tag, sizeof(tag), "LEGACY-%02d", i);
    return DevId(tag);
}

static void reset_to_legacy_device(int n_pairings) {
    fake_flash_erase_all();
    format_storage_header();
    for (int i = 0; i < n_pairings; i++)
        write_legacy_pairing(i, legacy_id(i), pairing_permissions_admin);
}

static ed25519_key make_key(byte fill) {
    ed25519_key k;
    crypto_ed25519_init(&k);
    memset(k.pub, fill, sizeof(k.pub));
    return k;
}

int main() {
    const DevId NEW0("NEW-0"), NEW1("NEW-1"), AFTER("NEW-AFTER-LEGACY");

    // Flash model itself: writes clear bits only, erase restores them.
    fake_flash_erase_all();
    ok(fake_flash_bytes()[0] == 0xff, "erase yields 0xff");
    byte v = 0x0f;
    fake_spi_flash_write(fake_flash_base(), &v, 1);
    ok(fake_flash_bytes()[0] == 0x0f, "write clears bits");
    v = 0xff;
    fake_spi_flash_write(fake_flash_base(), &v, 1);
    ok(fake_flash_bytes()[0] == 0x0f, "write cannot raise bits");

    // 1. Legacy pairings must survive the library upgrade. This is the
    //    regression that unpaired Simples.
    reset_to_legacy_device(2);
    ok(homekit_storage_init(0) == 0, "legacy sector passes magic check, no reformat");
    ok(fake_flash_bytes()[PAIRINGS_OFF] == 'H', "legacy record not erased by init");
    ok(count_pairings() == 2, "both legacy pairings enumerate");
    ok(has_pairing(legacy_id(0)), "legacy pairing 0 found by id");
    ok(has_pairing(legacy_id(1)), "legacy pairing 1 found by id");

    // 2. Legacy removals stay removed: the old delete zeroed the magic.
    reset_to_legacy_device(2);
    legacy_delete(0);
    ok(count_pairings() == 1, "legacy-deleted record does not resurrect");
    ok(!has_pairing(legacy_id(0)), "deleted legacy id absent");
    ok(has_pairing(legacy_id(1)), "surviving legacy id present");

    // 3. Records written by the current library read back.
    fake_flash_erase_all();
    homekit_storage_init(1);
    ed25519_key k = make_key(0x77);
    ok(homekit_storage_add_pairing(NEW0.v, &k, pairing_permissions_admin) == 0,
       "add_pairing succeeds on fresh storage");
    ok(count_pairings() == 1, "new pairing enumerates");
    ok(has_pairing(NEW0), "new pairing found by id");

    // 4. Removal through the library actually removes.
    ok(homekit_storage_add_pairing(NEW1.v, &k, pairing_permissions_admin) == 0,
       "second add succeeds");
    ok(count_pairings() == 2, "two new pairings");
    ok(homekit_storage_remove_pairing(NEW0.v) == 0, "remove succeeds");
    ok(count_pairings() == 1, "removed pairing gone");
    ok(!has_pairing(NEW0), "removed id absent");
    ok(has_pairing(NEW1), "other id survives removal");

    // 5. Mixed table. A legacy device that pairs a new controller must keep
    //    both. This is what rules out a global legacy-mode flag: the new
    //    record makes the table stop looking legacy.
    reset_to_legacy_device(2);
    homekit_storage_init(0);
    ok(homekit_storage_add_pairing(AFTER.v, &k, pairing_permissions_admin) == 0,
       "add onto a legacy table succeeds");
    ok(count_pairings() == 3, "legacy and new pairings coexist");
    ok(has_pairing(legacy_id(0)), "legacy id survives a new add");
    ok(has_pairing(legacy_id(1)), "second legacy id survives a new add");
    ok(has_pairing(AFTER), "new id present alongside legacy");

    // 6. Removing a legacy pairing through the library must stick.
    reset_to_legacy_device(2);
    homekit_storage_init(0);
    ok(homekit_storage_remove_pairing(legacy_id(0).v) == 0, "remove legacy pairing");
    ok(!has_pairing(legacy_id(0)), "removed legacy id absent");
    ok(count_pairings() == 1, "one legacy pairing remains");

    // 7. Capacity accounting still works with legacy records present.
    reset_to_legacy_device(16);
    homekit_storage_init(0);
    ok(count_pairings() == 16, "full legacy table enumerates");
    ok(!homekit_storage_can_add_pairing(), "full legacy table reports no room");
    reset_to_legacy_device(2);
    homekit_storage_init(0);
    ok(homekit_storage_can_add_pairing(), "partly filled legacy table reports room");

    // 8. reset_pairing_data must empty the whole table. The server calls it once
    //    the last admin pairing is gone, so a partial clear leaves controllers
    //    behind and burns slots.
    reset_to_legacy_device(3);
    homekit_storage_init(0);
    ok(homekit_storage_reset_pairing_data() == 0, "reset_pairing_data succeeds");
    ok(count_pairings() == 0, "reset_pairing_data clears every legacy record");

    fake_flash_erase_all();
    homekit_storage_init(1);
    homekit_storage_add_pairing(NEW0.v, &k, pairing_permissions_admin);
    homekit_storage_add_pairing(NEW1.v, &k, 0);
    ok(count_pairings() == 2, "two records before reset");
    ok(homekit_storage_reset_pairing_data() == 0, "reset_pairing_data succeeds on new records");
    ok(count_pairings() == 0, "reset_pairing_data clears every new record");
    ok(homekit_storage_can_add_pairing(), "table reusable after reset");

    // 9. Accessory identity written under the old layout must still load. If it
    //    does not, the server generates a fresh id AND a fresh long-term key and
    //    saves both, which silently invalidates every existing pairing while
    //    leaving homekit_paired reading 1.
    fake_flash_erase_all();
    format_storage_header();
    char idbuf[ACCESSORY_ID_SIZE + 8];
    memset(idbuf, 0, sizeof(idbuf));
    ok(homekit_storage_load_accessory_id(idbuf) == 0, "legacy accessory id loads");
    ok(strcmp(idbuf, "AA:BB:CC:DD:EE:FF") == 0, "legacy accessory id round-trips");

    printf("%d checks, %d failures\n", checks, failures);
    return failures ? 1 : 0;
}
