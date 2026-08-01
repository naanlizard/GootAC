// Host stub for crypto.h. storage.c only moves opaque key bytes around, so the
// ed25519 calls just copy buffers; no curve arithmetic is exercised by these tests.
#pragma once
#include <string.h>
#include <stdint.h>
#include "port.h"

#define ED25519_KEY_BYTES 32

typedef struct {
    byte pub[ED25519_KEY_BYTES];
    byte priv[ED25519_KEY_BYTES];
    int  initialized;
} ed25519_key;

#ifdef __cplusplus
extern "C" {
#endif

static inline void crypto_ed25519_init(ed25519_key *k) {
    memset(k, 0, sizeof(*k));
    k->initialized = 1;
}

static inline int crypto_ed25519_import_public_key(ed25519_key *k, const byte *data, size_t size) {
    if (size < ED25519_KEY_BYTES) return -1;
    memcpy(k->pub, data, ED25519_KEY_BYTES);
    k->initialized = 1;
    return 0;
}

static inline int crypto_ed25519_export_public_key(const ed25519_key *k, byte *buf, size_t *size) {
    if (*size < ED25519_KEY_BYTES) return -1;
    memcpy(buf, k->pub, ED25519_KEY_BYTES);
    *size = ED25519_KEY_BYTES;
    return 0;
}

static inline int crypto_ed25519_import_key(ed25519_key *k, const byte *data, size_t size) {
    if (size < 2 * ED25519_KEY_BYTES) return -1;
    memcpy(k->pub, data, ED25519_KEY_BYTES);
    memcpy(k->priv, data + ED25519_KEY_BYTES, ED25519_KEY_BYTES);
    k->initialized = 1;
    return 0;
}

static inline int crypto_ed25519_export_key(const ed25519_key *k, byte *buf, size_t *size) {
    if (*size < 2 * ED25519_KEY_BYTES) return -1;
    memcpy(buf, k->pub, ED25519_KEY_BYTES);
    memcpy(buf + ED25519_KEY_BYTES, k->priv, ED25519_KEY_BYTES);
    *size = 2 * ED25519_KEY_BYTES;
    return 0;
}

#ifdef __cplusplus
}
#endif
