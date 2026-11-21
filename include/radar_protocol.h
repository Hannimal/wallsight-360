#pragma once
// Self-contained parsers for LD19 and HLK-LD2450. No Arduino dependencies.
#include <stddef.h>
#include <stdint.h>

namespace radarproto {
static inline uint16_t le16(const uint8_t *p) {
  return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

// LD19 CRC-8: polynomial 0x4D, init 0x00, no reflection, no xor-out.
static inline uint8_t ld19Crc(const uint8_t *bytes, size_t count) {
  uint8_t crc = 0;
  for (size_t i = 0; i < count; ++i) {
    crc ^= bytes[i];
    for (int b = 0; b < 8; ++b) {
      crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x4D)
                         : (uint8_t)(crc << 1);
    }
  }
  return crc;
}

static inline bool validLD19(const uint8_t *packet, size_t len) {
  if (len != 47 || packet[0] != 0x54 || packet[1] != 0x2C) return false;
  if (le16(packet + 4) > 36000 || le16(packet + 42) > 36000) return false;
  return ld19Crc(packet, 46) == packet[46];
}

// LD2450: high bit SET -> positive; high bit CLEAR -> negative.
// This is NOT regular int16_t two's-complement.
static inline int16_t decodeLD2450Signed(uint16_t raw) {
  int16_t magnitude = (int16_t)(raw & 0x7FFF);
  return (raw & 0x8000) ? magnitude : (int16_t)-magnitude;
}

struct Target {
  int16_t xMm;
  int16_t yMm;
  int16_t speedCmS;
  uint16_t resolutionMm;
  bool valid;
};

static inline bool decodeLD2450(const uint8_t *p, size_t len,
                                Target (&targets)[3]) {
  if (len != 30 || p[0] != 0xAA || p[1] != 0xFF || p[2] != 0x03 ||
      p[3] != 0x00 || p[28] != 0x55 || p[29] != 0xCC) return false;
  for (int i = 0; i < 3; ++i) {
    const uint8_t *t = p + 4 + 8 * i;
    targets[i].xMm = decodeLD2450Signed(le16(t));
    targets[i].yMm = decodeLD2450Signed(le16(t + 2));
    targets[i].speedCmS = decodeLD2450Signed(le16(t + 4));
    targets[i].resolutionMm = le16(t + 6);
    targets[i].valid = targets[i].resolutionMm != 0;
  }
  return true;
}
} // namespace radarproto
