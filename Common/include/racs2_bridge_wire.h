/* SPDX-License-Identifier: Apache-2.0 */
#ifndef RACS2_BRIDGE_WIRE_H
#define RACS2_BRIDGE_WIRE_H
#include <stddef.h>
#include <stdint.h>
#include <string.h>
/* Existing RACS2 routing header and opaque body, without body padding. */
#define RACS2_HEADER_SIZE 32u
#define RACS2_BODY_MAX 128u
#define RACS2_MESSAGE_MAX (RACS2_HEADER_SIZE + RACS2_BODY_MAX)
static inline int racs2_wire_valid_size(size_t size)
{
    return size >= RACS2_HEADER_SIZE && size <= RACS2_MESSAGE_MAX;
}
static inline uint16_t racs2_wire_mid(const uint8_t *header)
{
    return (uint16_t)(((uint16_t)header[0] << 8) | header[1]);
}
static inline void racs2_wire_set_mid(uint8_t *header, uint16_t mid)
{
    memset(header, 0, RACS2_HEADER_SIZE);
    header[0] = (uint8_t)(mid >> 8);
    header[1] = (uint8_t)mid;
}
#endif
