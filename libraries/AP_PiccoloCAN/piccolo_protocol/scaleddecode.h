// scaleddecode.h was generated by ProtoGen version 2.18.c

#ifndef _SCALEDDECODE_H
#define _SCALEDDECODE_H

// C++ compilers: don't mangle us
#ifdef __cplusplus
extern "C" {
#endif

/*!
 * \file
 * scaleddecode routines extract scaled numbers from a byte stream.
 *
 * scaleddecode routines extract scaled numbers from a byte stream. The routines
 * in this module are the reverse operation of the routines in scaledencode.
 */
#define __STDC_CONSTANT_MACROS
#include <stdint.h>

//! Inverse scale the bitfield base integer type to a float32
float float32ScaledFromBitfield(unsigned int value, float min, float invscaler);

//! Compute a float scaled from 4 unsigned bytes in big endian order.
float float32ScaledFrom4UnsignedBeBytes(const uint8_t* bytes, int* index, float min, float invscaler);

//! Compute a float scaled from 4 unsigned bytes in little endian order.
float float32ScaledFrom4UnsignedLeBytes(const uint8_t* bytes, int* index, float min, float invscaler);

//! Compute a float scaled from 4 signed bytes in big endian order.
float float32ScaledFrom4SignedBeBytes(const uint8_t* bytes, int* index, float invscaler);

//! Compute a float scaled from 4 signed bytes in little endian order.
float float32ScaledFrom4SignedLeBytes(const uint8_t* bytes, int* index, float invscaler);

//! Compute a float scaled from 3 unsigned bytes in big endian order.
float float32ScaledFrom3UnsignedBeBytes(const uint8_t* bytes, int* index, float min, float invscaler);

//! Compute a float scaled from 3 unsigned bytes in little endian order.
float float32ScaledFrom3UnsignedLeBytes(const uint8_t* bytes, int* index, float min, float invscaler);

//! Compute a float scaled from 3 signed bytes in big endian order.
float float32ScaledFrom3SignedBeBytes(const uint8_t* bytes, int* index, float invscaler);

//! Compute a float scaled from 3 signed bytes in little endian order.
float float32ScaledFrom3SignedLeBytes(const uint8_t* bytes, int* index, float invscaler);

//! Compute a float scaled from 2 unsigned bytes in big endian order.
float float32ScaledFrom2UnsignedBeBytes(const uint8_t* bytes, int* index, float min, float invscaler);

//! Compute a float scaled from 2 unsigned bytes in little endian order.
float float32ScaledFrom2UnsignedLeBytes(const uint8_t* bytes, int* index, float min, float invscaler);

//! Compute a float scaled from 2 signed bytes in big endian order.
float float32ScaledFrom2SignedBeBytes(const uint8_t* bytes, int* index, float invscaler);

//! Compute a float scaled from 2 signed bytes in little endian order.
float float32ScaledFrom2SignedLeBytes(const uint8_t* bytes, int* index, float invscaler);

//! Compute a float scaled from 1 unsigned byte.
float float32ScaledFrom1UnsignedBytes(const uint8_t* bytes, int* index, float min, float invscaler);

//! Compute a float scaled from 1 signed byte.
float float32ScaledFrom1SignedBytes(const uint8_t* bytes, int* index, float invscaler);

#ifdef __cplusplus
}
#endif
#endif
