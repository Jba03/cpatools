#pragma once

#include <cstdint>
#include <fstream>
#include <type_traits>
#include <bit>

namespace cpa::memory {

using host_address_type = void*;
using target_address_type = uint32_t;

static constexpr std::endian endianness = std::endian::big;

/// Engine base address
extern memory::host_address_type baseAddress;
/// Size of the memory space
extern size_t size;
/// Is the memory marked readonly?
extern bool readonly;

static inline auto bswap16(uint16_t value) -> uint16_t {
  return value << 8 | value >> 8;
}

static inline auto bswap32(uint32_t value) -> uint32_t {
  return (uint32_t)bswap16(value) << 16 | bswap16(value >> 16);
}

static inline auto bswap64(uint64_t value) -> uint64_t {
  return (uint64_t)bswap32(value) << 32 | bswap32(value >> 32);
}

/// Swap integral byteorder
template<typename T>
static inline T constexpr bswap(const T v) {
  if constexpr (endianness != std::endian::native && std::is_integral<T>::value) {
    if constexpr (sizeof(T) == 1) return v;
    if constexpr (sizeof(T) == 2) return bswap16(v);
    if constexpr (sizeof(T) == 4) return bswap32(v);
    if constexpr (sizeof(T) == 8) return bswap64(v);
  } else {
    return v;
  }
}

static inline auto memoryBound(memory::host_address_type addr) -> bool {
  return intptr_t(addr) >= intptr_t(baseAddress) && intptr_t(addr) <= intptr_t(baseAddress) + size;
}
  
};
