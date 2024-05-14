#pragma once

#include <cpatools/configuration.hpp>
#include <cpatools/common.hpp>
#include <cpatools/memory.hpp>

namespace cpa {

/// An address type on the target platform
struct address {
  address() = default;
  static constexpr auto zero = 0;
  
  address(memory::target_address_type physicalAddress) {
    addr = memory::bswap(physicalAddress);
  }
  
  address(memory::host_address_type hostAddress) {
    intptr_t offset = intptr_t(hostAddress) - intptr_t(memory::baseAddress);
    addr = hostAddress ? memory::bswap(static_cast<memory::target_address_type>(offset)) : address::zero;
  }
  
  /// Physical hardware address
  inline auto physicalAddress() -> memory::target_address_type {
  #if CPA_PLATFORM == CPA_PLATFORM_GCN
    return memory::bswap(addr) | 0x80000000;
  #else
    return memory::bswap(addr);
  #endif
  };
  
  /// Effective (emulated) address
  inline auto effectiveAddress() -> memory::target_address_type {
  #if CPA_PLATFORM == CPA_PLATFORM_GCN
    return memory::bswap(addr) & 0x7FFFFFFF;
  #else
    return memory::bswap(addr);
  #endif
  }
  
  /// Host platform address
  inline auto hostAddress() -> memory::host_address_type {
    intptr_t offset = intptr_t(memory::baseAddress) + intptr_t(effectiveAddress());
    return valid() ? memory::host_address_type(offset) : nullptr;
  }
  
  /// Is the address non-zero?
  inline auto valid() -> bool {
    return effectiveAddress() != address::zero;
  }
  
  inline operator bool() {
    return valid();
  };
  
  inline auto operator==(address other) -> bool {
    return effectiveAddress() == other.effectiveAddress();
  }
  
  inline operator memory::target_address_type() const {
    return addr;
  };
  
  template<typename T> inline operator T*() {
    return (T*)hostAddress();
  }
  
private:
  memory::target_address_type addr = address::zero;
};
  
/// A type convertible to and from target platform memory
/// T0 = Base type, T1 = Operator type
template<typename T0, typename T1>
struct type {
  type() = default;
  
  template<typename S> inline type(const S value) {
    if constexpr (std::is_same<S, float>::value) {
      data = memory::bswap(*(T0*)&value);
    }
  }
  
  template<typename S> inline type& operator=(const S value) {
    if (writable())
      data = memory::bswap(*(T0*)&value);
    return *this;
  }
  
  inline operator T1() const {
    T0 tmp = memory::bswap(*(T0*)(&data));
    return *(T1*)&tmp;
  }
  
  /// Return the offset of this type in target memory
  inline auto memoryOffset() -> address { return &data; }
  /// Is the type bound to the address space of the target?
  inline auto memoryBound() -> bool { return memory::memoryBound(&data); }
  /// Is the memory of the type writable?
  inline auto writable() -> bool { return !memoryBound() ? true : !memory::readonly; }
  
  inline auto operator+(std::integral auto value) -> T1 { return T1(data) + T1(value); }
  inline auto operator-(std::integral auto value) -> T1 { return T1(data) - T1(value); }
  inline auto operator*(std::integral auto value) -> T1 { return T1(data) * T1(value); }
  inline auto operator/(std::integral auto value) -> T1 { return T1(data) / T1(value); }
  
  inline auto operator+=(T1 other) -> T1 { return *this = *this + other;  }
  inline auto operator-=(T1 other) -> T1 { return *this = *this - other;  }
  inline auto operator*=(T1 other) -> T1 { return *this = *this * other;  }
  inline auto operator/=(T1 other) -> T1 { return *this = *this / other;  }
  
  inline auto operator++(int) -> type { type c = *this; ++(*this); return c; }
  inline auto operator--(int) -> type { type c = *this; --(*this); return c; }
  inline auto operator++() -> type& { *this += 1; return *this; }
  inline auto operator--() -> type& { *this -= 1; return *this; }
  inline auto operator-() -> T1 { type v = *this; return -T1(v); }
  
  inline auto operator|=(T1 other) { *this = *this | other; return *this; }
  inline auto operator&=(T1 other) { *this = *this & other; return *this; }
  inline auto operator^=(T1 other) { *this = *this ^ other; return *this; }
  
  using underlying_type = T1;
private:
  T0 data = 0;
};

using char8   = type<int8_t, int8_t>;
using uchar8  = type<uint8_t, uint8_t>;
using int8    = type<int8_t, int8_t>;
using uint8   = type<uint8_t, uint8_t>;
using int16   = type<int16_t, int16_t>;
using uint16  = type<uint16_t, uint16_t>;
using int32   = type<int32_t, int32_t>;
using uint32  = type<uint32_t, uint32_t>;
using int64   = type<int64_t, int64_t>;
using uint64  = type<uint64_t, uint64_t>;
using float32 = type<uint32_t, float>;

#pragma mark - Pointer
  
/// A pointer exception
class bad_pointer {
  std::string msg;
public:
  std::string what() { return msg; }
  bad_pointer(std::string s) : msg(s) { /* ... */ }
};

/// A pointer
template<typename T = address>
struct pointer {
  pointer() = default;
  pointer(address addr) { ptr = addr; }
  pointer(memory::host_address_type addr) { ptr = addr; }
  
  template<typename S> inline pointer(pointer<S> other) {
    ptr = other.ptr;
  }
  
  template<typename S = T> inline auto pointee() -> S* {
    return ptr ? static_cast<S*>(ptr.hostAddress()) : nullptr;
  }
  
  template<typename S = T> inline operator S*() {
    return pointee<S>();
  }
  
  template<typename S = T> inline const S& operator*() const {
    if (!pointee()) throw bad_pointer("bad pointer dereference");
    return *pointee<S>();
  }

  template<typename S = T> inline S& operator*() {
    if (!pointee()) throw bad_pointer("bad pointer dereference");
    return *pointee<S>();
  }
  
  template<typename S = T> inline auto operator->() -> S* {
    if (!pointee()) throw bad_pointer("bad pointer");
    return pointee();
  }
  
  template<typename S = T> inline S& operator[](auto idx) {
    S* obj = pointee();
    if (!obj) throw bad_pointer("array access into bad pointer");
    return *(obj + idx);
  }
  
  /// Does the pointer point to a valid address?
  inline auto valid() -> bool { return ptr.valid(); }
  /// Memory offset of this pointer
  inline auto offset() -> address { return &ptr; }
  /// Address of the pointee
  inline auto pointeeAddress() -> address { return ptr; }
  
  inline auto operator+(auto offset) -> pointer { return (uint8_t*)pointee() + sizeof(T) * offset; }
  inline auto operator-(auto offset) -> pointer { return (uint8_t*)pointee() - sizeof(T) * offset; }
  inline auto operator++()           -> pointer { return *this = *this + 1;                        }
  inline auto operator++(auto)       -> pointer { auto t = *this; *this = *this + 1; return t;     }
  inline auto operator+=(auto offset)-> pointer { return (*this = *this + offset);                 }
  inline auto operator-=(auto offset)-> pointer { return (*this = *this - offset);                 }
  
  template<typename S = T>
  inline auto operator==(const pointer<S> other) -> bool { return ptr == other.ptr; }
  inline operator bool() { return valid(); }
  
  using underlying_type = T;
  
  address ptr;
};

/// A pointer to a pointer
template<typename T = address>
using doublepointer = pointer<pointer<T>>;

#pragma mark - String

/// A string, zero-terminated unless size specified
template<const size_t size = 0>
struct string {
  string() = default;
  static constexpr bool FixedSize = (size != 0);
  
  /// The length of the string
  inline constexpr auto length() -> size_t {
    if constexpr (FixedSize) {
      return size;
    } else {
      return str.length();
    }
  }
  
  /// The last path component, if such exists
  auto lastPathComponent() -> std::string {
    std::string string = *this;
    size_t idx = string.rfind(':');
    if (idx == std::string::npos) return "";
    return string.substr(idx + 1);
  }
  
  inline auto operator=(std::string string) -> void {
    if constexpr (FixedSize) {
      if (writable()) {
        std::memset(str, 0, size);
        std::memcpy(str, string.data(), size);
      }
    } else {
      str = string;
    }
  }
    
  /// Return the offset of this string in target memory
  inline auto memoryOffset() -> address { return &str; }
  /// Is the string bound to the address space of the target?
  inline auto memoryBound() -> bool { return memory::memoryBound(str); }
  /// Is the memory of the string writable?
  inline auto writable() -> bool { return !memoryBound() ? true : !memory::readonly; }
  
  inline operator std::string() { return std::string((char*)str, length()); }
  inline operator const char*() { return reinterpret_cast<const char*>(str); }
  inline auto operator==(const char *str) -> bool { return std::string(str) == std::string(str); }
  inline auto operator==(std::string str) -> bool { return std::string(str) == std::string(str); }
  
private:
  std::conditional_t<FixedSize, char8[size], std::string> str;
};

};
