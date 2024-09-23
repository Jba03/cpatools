#ifndef _CPATOOLS_HPP_
#define _CPATOOLS_HPP_

#define CPA_BIG_ENDIAN 1
#define CPA_LITTLE_ENDIAN 0

#define ENGINE_VERSION_R2 0
#define ENGINE_VERSION_R3 1

#define MAKE_PLATFORM(ID,E) (((E) * 0x40) | ((ID) & 0x3F))
#define GCN     MAKE_PLATFORM(00, CPA_BIG_ENDIAN)
#define PS1     MAKE_PLATFORM(01, CPA_LITTLE_ENDIAN)
#define PS2     MAKE_PLATFORM(02, CPA_LITTLE_ENDIAN)
#define PS3     MAKE_PLATFORM(03, CPA_BIG_ENDIAN)
#define XBOX    MAKE_PLATFORM(04, CPA_LITTLE_ENDIAN)
#define XBOX360 MAKE_PLATFORM(05, CPA_BIG_ENDIAN)
#define PC      MAKE_PLATFORM(06, CPA_LITTLE_ENDIAN)
#define MACOS   MAKE_PLATFORM(07, CPA_BIG_ENDIAN)
#define DC      MAKE_PLATFORM(08, CPA_LITTLE_ENDIAN)
#define NDS     MAKE_PLATFORM(09, CPA_LITTLE_ENDIAN)
#define N3DS    MAKE_PLATFORM(10, CPA_LITTLE_ENDIAN)
#define N64     MAKE_PLATFORM(11, CPA_BIG_ENDIAN)

#define VERSION_ID_BIT  (1 << 31)
#define MAKE_VERSION(ID,V,P) (((((ID) & 0x3F) | (((V) & 0x3) << 6)) << 8) | (P) | VERSION_ID_BIT)
#define R2_PC                   MAKE_VERSION(00, ENGINE_VERSION_R2, GCN)
#define R2_PC_DEMO_1999_08_18   MAKE_VERSION(01, ENGINE_VERSION_R2, GCN)
#define R2_PC_DEMO_1999_09_04   MAKE_VERSION(02, ENGINE_VERSION_R2, PS2)
#define R2_PS1                  MAKE_VERSION(03, ENGINE_VERSION_R2, PS1)
#define R2_PS2                  MAKE_VERSION(04, ENGINE_VERSION_R2, PS2)
#define R2_N64                  MAKE_VERSION(05, ENGINE_VERSION_R2, N64)
#define R2_NDS                  MAKE_VERSION(06, ENGINE_VERSION_R2, NDS)
#define R2_3DS                  MAKE_VERSION(07, ENGINE_VERSION_R2, N3DS)
#define R2_DC                   MAKE_VERSION(08, ENGINE_VERSION_R2, DC)
#define R2_DC_J                 MAKE_VERSION(09, ENGINE_VERSION_R2, DC)
#define R2R_PS2                 MAKE_VERSION(10, ENGINE_VERSION_R2, PS2)
#define R2R_PS2_J               MAKE_VERSION(11, ENGINE_VERSION_R2, PS2)
#define R3_GCN                  MAKE_VERSION(12, ENGINE_VERSION_R3, GCN)
#define R3_GCN_DEMO             MAKE_VERSION(13, ENGINE_VERSION_R3, GCN)
#define R3_PS2                  MAKE_VERSION(14, ENGINE_VERSION_R3, PS2)
#define R3_PS2_DEMO_2002_05_17  MAKE_VERSION(15, ENGINE_VERSION_R3, PS2)
#define R3_PS2_DEMO_2002_08_07  MAKE_VERSION(16, ENGINE_VERSION_R3, PS2)
#define R3_PS2_DEVB_2002_09_06  MAKE_VERSION(17, ENGINE_VERSION_R3, PS2)
#define R3_PS2_DEMO_2002_10_29  MAKE_VERSION(18, ENGINE_VERSION_R3, PS2)
#define R3_PS2_DEMO_2002_12_18  MAKE_VERSION(19, ENGINE_VERSION_R3, PS2)
#define R3_PS3                  MAKE_VERSION(20, ENGINE_VERSION_R3, PS3)
#define R3_PC                   MAKE_VERSION(21, ENGINE_VERSION_R3, PC)
#define R3_PC_DEMO_2002_10_04   MAKE_VERSION(22, ENGINE_VERSION_R3, PC)
#define R3_PC_DEMO_2002_10_21   MAKE_VERSION(23, ENGINE_VERSION_R3, PC)
#define R3_PC_DEMO_2002_12_10   MAKE_VERSION(24, ENGINE_VERSION_R3, PC)
#define R3_PC_DEMO_2003_01_08   MAKE_VERSION(25, ENGINE_VERSION_R3, PC)
#define R3_PC_DEMO_2003_01_29   MAKE_VERSION(26, ENGINE_VERSION_R3, PC)
#define R3_XBOX                 MAKE_VERSION(27, ENGINE_VERSION_R3, XBOX)
#define R3_XBOX360              MAKE_VERSION(28, ENGINE_VERSION_R3, XBOX360)

#if !(CPA_VERSION & VERSION_ID_BIT)
# error Unknown version
#endif

#define PLATFORM_IDENTIFIER_MASK  0x00FF
#define PLATFORM_ENDIANNESS_MASK  0x0040
#define ENGINE_VERSION_MASK       0xC000

#define R2          ENGINE_VERSION_R2
#define R3          ENGINE_VERSION_R3
#define platform    ((CPA_VERSION & PLATFORM_IDENTIFIER_MASK) >> 0)
#define endianness  ((CPA_VERSION & PLATFORM_ENDIANNESS_MASK) >> 6)
#define engine      ((CPA_VERSION & ENGINE_VERSION_MASK) >> 14)
#define game        CPA_VERSION

#define CPA_EXTERN extern

#include <cstdint>
#include <cstdlib>
#include <cassert>
#include <cmath>
#include <array>
#include <bit>
#include <map>
#include <vector>
#include <string>
#include <fstream>

#include <any>
#include <unordered_map>

namespace cpa {

struct _range {
  struct iterator {
    iterator(int64_t position, int64_t step = 0) : position(position), step(step) {}
    auto operator*() const -> int64_t { return position; }
    auto operator!=(const iterator& source) const -> bool { return step > 0 ? position < source.position : position > source.position; }
    auto operator++() -> iterator& { position += step; return *this; }
    
  private:
    int64_t position;
    const int64_t step;
  };
  
  auto begin() const -> iterator { return {origin, stride}; }
  auto end() const -> iterator { return {target}; }
  
  int64_t origin;
  int64_t target;
  int64_t stride;
};

static inline auto range(int64_t sz) {
  return _range { 0, sz, 1 };
}

static constexpr float EPSILON = 1e-6;


#pragma mark - Memory -

#define CPA_MEMORY_READONLY (1 << 0)
#define CPA_MEMORY_EXTERNAL (1 << 1)

namespace memory {
using size_type = size_t;
using host_address_type = void*;
using target_address_type = uint32_t;

#if endianness == CPA_BIG_ENDIAN
static constexpr std::endian _endianness = std::endian::big;
#elif endianness == CPA_LITTLE_ENDIAN
static constexpr std::endian _endianness = std::endian::little;
#else
# error Unknown endianness
#endif

/// The base address of the engine
CPA_EXTERN host_address_type baseAddress;
/// Size of the memory space
CPA_EXTERN size_type size;
/// Flags: CPA_MEMORY_...
CPA_EXTERN unsigned flags;

static inline auto bswap16(uint16_t value) -> uint16_t { return value << 8 | value >> 8; }
static inline auto bswap32(uint32_t value) -> uint32_t { return (uint32_t)bswap16(value) << 16 | bswap16(value >> 16); }
static inline auto bswap64(uint64_t value) -> uint64_t { return (uint64_t)bswap32(value) << 32 | bswap32(value >> 32); }

template<typename T> static inline T constexpr bswap(const T v) {
  if constexpr (_endianness != std::endian::native && std::is_integral<T>::value) {
    if constexpr (sizeof(T) == 1) return v;
    if constexpr (sizeof(T) == 2) return bswap16(v);
    if constexpr (sizeof(T) == 4) return bswap32(v);
    if constexpr (sizeof(T) == 8) return bswap64(v);
  } else {
    return v;
  }
}

/// Returns true if the specified address is within the memory range
static inline auto memoryBound(memory::host_address_type addr) -> bool {
  return intptr_t(addr) >= intptr_t(baseAddress) && intptr_t(addr) <= intptr_t(baseAddress) + size;
}

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
#if platform == GCN
    return memory::bswap(addr) | 0x80000000;
#else
    return memory::bswap(addr);
#endif
  }
  
  /// Effective (emulated) address
  inline auto effectiveAddress() -> memory::target_address_type {
#if platform == GCN
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
  }
  
  inline auto operator==(address other) -> bool {
    return effectiveAddress() == other.effectiveAddress();
  }
  
  inline operator memory::target_address_type() const {
    return addr;
  }
  
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
      data = bswap(*(T0*)&value);
    }
  }
  
  template<typename S> inline type& operator=(const S value) {
    if (writable())
      data = bswap(*(T0*)&value);
    return *this;
  }
  
  inline operator T1() const {
    T0 tmp = bswap(*(T0*)(&data));
    return *(T1*)&tmp;
  }
  
  /// Return the offset of this type in target memory
  inline auto memoryOffset() -> address { return &data; }
  /// Is the type bound to the address space of the target?
  inline auto memoryBound() -> bool { return memory::memoryBound(&data); }
  /// Is the memory of the type writable?
  inline auto writable() -> bool { return !memoryBound() ? true : !(memory::flags & CPA_MEMORY_READONLY); }
  
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

/// A pointer exception
struct bad_pointer {
  std::string what() { return msg; }
  bad_pointer(std::string s) : msg(s) { /* ... */ }
private:
  std::string msg;
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
  /// Dereferenced object
  inline auto dereference() -> T& { return **this; }
  
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

/// A string, zero-terminated unless size specified
template<const size_t size = 0>
struct string {
  string() = default;
  static constexpr bool FixedSize { size != 0 };
  
  inline constexpr auto length() -> size_t {
    if constexpr (FixedSize) {
      return size;
    } else {
      return str.length();
    }
  }
  
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
  inline auto writable() -> bool { return !memoryBound() ? true : !(memory::flags & CPA_MEMORY_READONLY); }
  
  inline operator std::string() { return std::string((char*)str, length()); }
  inline operator const char*() { return reinterpret_cast<const char*>(str); }
  inline auto operator==(const char *str) -> bool { return std::string(str) == std::string(str); }
  inline auto operator==(std::string str) -> bool { return std::string(str) == std::string(str); }
  
private:
  std::conditional_t<FixedSize, char[size], std::string> str;
};

using allocator_function = std::function<void*(size_type sz)>;
using deallocator_function = std::function<void(void*)>;
// To be set by the user...
CPA_EXTERN allocator_function alloc;
CPA_EXTERN deallocator_function dealloc;

struct allocable {
  void* operator new (size_t sz) {
    if (alloc)
      return alloc(sz);
    else
      throw "allocator not set";
  }
  
  void operator delete (void *p) {
    if (dealloc)
      dealloc(p);
  }
};

using userdata_store = std::unordered_map<std::string, std::any>;
CPA_EXTERN std::unordered_map<memory::target_address_type, userdata_store> userdata;

}; /* memory */

using char8   = memory::type<int8_t, int8_t>;
using uchar8  = memory::type<uint8_t, uint8_t>;
using int8    = memory::type<int8_t, int8_t>;
using uint8   = memory::type<uint8_t, uint8_t>;
using int16   = memory::type<int16_t, int16_t>;
using uint16  = memory::type<uint16_t, uint16_t>;
using int32   = memory::type<int32_t, int32_t>;
using uint32  = memory::type<uint32_t, uint32_t>;
using int64   = memory::type<int64_t, int64_t>;
using uint64  = memory::type<uint64_t, uint64_t>;
using float32 = memory::type<uint32_t, float>;

/// A pointer
template<typename T = memory::address>
using pointer = memory::pointer<T>;

/// A pointer to a pointer
template<typename T = memory::address>
using doublepointer = pointer<pointer<T>>;

/// A pointer exception
using bad_pointer = memory::bad_pointer;

/// A string, zero-terminated unless size specified
template<const size_t size = 0>
using string = memory::string<size>;


#pragma mark - Structure -

// namespace Common {
struct stEngineStructure;
struct stEngineTimer;
struct stLanguageStructure;
struct stAlways;
struct stObjectType;
struct stSuperObject;
struct stEngineObject;
struct stStandardGameInfo;
struct st3DData;
struct stCollideSet;
// }

/// Input module
namespace IPT {
struct stInputDevice;
struct stInputStructure;
struct stInputEntryElement;
struct stPadReadingOutput;
}

/// Random module
namespace RND {
struct stRandom;
}

/// Cinematics module
namespace CINE {
struct stCine;
struct stCineInfo;
struct stCineActor;
struct stCineManager;
}

/// Hierarchy module
namespace HIE {
using stSuperObject = stSuperObject;
}

/// Sector module
namespace SECT {
struct stSector;
struct stSectorInfo;
}

/// Physical object module
namespace PO {
struct stPhysicalObject;
}

/// Instantiated physical object module
namespace IPO {
struct stInstantiatedPhysicalObject;
}

/// Intelligence module
namespace AI {
struct stBrain;
struct stMind;
struct stAIModel;
struct stIntelligence;
struct stScriptAI;
struct stMacro;
struct stMacroList;
struct stBehavior;
struct stNodeInterpret;
struct stTreeInterpret;
struct stActionParam;
struct stActionTableEntry;
struct stActionTable;
struct stDsgMem;
struct stDsgVar;
struct stDsgVarInfo;
union uGetSetParam;
}

/// Dynamics module
namespace DNM {
struct stDynam;
struct stDynamics;
struct stDynamicsBaseBlock;
struct stDynamicsAdvancedBlock;
struct stDynamicsComplexBlock;
struct stDynamicsObstacle;
struct stDynamicsReport;
struct stDynamicsRotation;
struct stDynamicsMovement;
struct stDynamicsParsingData;
struct stMACDPID;
enum ObstacleType : unsigned;
}

/// Mechanics module
namespace MEC {
struct stMechanicsObstacle;
struct stMechanicsReport;
}

/// Collision module
namespace COL {
struct stOctreeNode;
struct stOctree;
struct stCollideObject;
struct stPhysicalCollideSet;
struct stColliderInfo;
struct stZdxListEntry;
struct stZdxList;
struct stCsaList;
struct stZoneSetList;
struct stCollideSet;
struct stCollideElementIndexedTriangles;
struct stCollideElementIndexedSphere;
struct stCollideElementIndexedSpheres;
struct stCollideMaterial;
struct stCollisionCase;
struct stIndexedAlignedBox;
struct stCollideElementAlignedBoxes;
struct stGVForCollision;
struct stBoundingSphere;
struct stParallelBox;
enum ElementType : int;
}

/// Geometry module
namespace GEO {
struct stGeometricObject;
struct stVisualSet;
struct stVisualElementIndexedTriangles;
struct stColor;
struct stParallelBox;
union uVisualObject;
enum ElementType : int;
}

/// Geometry morphing module
namespace MOR {
struct stMorphObject;
};

/// Game material module
namespace GMT {
struct stCollideMaterial;
struct stGameMaterial;
}

/// Graphics module
namespace GLI {
struct stVertex2D;
struct stCamera;
struct stLight;
struct stTexture;
struct stAnimatedTextureNode; // todo
struct stMaterial;
struct stMultiTextureMaterial;
}

/// Waypoint module
namespace WP {
struct stWayPoint;
struct stGraph;
struct stGraphNode;
struct stGraphChainList;
}

/// Microstructure module
namespace MS {
struct stMicro;
struct stMSWay;
struct stMSLight;
struct stMSSound;
}

/// Sound module
namespace SND {
struct stRandomElement;
struct stSwitchElement;
struct stBlockEvent;
struct stElement;
struct stDeTune;
struct stEventParametersPlay;
struct stEventParametersExtraAll;
union stEventParameters;
};

using Index3D = uint16;

#define concat(a, b) concat_inner(a, b)
#define concat_inner(a, b) a ## b
#define unique_name(base) concat(base, __LINE__)
#define padding(S) private: uint8_t unique_name(padding) [S]; public:

#pragma pack(push, 1)

/*************************/
/** ``STRUCTURE BEGIN`` **/
/*************************/

struct structure : memory::allocable {
  template<typename T>
  void setUserData(std::string key, T value) {
    memory::target_address_type addr = pointer<>(this).pointeeAddress().effectiveAddress();
    memory::userdata_store& map = memory::userdata[addr];
    map[key] = std::make_any<T>(value);
  }
  
  template<typename T>
  T getUserData(std::string key) {
    memory::target_address_type addr = pointer<>(this).pointeeAddress().effectiveAddress();
    memory::userdata_store& map = memory::userdata[addr];
    return std::any_cast<T>(map[key]);
  }
};

#pragma mark - Common types -

template<unsigned N, typename T = float32>
struct vector {
  vector(float v) { for (auto i : range(N)) data[i] = v; }
  template<typename... Args, std::enable_if_t<sizeof...(Args) == N && sizeof...(Args) != 1 && std::conjunction_v<std::is_convertible<Args, float>...>>* = nullptr>
  vector(Args... args) : data { static_cast<float>(args)... } { /* ... */ }
  template<unsigned N2> vector(std::array<float32, N2>& vec) { for (auto i : range(N)) data[i] = vec[i]; }
  vector() { /* ... */}
  
  inline auto dot(vector<N> v) const {
    float s = 0.0f;
    for (auto i : range(N))
      s += data[i] * v[i];
    return s;
  }
  
  inline auto square() const {
    return dot(*this);
  }
  
  inline auto length() const {
    return sqrt(square());
  }
  
  auto cross(vector<3> v) {
    vector<3> result;
    result.x() = data[1] * v.data[2] - data[2] * v.data[1];
    result.y() = data[2] * v.data[0] - data[0] * v.data[2];
    result.z() = data[0] * v.data[1] - data[1] * v.data[0];
    return result;
  }
  
  auto normalize() {
    vector result = *this;
    if(length() == 0) return result;
    float scale = 1.0f / length();
    for(auto i : range(N)) result[i] *= scale;
    return result;
  }
  
  inline auto isNullVector() -> bool {
    return x() == 0 && y() == 0 && z() == 0;
  }
  
  //access
  inline auto x() -> T& { return data[0]; }
  inline auto y() -> T& { return data[1]; }
  inline auto z() -> T& { return data[2]; }
  inline auto w() -> T& { return data[3]; }
  inline auto xy() -> vector<2> { return vector<2>(x(), y()); }
  inline auto xyz() -> vector<3> { return vector<3>(x(), y(), z()); }
  inline auto& operator[](auto i) { return data[i]; }
  //op
  auto operator +(vector v) { vector result; for(auto i : range(N)) result[i] = data[i] + v[i]; return result; }
  auto operator -(vector v) { vector result; for(auto i : range(N)) result[i] = data[i] - v[i]; return result; }
  auto operator *(vector v) { vector result; for(auto i : range(N)) result[i] = data[i] * v[i]; return result; }
  auto operator /(vector v) { vector result; for(auto i : range(N)) result[i] = data[i] / v[i]; return result; }
  auto operator *(auto   s) { vector result; for(auto i : range(N)) result[i] = data[i] *    s; return result; }
  auto operator /(auto   s) { vector result; for(auto i : range(N)) result[i] = data[i] /    s; return result; }
  auto operator -()         { vector result; for(auto i : range(N)) result[i] =-data[i];        return result; }
  auto operator >(vector v) { bool result = true; for(auto i : range(N)) if (data[i] <= v[i]) result = false; return result; }
  auto operator <(vector v) { bool result = true; for(auto i : range(N)) if (data[i] >= v[i]) result = false; return result; }
  auto operator>=(vector v) { bool result = true; for(auto i : range(N)) if (data[i] <  v[i]) result = false; return result; }
  auto operator<=(vector v) { bool result = true; for(auto i : range(N)) if (data[i] >  v[i]) result = false; return result; }
  auto operator==(vector v) { bool result = true; for(auto i : range(N)) if (data[i] != v[i]) result = false; return result; }
  auto operator!=(vector v) { return !(*this == v); }
  
  auto operator +=(vector v) { for(auto i : range(N)) data[i] = data[i] + v[i]; }
  auto operator -=(vector v) { for(auto i : range(N)) data[i] = data[i] - v[i]; }
  auto operator *=(vector v) { for(auto i : range(N)) data[i] = data[i] * v[i]; }
  auto operator /=(vector v) { for(auto i : range(N)) data[i] = data[i] / v[i]; }
  
private:
  std::array<T, N> data;
};

using stVector2D = vector<2>;
using stVector3D = vector<3>;
using stVector4D = vector<4>;

template<unsigned Rows, unsigned Columns, typename T>
struct matrix {
  matrix() {
    for (auto y : range(Rows)) {
      for (auto x : range(Columns)) {
        (*this)(x, y) = (y == x ? 1.0f : 0.0f);
      }
    }
  };
  
  static auto identity() {
    matrix result;
    for (auto y : range(Rows)) {
      for (auto x : range(Columns)) {
        result(x, y) = (y == x ? 1.0f : 0.0f);
      }
    }
    return result;
  }
  
  inline auto operator()(auto row, auto col) -> T& {
    return m[col + row * Columns];
  }
  
  inline auto operator[](auto index) -> T& {
    return m[index];
  }
  
  template <unsigned R, unsigned C>
  auto operator*(matrix<R, C, T> src) {
    static_assert(Columns == C);
    matrix<Rows, Columns, T> result;
    for (auto y : range(Rows)) {
      for (auto x : range(C)) {
        T sum = 0.0f;
        for (auto z : range(Columns)) {
          sum += src(y, z) * (*this)(z, x);
        }
        result(y,x) = sum;
      }
    }
    return result;
  }
  
  template <unsigned R, unsigned C>
  auto operator*=(matrix<R, C, T> m) {
    return (*this = *this * m);
  }
  
  auto operator*(stVector4D v) -> stVector4D {
    stVector4D result;
    for (auto y : range(Rows)) {
      result[y] = 0.0f;
      for (auto x : range(Columns)) {
        result[y] += (*this)(x,y) * v[x];
      }
    }
    return result;
  }
  
  auto operator*(stVector3D v) -> stVector4D {
    return ((*this) * stVector4D(v.x(), v.y(), v.z(), 1.0f));
  }
  
  static auto makeTranslation(stVector3D p) {
    matrix result = identity();
    for (auto i : range(3)) result(Rows-1,i) = p[i];
    return result;
  }
  
  static auto makeScale(stVector3D p) {
    matrix result = identity();
    for (auto i : range(3)) result(i,i) = p[i];
    return result;
  }
  
  static matrix makeRotationX(float radians) {
    matrix result = identity();
    result(1,1) = cos(radians);
    result(1,2) = sin(radians);
    result(2,1) = -sin(radians);
    result(2,2) = cos(radians);
    return result;
  }
  
  static matrix makeRotationY(float radians) {
    matrix result = identity();
    result(0,0) = cos(radians);
    result(0,2) = -sin(radians);
    result(2,0) = sin(radians);
    result(2,2) = cos(radians);
    return result;
  }
  
  static matrix makeRotationZ(float radians) {
    matrix result = identity();
    result(0,0) = cos(radians);
    result(0,1) = sin(radians);
    result(1,0) = -sin(radians);
    result(1,1) = cos(radians);
    return result;
  }
  
  static auto makePerspective(float fovY, float aspect, float near, float far) {
    float ct = 1.0f / std::tan(fovY / 2.0f);
    matrix<4,4,T> result = identity();
    result(0,0) = ct / aspect;
    result(1,1) = ct;
    result(2,2) = (far + near) / (near - far);
    result(2,3) = -1.0f;
    result(3,2) = (2.0f * far * near) / (near - far);
    result(3,3) = 0.0f;
    return result;
  }
  
  static auto makeLookAt(stVector3D eye, stVector3D center, stVector3D up) {
    stVector3D n = (eye - center).normalize();
    stVector3D u = up.cross(n).normalize();
    stVector3D v = n.cross(u);
    
    float nnx = (-u).dot(eye);
    float nny = (-v).dot(eye);
    float nnz = (-n).dot(eye);
    
    matrix<4,4,T> result;
    result(0,0) = u.x();
    result(0,1) = v.x();
    result(0,2) = n.x();
    result(0,3) = 0.0f;
    result(1,0) = u.y();
    result(1,1) = v.y();
    result(1,2) = n.y();
    result(1,3) = 0.0f;
    result(2,0) = u.z();
    result(2,1) = v.z();
    result(2,2) = n.z();
    result(2,3) = 0.0f;
    result(3,0) = nnx;
    result(3,1) = nny;
    result(3,2) = nnz;
    result(3,3) = 1.0f;
    
    return result;
  }
  
  auto transpose() -> matrix {
    matrix<Rows, Columns, T> result;
    for (auto y : range(Rows)) {
      for (auto x : range(Columns)) {
        result(x,y) = (*this)(y,x);
      }
    }
    return result;
  }
  
  auto inverse() -> matrix<4,4,T> {
    matrix<4,4,T> result;
    
    float s0 = (*this)(0,0) * (*this)(1,1) - (*this)(1,0) * (*this)(0,1);
    float s1 = (*this)(0,0) * (*this)(1,2) - (*this)(1,0) * (*this)(0,2);
    float s2 = (*this)(0,0) * (*this)(1,3) - (*this)(1,0) * (*this)(0,3);
    float s3 = (*this)(0,1) * (*this)(1,2) - (*this)(1,1) * (*this)(0,2);
    float s4 = (*this)(0,1) * (*this)(1,3) - (*this)(1,1) * (*this)(0,3);
    float s5 = (*this)(0,2) * (*this)(1,3) - (*this)(1,2) * (*this)(0,3);
    float c5 = (*this)(2,2) * (*this)(3,3) - (*this)(3,2) * (*this)(2,3);
    float c4 = (*this)(2,1) * (*this)(3,3) - (*this)(3,1) * (*this)(2,3);
    float c3 = (*this)(2,1) * (*this)(3,2) - (*this)(3,1) * (*this)(2,2);
    float c2 = (*this)(2,0) * (*this)(3,3) - (*this)(3,0) * (*this)(2,3);
    float c1 = (*this)(2,0) * (*this)(3,2) - (*this)(3,0) * (*this)(2,2);
    float c0 = (*this)(2,0) * (*this)(3,1) - (*this)(3,0) * (*this)(2,1);
    
    float const det = (s0 * c5 - s1 * c4 + s2 * c3 + s3 * c2 - s4 * c1 + s5 * c0);
    float const invdet = 1.0f / det;
    
    if (det == 0.0f) throw "non-invertible matrix";
    
    result(0,0) = ( (*this)(1,1) * c5 - (*this)(1,2) * c4 + (*this)(1,3) * c3) * invdet;
    result(0,1) = (-(*this)(0,1) * c5 + (*this)(0,2) * c4 - (*this)(0,3) * c3) * invdet;
    result(0,2) = ( (*this)(3,1) * s5 - (*this)(3,2) * s4 + (*this)(3,3) * s3) * invdet;
    result(0,3) = (-(*this)(2,1) * s5 + (*this)(2,2) * s4 - (*this)(2,3) * s3) * invdet;
    result(1,0) = (-(*this)(1,0) * c5 + (*this)(1,2) * c2 - (*this)(1,3) * c1) * invdet;
    result(1,1) = ( (*this)(0,0) * c5 - (*this)(0,2) * c2 + (*this)(0,3) * c1) * invdet;
    result(1,2) = (-(*this)(3,0) * s5 + (*this)(3,2) * s2 - (*this)(3,3) * s1) * invdet;
    result(1,3) = ( (*this)(2,0) * s5 - (*this)(2,2) * s2 + (*this)(2,3) * s1) * invdet;
    result(2,0) = ( (*this)(1,0) * c4 - (*this)(1,1) * c2 + (*this)(1,3) * c0) * invdet;
    result(2,1) = (-(*this)(0,0) * c4 + (*this)(0,1) * c2 - (*this)(0,3) * c0) * invdet;
    result(2,2) = ( (*this)(3,0) * s4 - (*this)(3,1) * s2 + (*this)(3,3) * s0) * invdet;
    result(2,3) = (-(*this)(2,0) * s4 + (*this)(2,1) * s2 - (*this)(2,3) * s0) * invdet;
    result(3,0) = (-(*this)(1,0) * c3 + (*this)(1,1) * c1 - (*this)(1,2) * c0) * invdet;
    result(3,1) = ( (*this)(0,0) * c3 - (*this)(0,1) * c1 + (*this)(0,2) * c0) * invdet;
    result(3,2) = (-(*this)(3,0) * s3 + (*this)(3,1) * s1 - (*this)(3,2) * s0) * invdet;
    result(3,3) = ( (*this)(2,0) * s3 - (*this)(2,1) * s1 + (*this)(2,2) * s0) * invdet;
    
    return result;
  }
  
  inline auto translation() -> stVector3D& {
    return *(stVector3D*)&(*this)(Rows-1,0);
  }
  
  inline auto scale(bool ref = false) {
    if (ref) return vector<3, float32*>(&(*this)(0,0), &(*this)(1,1), &(*this)(2,2));
    return stVector3D((*this)(0,0), (*this)(1,1), (*this)(2,2));
  }
  
  std::array<T, Rows * Columns> m;
};

using stMatrix3D = matrix<3, 3, float32>;
using stMatrix4D = matrix<4, 4, float32>;

#pragma mark - Containers

enum class LinkedListType {
  Single,
  Double,
};

/// A linked list
template<typename T = uint32, enum LinkedListType K = LinkedListType::Single>
struct LinkedList {
  pointer<T> first;
  std::conditional_t<K == LinkedListType::Single, pointer<T>, std::monostate> last;
  int32 numEntries;
  
  template<typename F> void forEach(const F& f, void *userdata = nullptr) {
    //pointer<T> c = first;
//    for (auto i : range(numEntries)) {
//      f(first+i, userdata);
//    }
    try {
      for (T *c = first; c; c = c->next) {
        f(c, userdata);
      }
    } catch (...) {}
  }
  
  struct iterator {
    iterator(pointer<T> pos) : _pos(pos) { /* ... */ }
    auto operator*() -> pointer<T> { return _pos; }
    auto operator!=(iterator& source) -> bool { return _pos != source._pos; }
    auto operator++() -> iterator& { if (_pos) _pos = _pos->next; return *this; }
  private:
    pointer<T> _pos;
  };
  
  auto begin() -> iterator { return first; }
  auto end() -> iterator { return last; }
};

template<typename T = uint32> using stSingleLinkedList = LinkedList<T, LinkedListType::Single>;
template<typename T = uint32> using stDoublyLinkedList = LinkedList<T, LinkedListType::Single>;
template<typename T = uint32> using stLinkedList = stDoublyLinkedList<T>;

template<typename T>
struct LinkedListElement {
  pointer<T> data;
  pointer<LinkedListElement> next;
  pointer<LinkedListElement> prev;
  pointer<stLinkedList<>> list;
};

#pragma mark - stTransform

/// World transform
struct stTransform : structure {
  enum Type {
    Uninitialized = 0,
    Identity = 1,
    Translate = 2,
    Zoom = 3,
    Scale = 4,
    Rotation = 5,
    RotationZoom = 6,
    RotationScale = 7,
    ComplexRotationScale = 8,
    Undefined = 9,
  };
  
  stTransform() = default;
  stTransform(uint32 _type, stMatrix4D T = stMatrix4D(), stVector4D _scale = stVector4D(1.0f, 1.0f, 1.0f, 1.0f)) : type(_type), matrix(T), scale(_scale) { /* ... */ }
  
  /// Type of the transform
  uint32 type = Type::Uninitialized;
  /// Transform matrix
  stMatrix4D matrix = stMatrix4D::identity();
  /// Scale parameter
  stVector4D scale;
  
  /// Translation vector
  auto translation() -> stVector3D& {
    return matrix.translation();
  }
  
  /// Get rotation vectors if the type is `transformTypeRotation`
  auto getRotation(stVector3D& i, stVector3D& j, stVector3D& k) -> bool {
    if (static_cast<uint32_t>(type) == Type::Rotation) {
      i = *(stVector3D*)&matrix(0,0);
      j = *(stVector3D*)&matrix(1,0);
      k = *(stVector3D*)&matrix(2,0);
      return true;
    } else {
      return false;
    }
  }
  
  auto operator*(stVector3D v) -> stVector3D {
    return (matrix * stVector4D(v.x(), v.y(), v.z(), 1.0f)).xyz();
  }
  
  auto operator*(stVector4D v) -> stVector4D {
    return matrix * v;
  }
  
  auto operator*(stTransform other) -> stTransform {
    // TODO: Also transform the scale here?
    stTransform T(type, matrix * other.matrix, scale);
    return T;
  }
  
  auto inverse() -> stTransform {
    // TODO: Also transform the scale here?
    stTransform T(type, matrix.inverse(), scale);
    return T;
  }
  
  auto rotateVector(stVector3D v) -> stVector3D {
    uint32_t type = this->type;
    if (type == Rotation) {
      return *this * v;
    } else if (type < Rotation) {
      return v;
    } else {
      // TODO
      return *this * v;
    }
  }
  
  inline auto typeName() -> std::string {
    switch (static_cast<uint32_t>(type)) {
      case Uninitialized:
        return "Uninitialized";
      case Identity:
        return "Identity";
      case Translate:
        return "Translate";
      case Zoom:
        return "Zoom";
      case Scale:
        return "Scale";
      case Rotation:
        return "Rotation";
      case RotationZoom:
        return "RotationZoom";
      case RotationScale:
        return "RotationScale";
      case ComplexRotationScale:
        return "ComplexRotationScale";
      case Undefined:
        return "Undefined";
      default:
        return "Invalid";
    }
  }
};

struct stAlwaysModelList : structure {
  pointer<stAlwaysModelList> next;
  pointer<stAlwaysModelList> prev;
  pointer<stLinkedList<stAlwaysModelList>> parentList;
  int32 objectModelType;
  pointer<stEngineObject> alwaysObject;
};

struct stAlways : structure {
  uint32 numAlways;
  stDoublyLinkedList<stAlwaysModelList> alwaysModels;
  pointer<stSuperObject> alwaysSuperobject;
  pointer<stEngineObject> alwaysActors;
  pointer<stSuperObject> alwaysGeneratorSuperobjects;
};

enum ObjectType {
  Family = 0,
  Model = 1,
  Instance = 2
};

/// Object identifier
struct stObjectTypeElement : structure {
  /// Next object type element
  pointer<stObjectTypeElement> next;
  /// Previous object type element
  pointer<stObjectTypeElement> prev;
  /// Pointer to list containing this element
  pointer<stDoublyLinkedList<stObjectTypeElement>> list;
  /// Pointer to name of the element
  pointer<string<>> name;
  /// Priority
  uint8 priority;
  /// Identifier
  uint8 identifier;
  /// Padding
  padding(2)
};

/// Global object type table
struct stObjectType : structure {
  /// Family object types
  stDoublyLinkedList<stObjectTypeElement> family;
  /// Model object types
  stDoublyLinkedList<stObjectTypeElement> model;
  /// Instance object types
  stDoublyLinkedList<stObjectTypeElement> instance;
};

#pragma mark - Engine

/// High-resolution counter
struct stTimerCount : structure {
  uint32 low;
  uint32 high;
};

/// Global engine timer
struct stEngineTimer : structure {
  /// Current frame of the level
  uint32 currentFrame;
  /// Internal timer ID handle
  int16 timerHandle;
  /// Padding
  padding(2)
  /// Current counter
  uint32 currentCount;
  uint32 deltaCount;
  /// Miscellaneous counters
  uint32 counter[16];
  uint32 usefulDeltaTime;
  uint32 pauseTime;
  /// Optimal length of one frame, in seconds
  float32 frameLength;
  /// Total time the game was played
  stTimerCount totalRealTime;
  /// Total time the game was paused
  stTimerCount totalPauseTime;
  /// Number of ticks per millisecond
  uint32 ticksPerMs;
};

enum EngineMode {
  Invalid               = 0,
  Initialize            = 1,
  Deinitialize          = 2,
  InitializeGameplay    = 3,
  DeinitializeGameplay  = 4,
  EnterLevel            = 5,
  ChangeLevel           = 6,
  Gameplay              = 9,
};

enum InputMode {
  Normal = 0,
  Commands = 1,
};

/// Engine structure
struct stEngineStructure : structure {
  /// Engine mode
  uint8 mode;
  /// Current level name
  string<30> currentLevelName;
  /// Name of next level to be loaded
  string<30> nextLevelName;
  /// Name of entry level
  string<30> firstLevelName;
  /// Input mode (`engineInputMode#`)
  uint8 inputMode;
  uint8 displayFixMode;
  padding(3)
  uint32 displayMode;
  stEngineTimer timer;
  uint8 multimodePlayerCount;
  uint8 multimodeColumnCount;
  uint8 multimodeMiniScreenRatio;
  padding(1)
  pointer<stSuperObject> currentMainPlayers[4];
  int16 gldDevice;
  int16 gldViewport[5];
  padding(5 * 28 * 4) /* viewport attributes */
  pointer<GLI::stCamera> viewportCamera[5];
  int16 gldFixViewport[5];
  padding(2)
  padding(5 * 28 * 4) /* fix viewport attributes */
  padding(5 * 2 * 4) /* fix 3d attributes */
  pointer<GLI::stCamera> fixCamera[5];
  padding(5 * 2 * 4) /* game 3d attributes */
  pointer<> viewportArray;
  stDoublyLinkedList<> cameraList;
  pointer<> drawSem;
  stDoublyLinkedList<> familyList;
  stDoublyLinkedList<> alwaysList;
  stDoublyLinkedList<stSuperObject> mainCharacterList;
  pointer<stSuperObject> standardCamera;
  pointer<stSuperObject> debugCamera;
  pointer<> languageStructure;
  pointer<> levelFilenameList;
  stTransform mainActorTransform;
  stTransform mainCameraTransform;
  int32 submapNumber;
  
  uint8 paused;
  uint8 paused2;
  uint8 doGameSave;
  
  /// List of level names
  string<30> levelNames[150];
  /// List of demo save names
  string<12> demoNames[30];
  /// List of demo level names
  string<12> demoLevelNames[30];
  /// Number of demo levels
  uint8 demoCount;
  /// Number of real levels
  uint8 levelCount;
  uint8 currentLevel;
  uint8 previousLevel;
  uint8 previousLevelExitID;
  uint8 globalLevelCounter;
  /// Is in demo mode?
  uint8 demoMode;
  /// Current language index
  uint8 currentLanguage;
  /// Number of languages
  uint8 languageCount;
  uint8 engineFrozen;
  uint8 resurrection;
  /// Camera mode
  uint8 cameraMode;
  uint8 currentImportance;
  uint8 numSuperObjectsAllocated;
  uint8 numSuperObjectsLoaded;
  uint8 numNonPersistentSOLinks;
  /// Padding
  padding(9);
  ///
  doublepointer<> superObjectLinks;
  pointer<WP::stGraphChainList> graphList;
  pointer<CINE::stCineManager> cineManager;
  
  /// Load level by name
  inline auto loadLevel(std::string levelName) -> void;
};


#pragma mark - IPT -

/// Structure for ReadAnalogJoystick function
struct IPT::stPadReadingOutput : structure {
  /// The world vector the joystick value translates to
  stVector3D globalVector;
  int16 horizontalAxis;
  int16 verticalAxis;
  float32 analogForce;
  float32 trueAnalogForce;
  float32 rotationAngle;
  /// Strafe sector (0-7 clockwise)
  int32 strafeSector;
};

struct IPT::stInputDevice : structure {
  uint8 valid;
  padding(3)
  pointer<> handle;
  uint8 joypadCounter[44];
  int8 joyMinX;
  int8 joyMaxX;
  int8 joyMinY;
  int8 joyMaxY;
  int8 joyCenterX;
  int8 joyCenterY;
  padding(2)
  pointer<stInputEntryElement> joyAxisX;
  pointer<stInputEntryElement> joyAxisY;
  pointer<stInputEntryElement> joyAxisZ;
  pointer<stInputEntryElement> joyAxisR;
  pointer<stInputEntryElement> keyUp;
  pointer<stInputEntryElement> keyDown;
  pointer<stInputEntryElement> keyLeft;
  pointer<stInputEntryElement> keyRight;
  pointer<stInputEntryElement> keySpeedUp;
  uint8 noKeyboardInertia;
  padding(3)
  pointer<stInputEntryElement> joyButton[16];
  pointer<stInputEntryElement> keyButton[16];
  stPadReadingOutput padReadOutput;
};

struct IPT::stInputEntryElement : structure {
  padding(6 * 4) /* ? */
  uint32 numKeywords;
  pointer<> keywordArray;
  pointer<string<>> actionName;
  pointer<string<>> entryName;
  int32 state;
  float32 analogValue;
  int8 active;
  padding(3)
};

struct IPT::stInputStructure : structure {
  uint8 onePadActivate;
  padding(3)
  stInputDevice device[18];
  uint8 keyboardCounter[256];
  uint8 keyboardType;
  uint8 mouseButtonCounter[9];
  padding(2)
  uint32 numEntries;
  pointer<stInputEntryElement> entries;
  uint32 numCommands;
  pointer<> commands;
  int16 eventSize;
  padding(2)
  pointer<> historicEvent;
  uint8 oneActionValidated;
  string<78> line;
  string<78> lineInternal;
  string<78> lineSearch;
  string<78> lineLast[10];
  padding(1)
  int32 historicIndex;
  pointer<> searchedCommand;
  pointer<stInputEntryElement> commandModeEntrySwap;
  pointer<stInputEntryElement> commandModeEntryClear;
};

#pragma mark - RND -

#define RND_TableIndexCount 0x0032
#define RND_TableCount 0x2710
#define RND_DefaultIndex 0x0000

struct RND::stRandom : structure {
  /// Size of the table
  uint32 tableSize;
  /// Indices into the table
  uint32 tableIndices[RND_TableIndexCount];
  /// Last index from tableIndices
  uint32 lastIndex;
  /// Largest number present in table
  uint32 tableMax;
  /// 1.0 / tableMax
  float32 tableMaxInverse;
  /// Random number table
  pointer<uint32> table;
  
  /// Index the random number table by absolute offset
  int32_t index(unsigned i) {
    uint32_t* T = table;
    return T ? ((T[i % RND_TableCount] >> 16) & 0x7FFF) : 0;
  }
  
  /// Index the random number table using an index from tableIndices, optionally offset
  int32_t indexRelative(unsigned TableIndicesIdx, unsigned Offset) {
    return index(uint32_t(tableIndices[TableIndicesIdx]) + Offset);
  }
  
  /// Simulate `Count` calls into the RND table, bounding the value by `Min` and `Max`
  int32_t call(unsigned const Count, unsigned const Min, unsigned const Max, unsigned const Index = RND_DefaultIndex) {
    int32_t n, v = 0;
    for (n = 0; n < Count; n++)
      v = (Min + ((Max + 1 - Min) * indexRelative(Index, n)) / (tableMax + 1));
    return v;
  }
};

#pragma mark - 3D

struct stAnim3D : structure {
  
};

struct stSubAnim : structure {
  pointer<stAnim3D> subAnim;
};

struct stActiveSubAnim : structure {
  pointer<stActiveSubAnim> next;
  pointer<stActiveSubAnim> prev;
  stDoublyLinkedList<stActiveSubAnim> parent;
  pointer<stSubAnim> subAnim;
  pointer<uint8> eventActivation;
  uint32 startFrame;
  uint32 customBits;
  uint32 loop;
  float32 frame;
  uint8 nextEvent;
  uint8 stop;
  uint8 merge;
  padding(1);
};


#pragma mark - CINE -

/// Actor state in a cinematic
struct CINE::stCineActor : structure {
  stSubAnim subAnim;
  pointer<stActiveSubAnim> activeSubAnim;
  string<255> animationName;
  padding(1)
  pointer<stEngineObject> actor;
  pointer</*stState*/> stateAfterCine;
  pointer</*stState*/> stateDuringCine;
  pointer</*stState*/> stateForActorTmp;
  uint8 skipAI;
  uint8 skipMechanics;
  uint8 previousAIState;
  uint8 previousMechanicsState;
  uint8 repeatAnimation;
  int8 animationSpeed;
  uint8 actorMoveAtStart;
  padding(1)
  pointer<stSuperObject> superobject;
  uint8 actorMoveAtEnd;
  padding(1)
  uint16 channel;
  uint8 playingAnimation;
  uint8 isSubAnim;
  uint8 changeIntelligenceAtStart;
  padding(1)
  pointer<AI::stBehavior> intelligenceStart;
  uint8 changeReflexAtStart;
  padding(3)
  pointer<AI::stBehavior> reflexStart;
  uint8 changeIntelligenceAtEnd;
  padding(3)
  pointer<AI::stBehavior> intelligenceEnd;
  uint8 changeReflexAtEnd;
  padding(3)
  pointer<AI::stBehavior> reflexEnd;
  uint8 startUseSoundRequest;
  uint8 startUseVoiceRequest;
  uint8 startUseMusicRequest;
  uint8 startUseAmbianceRequest;
  pointer<SND::stBlockEvent> startSoundRequest;
  pointer<SND::stBlockEvent> startVoiceRequest;
  pointer<SND::stBlockEvent> startMusicRequest;
  pointer<SND::stBlockEvent> startAmbianceRequest;
  uint8 endUseSoundRequest;
  uint8 endUseVoiceRequest;
  uint8 endUseMusicRequest;
  uint8 endUseAmbianceRequest;
  pointer<SND::stBlockEvent> endSoundRequest;
  pointer<SND::stBlockEvent> endVoiceRequest;
  pointer<SND::stBlockEvent> endMusicRequest;
  pointer<SND::stBlockEvent> endAmbianceRequest;
  pointer<stCine> cinematic;
  stDoublyLinkedList<> channelLink;
  pointer<stCineActor> next;
  pointer<stCineActor> prev;
  pointer<stDoublyLinkedList<stCineActor>> parents;
};

/// A cinematic
struct CINE::stCine : structure {
  /// Actors controlling the cinematic
  stDoublyLinkedList<stCineActor> actors;
  /// Next cinematic in this list
  pointer<stCine> next;
  /// Previous cinematic in this list
  pointer<stCine> prev;
  /// Parent list
  pointer<stDoublyLinkedList<stCine>> parentList;
  /// Is the cinematic playing?
  uint8 playing;
  /// Padding
  padding(3)
  /// Event identifier
  uint32 event;
  /// Name of the cinematic
  string<255> name;
};

/// Cinematics state manager
struct CINE::stCineManager : structure {
  /// List of level cinematics
  stDoublyLinkedList<stCine> cineList;
  /// Padding
#if platform == PS2
  padding(4)
#endif
  /// Force camera transform
  stTransform fixedCameraTransform;
  /// Padding
#if platform == PS2
  padding(2)
#endif
  /// Currently active cutscene camera
  pointer<stSuperObject> activeCamera;
};


#pragma mark - DNM -

// Control flags
#define DNM_Flag_Animation              (1 <<  0) // Use animation speed?
#define DNM_Flag_Collide                (1 <<  1) // Enable geometry collision?
#define DNM_Flag_Gravity                (1 <<  2) // Enable gravity?
#define DNM_Flag_Tilt                   (1 <<  3) // Tilt
#define DNM_Flag_Gi                     (1 <<  4) // Hanging from ceiling
#define DNM_Flag_OnGround               (1 <<  5) // Is on ground
#define DNM_Flag_Climb                  (1 <<  6) // Climbing
#define DNM_Flag_CollisionControl       (1 <<  7) // Use dynamics param when colliding?
#define DNM_Flag_KeepWallZSpeed         (1 <<  8) // Preseve Z-axis momentum when colliding with a wall?
#define DNM_Flag_SpeedLimit             (1 <<  9) // Limit speed
#define DNM_Flag_Inertia                (1 << 10) // Has inertia?
#define DNM_Flag_Stream                 (1 << 11) // Is affected by a stream?
#define DNM_Flag_StuckToPlatform        (1 << 12) // No slide on platform
#define DNM_Flag_IsScale                (1 << 13) // Use scale parameters
#define DNM_Flag_SpeedImposeAbsolute    (1 << 14) // Impose absolute speed
#define DNM_Flag_SpeedProposeAbsolute   (1 << 15) // Propose absolute speed
#define DNM_Flag_SpeedAddAbsolute       (1 << 16) // Add absolute speed
#define DNM_Flag_SpeedImposeX           (1 << 17) // Impose absolute X-speed (after inertia & gravity)
#define DNM_Flag_SpeedImposeY           (1 << 18) // Impose absolute Y-speed (after inertia & gravity)
#define DNM_Flag_SpeedImposeZ           (1 << 19) // Impose absolute Z-speed (after inertia & gravity)
#define DNM_Flag_SpeedProposeX          (1 << 20) // Propose absolute X-speed (before inertia & gravity)
#define DNM_Flag_SpeedProposeY          (1 << 21) // Propose absolute Y-speed (before inertia & gravity)
#define DNM_Flag_SpeedProposeZ          (1 << 22) // Propose absolute Z-speed (before inertia & gravity)
#define DNM_Flag_SpeedAddX              (1 << 23) // Add absolute X-speed
#define DNM_Flag_SpeedAddY              (1 << 24) // Add absolute Y-speed
#define DNM_Flag_SpeedAddZ              (1 << 25) // Add absolute Z-speed
#define DNM_Flag_LimitX                 (1 << 26) //
#define DNM_Flag_LimitY                 (1 << 27) //
#define DNM_Flag_LimitZ                 (1 << 28) //
#define DNM_Flag_ImposeRotation         (1 << 29) // Impose axis rotation
#define DNM_Flag_PlatformLock           (1 << 30) // Keep on platform
#define DNM_Flag_ImposeTranslation      (1 << 31) // Impose translation

// Info/verification flags
#define DNM_EndFlag_BaseSize              (1 <<  0) // Base size dynamics
#define DNM_EndFlag_AdvancedSize          (1 <<  1) // Advanced size dynamics
#define DNM_EndFlag_ComplexSize           (1 <<  2) // Complex size dynamics
#define DNM_EndFlag_Reserved              (1 <<  3)
#define DNM_EndFlag_MechanicsChanged      (1 <<  4)
#define DNM_EndFlag_PlatformCrash         (1 <<  5)
#define DNM_EndFlag_CanFall               (1 <<  6)
#define DNM_EndFlag_IsInit                (1 <<  7)
#define DNM_EndFlag_SpiderMechanic        (1 <<  8)
#define DNM_EndFlag_IsShoot               (1 <<  9)
#define DNM_EndFlag_SafeValid             (1 << 10)
#define DNM_EndFlag_ComputeInvertMatrix   (1 << 11)
#define DNM_EndFlag_ChangeScale           (1 << 12)
#define DNM_EndFlag_Exec                  (1 << 13)
#define DNM_EndFlag_CollisionReport       (1 << 14)
#define DNM_EndFlag_NoGravity             (1 << 15)
#define DNM_EndFlag_Stop                  (1 << 16)
#define DNM_EndFlag_SlidingGround         (1 << 17)
#define DNM_EndFlag_Always                (1 << 18)
#define DNM_EndFlag_Crash                 (1 << 19)
#define DNM_EndFlag_Swim                  (1 << 20)
#define DNM_EndFlag_NeverFall             (1 << 21)
#define DNM_EndFlag_Hanging               (1 << 22)
#define DNM_EndFlag_WallAdjust            (1 << 23)
#define DNM_EndFlag_ActorMove             (1 << 24)
#define DNM_EndFlag_ForceSafeWalk         (1 << 25)
#define DNM_EndFlag_DontUseNewMechanic    (1 << 26)

// Obstacle types
enum DNM::ObstacleType : unsigned {
  None        = 0x00000000,
  Ground      = 0x00000001,
  Wall        = 0x00000004,
  Ceiling     = 0x00000010,
  Water       = 0x00000040,
  ForceMobile = 0x00000080,
  Mobile      = 0x00010000,
  Error       = 0x80000000,
};
  
/// Axis-angle
struct DNM::stDynamicsRotation : structure {
  float32 angle;
  stVector3D axis;
};

/// Dynamics base block
struct DNM::stDynamicsBaseBlock : structure {
  /// Type of the object
  int32 objectType;
  /// Current mechanics ID card
  pointer<> idcard;
  /// Mechanics control flags
  uint32 flags;
  /// Mechanics info/verification flags
  uint32 endFlags;
  /// Gravity
  float32 gravity;
  /// Slope limit (1.0f)
  float32 slopeLimit;
  /// Wall/ground limit (45 degrees)
  float32 slopeCosine;
  /// Ground slide factor
  float32 slide;
  /// Rebound factor
  float32 rebound;
  /// Impose absolute speed (after inertia and gravity calculations)
  stVector3D imposeSpeed;
  /// Propose speed (before inertia and gravity calculations)
  stVector3D proposeSpeed;
  /// Previous speed
  stVector3D previousSpeed;
  /// Actor scale
  stVector3D scale;
  /// Animation-specific speed
  stVector3D animationProposeSpeed;
  /// Previous safe translation
  stVector3D safeTranslation;
  /// Additional translation
  stVector3D addTranslation;
  
#if engine == R3 && platform == GCN
  /// Padding
  padding(8)
#endif
  
  /// Previous transform
  stTransform previousTransform;
  /// Current transform
  stTransform currentTransform;
  /// Impose absolute rotation
  stMatrix3D imposedRotation;
  /// Previous number of frames
  uint8 numFrames;
  /// Padding
  padding(3)
  /// Collision report copied from mechanics
  pointer<stDynamicsReport> report;
  
#if engine == R3 && platform == PS2
  /// Padding
  padding(8)
#endif
};

/// Dynamics advanced block
struct DNM::stDynamicsAdvancedBlock : structure {
  /// Inertia (NOTE: originally component-separated)
  stVector3D inertia;
  /// Priority of stream
  float32 streamPriority;
  /// Stream effect factor
  float32 streamFactor;
  /// Slide factor (NOTE: originally component-separated)
  stVector3D slideFactor;
  /// Previous slide
  float32 previousSlide;
  /// Speed limit
  stVector3D maxSpeed;
  /// Speed of stream
  stVector3D streamSpeed;
  /// Speed to add
  stVector3D addSpeed;
  /// Positional limits?
  stVector3D limit;
  /// Collision translation
  stVector3D collisionTranslation;
  /// Translation separate of inertia
  stVector3D inertiaTranslation;
  /// The normal of the collide ground, if any
  stVector3D groundNormal;
  /// The normal of the collided wall, if any
  stVector3D wallNormal;
  /// Number of calls made to mechanics without colliding with anything
  int8 collideCount;
  /// Padding
  padding(3)
};

/// AI and DNM message-interchange:
/// "Module Allowing the Communication of Datas from the Player or the Intelligence to the Dynamics"
struct DNM::stMACDPID : structure {
  float32 data0;
  stVector3D data1;
  stVector3D data2;
  stVector3D data3;
  float32 data4;
  float32 data5;
  float32 data6;
  stDynamicsRotation data7;
  stDynamicsRotation data8;
  int8 data9;
  uint16 data10;
  stVector3D data11;
  float32 data12;
  stVector3D data13;
  float32 data14;
  uint8 data15;
};

/// Dynamics complex block
struct DNM::stDynamicsComplexBlock : structure {
  float32 tiltStrength;
  float32 tiltInertia;
  float32 tiltOrigin;
  float32 tiltAngle;
  float32 hangingLimit;
  stVector3D contact;
  stVector3D fallTranslation;
  /// Injectable parameters
  stMACDPID macdpid;
  pointer<stSuperObject> platformSuperObject;
  stTransform previousMatrixAbsolute;
  stTransform previousMatrixPrevious;
};

/// Dynamics obstacle reported from mechanics
struct DNM::stDynamicsObstacle : structure {
  /// Collision rate
  float32 rate;
  /// Contact normal
  stVector3D normal;
  /// World contact point
  stVector3D contact;
  /// Material for entity 1 (self)
  pointer<GMT::stGameMaterial> myMaterial;
  /// Material for entity 2 (object collided with)
  pointer<GMT::stGameMaterial> collidedMaterial;
  /// Collided object
  pointer<stSuperObject> superObject;
};

/// A linear and angular movement offset
struct DNM::stDynamicsMovement : structure {
  /// The linear movement
  stVector3D linear;
  /// The angular movement
  stDynamicsRotation angular;
};

/// Dynamics collision report
struct DNM::stDynamicsReport : structure {
  /// The previous surface state
  uint32 previousSurfaceState;
  /// The current surface state
  uint32 currentSurfaceState;
  /// Generic obstacle
  stDynamicsObstacle obstacle;
  /// Ground obstacle
  stDynamicsObstacle ground;
  /// Wall obstacle
  stDynamicsObstacle wall;
  /// Actor obstacle
  stDynamicsObstacle character;
  /// Water obstacle
  stDynamicsObstacle water;
  /// Ceiling obstacle
  stDynamicsObstacle ceiling;
  /// Previous absolute speed
  stDynamicsMovement previousAbsoluteSpeed;
  /// Current absolute speed
  stDynamicsMovement currentAbsoluteSpeed;
  /// Previous absolute position
  stDynamicsMovement previousAbsolutePosition;
  /// Current absolute position
  stDynamicsMovement currentAbsolutePosition;
  /// Extra flags
  char8 bitField;
  /// Padding
  padding(3)
};

/// Parameters for mechanics engine
struct DNM::stDynamics : structure {
  stDynamicsBaseBlock base;
  stDynamicsAdvancedBlock advanced;
  stDynamicsComplexBlock complex;
  
  auto flag(int flag) -> bool {
    return base.flags & flag;
  }
  
  auto endFlag(int flag) -> bool {
    return base.endFlags & flag;
  }
};

struct DNM::stDynamicsParsingData : structure {
  stVector3D position;
  float32 outAlpha;
  stVector3D vector;
};

/// Dynamics reference structure
struct DNM::stDynam : structure {
  pointer<stDynamics> dynamics;
  pointer<stDynamicsParsingData> parsingDatas;
  uint32 usedMechanics;
};


#pragma mark - MEC -

// Dynamics obstacle type
#define dynamicsObstacleTypeNothing     0
#define dynamicsObstacleTypeScenery     1
#define dynamicsObstacleTypeMobile      2
#define dynamicsObstacleTypeDoubleEdge  4
#define dynamicsObstacleTypeMobileWall  9

/// Mechanics engine obstacle (used internally)
/// Cast to stCollisionCase
struct MEC::stMechanicsObstacle : structure {
  /// Collision rate
  float32 rate;
  /// Contact normal
  stVector3D normal;
  /// World contact poimt
  stVector3D contact;
  /// Material for entity 1 (self)
  pointer<GMT::stGameMaterial> myMaterial;
  /// Material for entity 2 (object collided with)
  pointer<GMT::stGameMaterial> collidedMaterial;
  /// Collided Object
  pointer<stSuperObject> superObject;
  /// Type of the obstacle
  uint32 type;
  /// Entity 1 type (self)
  int16 myEntity;
  /// Entity 2 type (object collided with)
  int16 collidedEntity;
  /// Translation to resolve the collision
  stVector3D translation;
  /// Zone movement
  stVector3D zoneMove;
  /// End position of dynamic object
  stVector3D zonePosition;
  /// Zone radius of dynamic object
  float32 zoneRadius;
};

struct MEC::stMechanicsReport : structure {
  /// The current surface state
  uint32_t currentSurfaceState;
  /// Generic obstacle
  stMechanicsObstacle obstacle;
  /// Ground obstacle
  stMechanicsObstacle ground;
  /// Wall obstacle
  stMechanicsObstacle wall;
  /// Actor obstacle
  stMechanicsObstacle character;
  /// Water obstacle
  stMechanicsObstacle water;
  /// Ceiling obstacle
  stMechanicsObstacle ceiling;
};

#pragma mark - Engine object

struct stStandardGameInfo : structure {
  /// Family object type index
  int32 familyType;
  /// Model object type index
  int32 modelType;
  /// Instance object type index
  int32 instanceType;
  /// Superobject containing this actor
  pointer<stSuperObject> superObject;
  uint8 initialFlag;
  uint8 flag1;
  uint8 flag2;
  padding(2)
  uint32 lastFrame;
  uint32 capabilities;
  uint8 tractionFactor;
  uint8 hitPoints;
  uint8 maxHitPoints;
  uint8 maxHitPointsMax;
  uint32 customBits;
  uint32 aiCustomBits;
  uint8 platformType;
  uint8 miscFlags;
  uint8 transparencyZoneMin;
  uint8 transparencyZoneMax;
  uint32 initialCustomBits;
  uint32 aiInitialCustomBits;
  uint8 initialHitPoints;
  uint8 maxInitialHitPoints;
  uint8 initialMiscFlags;
  uint8 tooFarLimit;
  uint8 importance;
  uint8 optional;
  padding(2)
  /* :: custom values :: */
};

/// Engine object - an actor in the dynamic world
struct stEngineObject : structure {
  /// 3D-related parameters
  pointer<st3DData> data3D;
  /// Standard game info
  pointer<stStandardGameInfo> stdGame;
  /// Dynamics
  pointer<DNM::stDynam> dynam;
  /// Brain and AI
  pointer<AI::stBrain> brain;
  /// Cinematic-related info of this actor
  pointer<CINE::stCineInfo> cineInfo;
  /// Collision geometry set
  pointer<COL::stCollideSet> collSet;
  /// Waypoint microstructure
  pointer<MS::stMSWay> msWay;
  /// Light microstructure
  pointer<MS::stMSLight> msLight;
  /// Sector info
  pointer<SECT::stSectorInfo> sectorInfo;
  /// ?
  pointer<MS::stMicro> micro;
  /// Sound microstructure
  pointer<MS::stMSSound> msSound;
  
  /// Get the name of this actor in order of [Instance, Model, Family]
  inline auto name(ObjectType type = Instance) -> std::string;
  /// Get the superobject associated with this actor
  inline auto superobject() -> pointer<stSuperObject>;
  
  /// Get the speed of this actor
  inline auto speed() -> stVector3D;
  /// Get the horizontal speed of this actor
  inline auto horizontalSpeed() -> float;
  /// Get the vertical speed of this actor
  inline auto verticalSpeed() -> float;
  
  
  /// Return the AI model of this actor
  inline auto aiModel() -> pointer<AI::stAIModel>;
  /// Get the dsg variable memory
  inline auto dsgMem() -> pointer<AI::stDsgMem>;
  /// Get the dsg variable memory at specified index
  inline auto dsgVar(int idx, uint32_t* type = nullptr) -> pointer<>;
};


#pragma mark - SECT -

namespace SECT {
using stListOfCharacters = LinkedListElement<stSuperObject>;
using stListOfStaticLights = LinkedListElement<GLI::stLight>;
using stListOfDynamicLights = LinkedListElement<GLI::stLight>;
using stListOfSectorsInGraphicInteraction = LinkedListElement<stSuperObject>;
using stListOfSectorsInCollisionInteraction = LinkedListElement<stSuperObject>;
using stListOfSectorsInActivityInteraction = LinkedListElement<stSuperObject>;
using stListOfSectorsInSoundInteraction = LinkedListElement<stSuperObject>;
}

struct SECT::stSector : structure {
  stDoublyLinkedList<stListOfCharacters> characters;
  stDoublyLinkedList<stListOfStaticLights> staticLights;
  stDoublyLinkedList<stListOfDynamicLights> dynamicLights;
  stDoublyLinkedList<stListOfSectorsInGraphicInteraction> sectorsInGraphInteraction;
  stDoublyLinkedList<stListOfSectorsInCollisionInteraction> sectorsInCollisionInteraction;
  stDoublyLinkedList<stListOfSectorsInActivityInteraction> sectorsInActivityInteraction;
  stDoublyLinkedList<stListOfSectorsInSoundInteraction> sectorsInSoundInteraction;
  stDoublyLinkedList<> soundEventList;
  stVector3D min;
  stVector3D max;
  float32 farPlane;
  uint8 isVirtual;
  int8 cameraType;
  int8 counter;
  int8 priority;
  pointer<> skyMaterial;
  uint8 fog;
#if platform == GCN
  string<0x100> name;
#endif
  
  enum priority { Min = 0, Normal = 64, Max = 127 };
};

#pragma mark - COL -

// Collide element type
enum COL::ElementType : int {
  IndexedTriangles = 1,
  Facemap = 2,
  Sprite = 3,
  TMesh = 4,
  Points = 5,
  Lines = 6,
  IndexedSpheres = 7,
  AABB = 8,
  Cones = 9,
  DeformationSetInfo = 13,
  Invalid = 0xFFFF,
};
  
// Material identifier mask
#define COL_MaterialIdMask_None              (0 << 0)
#define COL_MaterialIdMask_Slide             (1 << 0)
#define COL_MaterialIdMask_Trampoline        (1 << 1)
#define COL_MaterialIdMask_GrabbableLedge    (1 << 2)
#define COL_MaterialIdMask_Wall              (1 << 3)
#define COL_MaterialIdMask_Unknown           (1 << 4)
#define COL_MaterialIdMask_HangableCeiling   (1 << 5)
#define COL_MaterialIdMask_ClimbableWall     (1 << 6)
#define COL_MaterialIdMask_Electric          (1 << 7)
#define COL_MaterialIdMask_LavaDeathWarp     (1 << 8)
#define COL_MaterialIdMask_FallTrigger       (1 << 9)
#define COL_MaterialIdMask_HurtTrigger       (1 << 10)
#define COL_MaterialIdMask_DeathWarp         (1 << 11)
#define COL_MaterialIdMask_Unknown2          (1 << 12)
#define COL_MaterialIdMask_Unknown3          (1 << 13)
#define COL_MaterialIdMask_Water             (1 << 14)
#define COL_MaterialIdMask_NoCollision       (1 << 15)
#define COL_MaterialIdMask_All               (65535)

struct COL::stOctreeNode : structure {
  /// Minimum point
  stVector3D min;
  /// Maximum point
  stVector3D max;
  /// 8 child nodes
  doublepointer<stOctreeNode> children;
  /// Face indices: overlapping indices into element and element data. May be NULL.
  pointer<uint8> faceIndices;
  
  /// The number of elements in the face index list
  inline auto numElements() -> int16 {
    return faceIndices ? *static_cast<int16*>(faceIndices) : int16(0);
  }
};

struct COL::stOctree : structure {
  /// Root node
  pointer<stOctreeNode> rootNode;
  /// Faces which this octree encompasses
  int16 numFaces;
  /// Padding
  padding(2)
  /// Element bases table
  pointer<uint16> elementBases;
  /// Minimum point
  stVector3D min;
  /// Maximum point
  stVector3D max;
};

struct COL::stCollideObject : structure {
  /// Number of vertices
  int16 numVertices;
  /// Number of elements
  int16 numElements;
  /// Number of bounding boxes
  int16 numBoundingBoxes;
  /// Padding
  padding(2)
  /// Vertex data
  pointer<stVector3D> vertices;
  /// Element types
  pointer<int16> elementTypes;
  /// stCollideElement...
  doublepointer<> elements;
  /// Octree partitioning for this collide object
  pointer<stOctree> octree;
  ///
  pointer<> boundingBoxes;
  /// Radius of the bounding sphere which encompasses this object
  float32 boundingSphereRadius;
  /// Position of the bounding sphere which encompasses this object
  stVector4D boundingSpherePosition;
};

struct COL::stPhysicalCollideSet : structure {
  pointer<stCollideObject> zdm;
  pointer<stCollideObject> zdd;
  pointer<stCollideObject> zde;
  pointer<stCollideObject> zdr;
};

struct COL::stColliderInfo : structure {
  pointer<stSuperObject> colliderActors[2];
  stVector3D colliderVectors[2];
  float32 colliderReal[2];
  uint8 colliderType;
  uint8 colliderPriority;
  uint8 unused[2];
};

struct COL::stZdxListEntry : structure {
#if platform == GCN
  pointer<stZdxListEntry> next;
  pointer<stZdxListEntry> prev;
  pointer<> parent;
  pointer<stCollideObject> data;
#else
  pointer<stCollideObject> data;
#endif
};

struct COL::stZdxList : structure {
#if platform == GCN
  stDoublyLinkedList<stZdxListEntry> list;
#else
  stLinkedList<stZdxListEntry> list;
#endif
  uint16 numZdx;
  padding(2)
  
  /// Return a vector of all the collide zdx objects
  inline auto all() -> std::vector<pointer<stCollideObject>>;
};

struct COL::stCsaList : structure {
  stDoublyLinkedList<> list;
};

struct COL::stZoneSetList : structure {
  
};

struct COL::stCollideSet : structure {
  pointer<stZdxList> zddList;
  pointer<stZdxList> zdeList;
  pointer<stZdxList> zdmList;
  pointer<stZdxList> zdrList;
  pointer<stCsaList> zddActivationList;
  pointer<stCsaList> zdeActivationList;
  pointer<stCsaList> zdmActivationList;
  pointer<stCsaList> zdrActivationList;
  pointer<stZoneSetList> zddCurrentActivation;
  pointer<stZoneSetList> zdeCurrentActivation;
  pointer<stZoneSetList> zdrCurrentActivation;
  pointer<stZoneSetList> zdmCurrentActivation;
  uint32 zddPrivilegedZone;
  uint32 zdePrivilegedZone;
  uint32 zdmPrivilegedZone;
  uint32 zdrPrivilegedZone;
  uint8 computeFrequency;
  uint8 characterPriority;
  uint8 collisionFlag;
  padding(1)
  stColliderInfo colliderInfo;
};

struct COL::stCollideElementIndexedTriangles : structure {
  /// Collide material
  pointer<COL::stCollideMaterial> material;
  /// Indices into collide element vertex array
  pointer<uint16> faceIndices;
  /// List of normals
  pointer<stVector3D> normals;
  /// Number of faces
  int16 numFaces;
  /// Index of AABB
  int16 aabbIndex;
  /// Visual set
  pointer<GEO::stVisualElementIndexedTriangles> visual;
  /// Indices of triangle edges
  pointer<uint16> edgeIndices;
  /// Indices of edge normals
  pointer<stVector3D> edgeNormals;
  /// Edge coefficients
  pointer<float32> edgeCoefficients;
  /// Number of edges
  int16 numEdges;
  /// Padding
  padding(2)
};

/// Indexed collide sphere
struct COL::stCollideElementIndexedSphere : structure {
  /// Sphere radius
  float32 radius;
  /// Collide material
  pointer<GMT::stGameMaterial> material;
  /// Index into collide element vertex array
  int16 indexOfCenterPoint;
  /// Padding
  padding(2)
};

/// A collide element of multiple indexed spheres
struct COL::stCollideElementIndexedSpheres : structure {
  /// List of spheres
  pointer<stCollideElementIndexedSphere> spheres;
  /// Number of spheres
  int16 numSpheres;
  /// Collide object AABB index
  int16 aabbIndex;
};

struct COL::stCollideMaterial : structure {
  int16 zoneType;
  /// COL_MaterialIdMask_
  uint16 identifier;
  float32 xDirection;
  float32 yDirection;
  float32 zDirection;
  float32 coefficient;
  uint16 aiType;
  padding(2)
};

/// Collision case, cast internally to MEC::stMechanicsObstacle.
struct COL::stCollisionCase : structure {
  /// Time of collision (-1.0 to 1.0)
  float32 collisionTime;
  /// Normal of the collision
  stVector3D collisionNormal;
  /// World point of the collision
  stVector3D collisionPoint;
  /// Material of the dynamic object
  pointer<GMT::stGameMaterial> dynamicMaterial;
  /// Material of the static object
  pointer<GMT::stGameMaterial> staticMaterial;
  /// Parameter 1 (superobject)
  pointer<> param1;
  /// Parameter 2
  int32 param2;
  int16 dynamicEntity;
  int16 staticEntity;
  stVector3D translation;
  stVector3D movement;
  stVector3D endPosition;
  float32 sphereRadius;
  float32 slide1;
  float32 rebound1;
  float32 slide2;
  float32 rebound2;
};

struct COL::stIndexedAlignedBox : structure {
  int16 min;
  int16 max;
  pointer<GMT::stGameMaterial> material;
};

struct COL::stCollideElementAlignedBoxes : structure {
  pointer<stIndexedAlignedBox> boxes;
  int16 numBoxes;
  int16 parallelBoxIndex;
};

#define COL_MaxSelectedOctreeNodes  100

struct COL::stGVForCollision : structure {
  pointer<stVector3D> vertex1;
  stVector3D edgeVector;
  pointer<stVector3D> vertex2;
  stVector3D dinST0Point;
  float32 dynamicRadius;
  pointer<stTransform> staticGeometricObjMatrix;
  stVector3D dinST1Point;
  stVector3D dinST01Vector;
  pointer<GMT::stGameMaterial> dynamicMaterial;
  pointer<GMT::stGameMaterial> staticMaterial;
  pointer<> vParameter1;
  int16 sParameter2;
  pointer<stCollideObject> staticCollideObject;
  pointer<stCollideElementIndexedTriangles> staticElementIndexedTriangles;
  uint32 selectedCollisionCases;
  int16 staticElementIndex;
  int16 staticIndexedTriangleIndex;
  pointer<stCollideElementIndexedSphere> dynamicIndexedSphere;
  pointer<stOctree> octree;
  uint8 staticGeomObjHasNoTransformationMatrix;
  uint8 dynamicGeomObjHasZoomInsteadOfScale;
  pointer<stCollideObject> dynamicCollideObject;
  pointer<stTransform> dynamicGeometricObjectMatrixT0;
  pointer<stTransform> dynamicGeometricObjectMatrixT1;
  stTransform inverseMatrix;
  stTransform transformMatrixD2ST0;
  stTransform transformMatrixD2ST1;
  stTransform transformMatrixS2DT0;
  stTransform transformMatrixS2DT1;
  float32 staticScale;
  pointer<stCollideElementIndexedSpheres> dynamicElementSpheres;
  pointer<stCollideElementIndexedSpheres> staticElementSpheres;
  int32 bitFieldOfIndexedSpheresInCollision;
  pointer<stCollideElementIndexedSphere> staticIndexedSphere;
  stVector3D swapDinST0Point;
  pointer<stVector3D> dynamicCenter;
  pointer<stVector3D> staticCenter;
  float32 swapRadius;
  uint8 useEnlargedSphere;
  int16 numSelectedNodes;
  pointer<stOctreeNode> selectedOctreeNodes[COL_MaxSelectedOctreeNodes];
  float32 selectedOctreeT[COL_MaxSelectedOctreeNodes];
  pointer<stCollideElementAlignedBoxes> dynamicElementAlignedBoxes;
  int16 dynamicIndexedAlignedBoxIndex;
  pointer<stIndexedAlignedBox> dynamicIndexedAlignedBox;
  pointer<stVector3D> dynamicMinPoint;
  pointer<stVector3D> dynamicMaxPoint;
  stVector3D dinST0MaxPoint;
  stVector3D dinST0MinPoint;
  stVector3D dinST1MaxPoint;
  stVector3D dinST1MinPoint;
  stVector3D dinST08VBox[8];
  stVector3D dinST18VBox[8];
  stVector3D dinST01Vect[8];
  int16 staticIndexedSphereIndex;
  pointer<stCollideElementAlignedBoxes> staticElementAlignedBoxes;
  pointer<stIndexedAlignedBox> staticIndexedBox;
  pointer<stVector3D> pStaticMinPoint;
  pointer<stVector3D> pStaticMaxPoint;
  stVector3D staticMinPoint;
  stVector3D staticMaxPoint;
  stVector3D static8VBox[8];
};

struct COL::stBoundingSphere : structure {
  stVector4D center;
  float32 radius;
#if engine == R3 && platform == PS2
  padding(12)
#endif
};

struct COL::stParallelBox : structure {
  stVector3D min;
  stVector3D max;
};

#pragma mark - GEO

enum GEO::ElementType : int {
  IndexedTriangles = 1,
  Facemap = 2,
  Sprite = 3,
  TMesh = 4,
  Points = 5,
  Lines = 6,
  IndexedSpheres = 7,
  AABB = 8,
  Cones = 9,
  Altimap = 11,
  DeformationSetInfo = 13,
};

union GEO::uVisualObject {
  pointer<GEO::stGeometricObject> geometricObject;
  pointer<MOR::stMorphObject> morphObject;
};

struct GEO::stGeometricObject : structure {
  pointer<stVector3D> vertices;
  pointer<stVector3D> vertexNormals;
#if engine == R3 && platform == GCN
  pointer<> unknown;
#endif
  doublepointer<float32> vertexTransparency;
  pointer<int16> elementTypes;
  pointer<> elements;
  pointer<> edges;
  pointer<> parallelBoxes;
  uint32 type;
  uint16 numVertices;
  uint16 numElements;
  uint16 numEdges;
  uint16 numParallelBoxes;
  float32 boundingSphereRadius;
#if engine == R3 && platform == PS2
  padding(4)
#endif
  stVector4D boundingSphereCenter;
  pointer<> edgesDI;
  int16 numEdgesDI;
  int16 numOctreeEdges;
  int32 usedForDrawingShadow;
  int32 usedForCreatingShadow;
  pointer<> sdcData;
  uint32 isStatic;
  uint32 displayList;
  uint8 vtForSinus;
  padding(3)
};

struct GEO::stVisualSet : structure {
  float32 lastDistance;
  int16 numLodDefinitions;
  int16 type;
  pointer<float32> LODThresholdTable;
  pointer<uVisualObject> LODDefinitions;
  doublepointer<> hRLI;
  int32 numRLI;
};

struct GEO::stVisualElementIndexedTriangles : structure {
  pointer<GLI::stMaterial> visualMaterial;
  int16 numFaces;
  int16 numUVs;
  int16 numUVStages;
  padding(2)
  pointer<uint16> faceIndices;
#if game == R3_GCN
  padding(4)
#endif
  pointer<> faceUVIndices;
  pointer<stVector3D> faceNormals;
  pointer<> UVElements;
  pointer<> edges;
  pointer<> adjacentFaces;
  pointer<> thisIndexList;
  int16 numUsedIndices;
  int16 boundingBoxIndex;
  uint32 displayList;
  pointer<> unknown;
  uint8 portalVisibility;
  padding(3)
  uint32 vao[4];
};

struct GEO::stColor {
  float32 r;
  float32 g;
  float32 b;
  float32 a;
};

struct GEO::stParallelBox : structure {
  stVector3D min;
  stVector3D max;
};

#pragma mark - GMT

struct GMT::stCollideMaterial : structure {
  int16 zoneType;
  uint16 identifier;
  stVector3D direction;
  float32 coefficient;
  uint16 aiType;
  padding(2)
};

struct GMT::stGameMaterial : structure {
  int32 soundMaterial;
  pointer<stCollideMaterial> collideMaterial;
};

#pragma mark - PO

struct PO::stPhysicalObject : structure {
  pointer<GEO::stVisualSet> visualSet;
  pointer<COL::stPhysicalCollideSet> physicalCollideSet;
  pointer<COL::stBoundingSphere> visualBoundingVolume;
  pointer<COL::stBoundingSphere> collideBoundingVolume;
};


#pragma mark - IPO

struct IPO::stInstantiatedPhysicalObject : structure {
  pointer<PO::stPhysicalObject> physicalObject;
  pointer<> currentRadiosity;
  doublepointer<> radiosity;
  pointer<stSuperObject> portalCamera;
  uint32 lastTransitionID;
  float32 lastRatioUsed;
#if platform == GCN
  padding(4)
  string<0x32> name;
#endif
};

#pragma mark - stSuperObject

struct stSuperObject : structure {
  uint32 type;
  
  enum type {
    None                 = (0 << 0),
    World                = (1 << 0),
    Actor                = (1 << 1),
    Sector               = (1 << 2),
    PhysicalObject       = (1 << 3),
    PhysicalObjectMirror = (1 << 4),
    IPO                  = (1 << 5),
    IPOMirror            = (1 << 6),
    SpecialEffect        = (1 << 7),
    NoAction             = (1 << 8),
    Mirror               = (1 << 9),
  };
  
  stSuperObject() {}
  
  union {
    pointer<> data;
    pointer<stEngineObject> actor;
    pointer<SECT::stSector> sector;
    pointer<IPO::stInstantiatedPhysicalObject> ipo;
    pointer<PO::stPhysicalObject> physicalObject;
  };
  
  /// The first child in the below hierarchy
  pointer<stSuperObject> firstChild;
  /// The last child in the below hierarchy
  pointer<stSuperObject> lastChild;
  /// The number of children in the below hierarchy
  int32 numChildren;
  /// The next superobject in this hierarchy
  pointer<stSuperObject> next;
  /// The previous superobject in this hierarchy
  pointer<stSuperObject> prev;
  /// The parent of this superobject
  pointer<stSuperObject> parent;
  /// The transform local to the object
  pointer<stTransform> localTransform;
  /// The transform local to the world
  pointer<stTransform> globalTransform;
  /// The last level frame this object was updated
  int32 prevFrameProcessed;
  /// GLI draw flags
  int32 drawFlags;
  /// General flags
  uint32 flags;
  /// The visual bounding box
  pointer<GEO::stParallelBox> visualBoundingBox;
  /// The bounding box of the collision of this object
  pointer<COL::stParallelBox> collideBoundingBox;
  /// Approximate lookat vector
  stVector3D semiLookAt;
  /// Render transparency
  float32 transparency;
  /// Color of outline (when drawflags are set?)
  uint32 outlineColor;
  ///
  int32 displayPriority;
  /// ?
  int32 ilstatus;
  /// Ambient light default color
  stVector3D ambientColor;
  ///
  stVector3D parallelDirection;
  ///
  stVector3D parallelColor;
  /// Superimpose on the viewport
  uint8 superimpose;
  ///
  uint8 isSuperObject;
  ///
  uint8 transition;
  ///
  padding(1)
  
  /// Return the name of this superobject's type
  inline auto typeName() -> std::string;
  /// Return the name of the superobject
  inline auto name(bool fullname = false) -> std::string;
  /// Get the position of the superobject
  inline auto position() -> stVector3D&;
  
  /// Adds a new child object to this superobject,
  /// detaching it from any previous hierarchy.
  auto addChild(pointer<stSuperObject> obj) -> bool {
    if (!obj) return false;
    obj->detach();
    if (!firstChild && !lastChild && numChildren == 0) {
      firstChild = obj;
      lastChild = obj;
      obj->parent = this;
      obj->prev = nullptr;
      obj->next = nullptr;
    } else {
      lastChild->next = obj;
      obj->prev = lastChild;
      lastChild = obj;
      numChildren++;
    }
    return true;
  }
  
  /// Detach this object from the hierarchy
  auto detach() -> pointer<stSuperObject> {
    if (prev) prev->next = next;
    if (next) next->prev = prev;
    if (parent) parent->numChildren--;
    if (parent && parent->firstChild == this) parent->firstChild = next;
    if (parent && parent->lastChild == this) parent->lastChild = prev;
    return this;
  }
  
  auto isDetached() -> bool {
    return !next && !prev && !parent;
  }
  
  /// Find a superobject in this hierarchy
  auto find(std::string name) -> pointer<stSuperObject> {
    _recurse(this, nullptr, [&name](pointer<stSuperObject> obj, void*) {
      std::string found = obj->name();
      printf("finding: %s\n", found.c_str());
      if (found == name) return obj;
    });
    return nullptr;
  }
  
  /// Recurse the tree below this superobject
  template <typename F, typename UserData>
  auto recurse(const F& f, UserData userdata) {
    _recurse(this, userdata, f);
  }
  
  /// Run a custom function for each child object
  template <typename F>
  void forEachChild(const F& f, void *userdata = nullptr) {
    for (pointer<stSuperObject> ii = firstChild; ii; ii = ii->next) {
      f(ii, userdata);
    }
  }
  
  struct iterator {
    iterator(pointer<stSuperObject> start) : obj(start) { /* ... */ }
    auto operator*() const -> pointer<stSuperObject> { return obj; }
    auto operator!=(iterator& source) -> bool { return obj; }
    auto operator++() -> iterator& { obj = obj->next; return *this; }
    auto operator--() -> iterator& { obj = obj->prev; return *this; }
  private:
    pointer<stSuperObject> obj;
  };
  
  auto begin() const -> iterator { return firstChild; }
  auto end() const -> iterator { return lastChild; }
  
private:
  template <typename F, typename UserData>
  void _recurse(stSuperObject *root, UserData userdata, const F& f) {
    for (stSuperObject *ii = root->firstChild; ii; ii = ii->next) {
      f(ii, userdata);
      _recurse(ii, userdata, f);
    }
  }
};


#pragma mark - AI -

// Designer variable types
#define AI_DsgVarType_Boolean           0
#define AI_DsgVarType_Byte              1
#define AI_DsgVarType_UByte             2
#define AI_DsgVarType_Short             3
#define AI_DsgVarType_UShort            4
#define AI_DsgVarType_Int               5
#define AI_DsgVarType_UInt              6
#define AI_DsgVarType_Float             7
#define AI_DsgVarType_Vector            8
#define AI_DsgVarType_List              9
#define AI_DsgVarType_Comport           10
#define AI_DsgVarType_Action            11
#define AI_DsgVarType_Capabilities      12
#define AI_DsgVarType_Input             13
#define AI_DsgVarType_SoundEvent        14
#define AI_DsgVarType_Light             15
#define AI_DsgVarType_GameMaterial      16
#define AI_DsgVarType_VisualMaterial    17
#define AI_DsgVarType_Actor             18
#define AI_DsgVarType_Waypoint          19
#define AI_DsgVarType_Graph             20
#define AI_DsgVarType_Text              21
#define AI_DsgVarType_SuperObject       22
#define AI_DsgVarType_SOLinks           23
#define AI_DsgVarType_ActorArray        24
#define AI_DsgVarType_VectorArray       25
#define AI_DsgVarType_FloatArray        26
#define AI_DsgVarType_IntArray          27
#define AI_DsgVarType_WaypointArray     28
#define AI_DsgVarType_TextArray         29
#define AI_DsgVarType_TextRefArray      30
#define AI_DsgVarType_GraphArray        31
#define AI_DsgVarType_Array9            32
#define AI_DsgVarType_SNDEventArray     33
#define AI_DsgVarType_Array11           34
#define AI_DsgVarType_Way               35
#define AI_DsgVarType_ActionArray       36
#define AI_DsgVarType_SuperObjectArray  37
#define AI_DsgVarType_ObjectList        38
#define AI_NumDsgVarTypes               39

// Script node types
#define AI_ScriptNodeTypeKeyword            0
#define AI_ScriptNodeTypeCondition          1
#define AI_ScriptNodeTypeOperator           2
#define AI_ScriptNodeTypeFunction           3
#define AI_ScriptNodeTypeProcedure          4
#define AI_ScriptNodeTypeMetaAction         5
#define AI_ScriptNodeTypeBeginMacro         6
#define AI_ScriptNodeTypeBeginMacro2        7
#define AI_ScriptNodeTypeEndMacro           8
#define AI_ScriptNodeTypeField              9
#define AI_ScriptNodeTypeDsgVarRef          10
#define AI_ScriptNodeTypeDsgVarRef2         11
#define AI_ScriptNodeTypeConstant           12
#define AI_ScriptNodeTypeReal               13
#define AI_ScriptNodeTypeButton             14
#define AI_ScriptNodeTypeConstantVector     15
#define AI_ScriptNodeTypeVector             16
#define AI_ScriptNodeTypeMask               17
#define AI_ScriptNodeTypeModuleRef          18
#define AI_ScriptNodeTypeDsgVarID           19
#define AI_ScriptNodeTypeString             20
#define AI_ScriptNodeTypeLipsSynchroRef     21
#define AI_ScriptNodeTypeFamilyRef          22
#define AI_ScriptNodeTypeActorRef           23
#define AI_ScriptNodeTypeActionRef          24
#define AI_ScriptNodeTypeSuperObjectRef     25
#define AI_ScriptNodeTypeSOLinksRef         26 /* ? */
#define AI_ScriptNodeTypeWaypointRef        27
#define AI_ScriptNodeTypeTextRef            28
#define AI_ScriptNodeTypeBehaviorRef        29
#define AI_ScriptNodeTypeModuleRef2         30
#define AI_ScriptNodeTypeSoundEventRef      31
#define AI_ScriptNodeTypeObjectTableRef     32
#define AI_ScriptNodeTypeGameMaterialRef    33
#define AI_ScriptNodeTypeVisualMaterial     34
#define AI_ScriptNodeTypeParticleGenerator  35
#define AI_ScriptNodeTypeModelRef           36
#define AI_ScriptNodeTypeModelRef2          37
#define AI_ScriptNodeTypeCustomBits         38
#define AI_ScriptNodeTypeCaps               39
#define AI_ScriptNodeTypeGraph              40 /* ? */
#define AI_ScriptNodeTypeSubroutine         41
#define AI_ScriptNodeTypeNULL               42
#define AI_ScriptNodeTypeCineRef            43 /* ? */
#define AI_ScriptNodeTypeGraphRef           44

struct AI::stBrain : structure {
  pointer<stMind> mind;
  pointer<GMT::stGameMaterial> lastNoCollideMaterial;
  uint8 warnMechanics;
  uint8 activeDuringTransition;
  padding(2)
};

struct AI::stMind : structure {
  pointer<stAIModel> aiModel;
  pointer<stIntelligence> intelligence;
  pointer<stIntelligence> reflex;
  pointer<stDsgMem> dsgMem;
  pointer<> unknown;
  uint8 runIntelligence;
  padding(3)
};

struct AI::stAIModel : structure {
  pointer<stScriptAI> intelligenceBehaviorList;
  pointer<stScriptAI> reflexBehaviorList;
  pointer<stDsgVar> dsgVar;
  pointer<stMacroList> macroList;
  uint8 secondPassFinished;
  padding(3)
};

struct AI::stNodeInterpret : structure {
#if platform == GCN
  uint32 param;
  padding(3)
  uint8 type;
  padding(2)
  uint8 depth;
  padding(1)
#endif
  using ParamType = decltype(param);
};

struct AI::stTreeInterpret : structure {
  pointer<stNodeInterpret> node;
};

union AI::uGetSetParam {
  int8 s8Value;
  int16 s16Value;
  int32 s32Value;
  float32 floatValue;
  pointer<> pointerValue;
};

struct AI::stActionParam : structure {
  union uGetSetParam param[8];
};

struct AI::stActionTableEntry : structure {
#if platform == GCN
  string<0x50> name;
  uint32 param[8];
  padding(4) /* ? */
  padding(4) /* ? */
  pointer<string<>> namePointer; /* ? */
#elif platform == PS2
  stActionParam actionParam;
#endif
  pointer<stNodeInterpret> node;
  uint8 used;
  uint8 numRules;
  uint8 useDefaultReturn;
  uint8 newReturn;
};

struct AI::stActionTable : structure {
  pointer<stActionTableEntry> entries;
  uint8 numEntries;
  uint8 numEntriesUsed;
  uint8 currentEntry;
  padding(1)
};

struct AI::stBehavior : structure {
  string<0x100> name; /* 256 on GCN, at least */
  pointer<stTreeInterpret> scripts;
  pointer<stTreeInterpret> firstScript;
  uint8 numScripts;
  padding(3)
};

struct AI::stMacro : structure {
  string<0x100> name;
  pointer<stTreeInterpret> initialTree;
  pointer<stTreeInterpret> currentTree;
};

struct AI::stMacroList : structure {
  pointer<stMacro> macros;
  uint8 numMacros;
  padding(3)
};

struct AI::stScriptAI : structure {
  pointer<stBehavior> behavior;
  uint32 numBehaviors;
  uint32 noInitialization;
  uint8 numActionTableEntries;
  padding(3)
};

struct AI::stIntelligence : structure {
  doublepointer<stScriptAI> scriptAI;
  pointer<stNodeInterpret> currentTree;
  pointer<stBehavior> currentBehavior;
  pointer<stBehavior> previousBehavior;
  pointer<> actionTable;
  uint32 initializeBehavior;
};

struct AI::stDsgVarInfo : structure {
  uint32 memoryOffset;
  uint32 type;
  int16 saveType;
  padding(2)
  uint32 objectTreeInitialType;
};

struct AI::stDsgVar : structure {
  pointer<> memory;
  pointer<stDsgVarInfo> info;
  uint32 memorySize;
  uint8 infoLength;
  padding(3)
};

struct AI::stDsgMem : structure {
  doublepointer<stDsgVar> dsgVars;
  pointer<> initialBuffer;
  pointer<> currentBuffer;
  
  inline auto dsgVarInfo(int idx) -> pointer<stDsgVarInfo> { return (*dsgVars)->info + idx; }
};

#pragma mark - GLI

struct GLI::stVertex2D : structure {
  float32 x;
  float32 y;
  float32 dz;
};

struct GLI::stCamera : structure {
  int32 cameraMode;
  stTransform transform;
  /// Field of view
  float32 xAlpha;
  /// Field of view
  float32 yAlpha;
  float32 near;
  float32 far;
  float32 screen;
  stVertex2D scale;
  stVertex2D trans;
  float32 xProjectionR;
  float32 yprojectionR;
  stVector3D left;
  float32 dLeft;
  stVector3D right;
  float32 dRight;
  stVector3D up;
  float32 dUp;
  stVector3D down;
  float32 dDown;
  float32 ratio;
  uint8 transparency;
  float32 transpDist;
  uint8 mirrored;
};

struct GLI::stLight {
  int32 active;
  int32 isZBuffered;
  int32 lightType;
  float32 far;
  float32 near;
  float32 littleAlpha;
  float32 bigAlpha;
  float32 littleTangent;
  float32 bigTangent;
#if engine == R3 && platform == PS2
  padding(12)
#elif engine == R3 && platform == GCN
  float32 attenuation0;
  float32 attenuation1;
  float32 attenuation2;
#endif
  stTransform transform;
  GEO::stColor color;
  float32 sqNear;
  float32 sqFar;
  float32 sqDiv;
  
};

struct GLI::stTexture : structure {
  uint32 format;
  uint8 available;
  uint8 textureQuality;
  uint8 depthQuality;
  padding(1)
  pointer<> bitmapData;
  pointer<> colorTable;
  pointer<> specularParam;
  uint32 caps;
  uint16 height;
  uint16 width;
  uint16 realHeight;
  uint16 realWidth;
  float32 addU;
  float32 addV;
  uint32 incrementEnable;
  uint32 chromaKeyColor;
  uint32 blendColor;
  int32 numLOD;
  uint32 compressionCounter;
  uint32 compressionType;
  uint32 mipMapType;
  pointer<stTexture> substitutionTexture;
  uint8 bilinearMode;
  uint8 cyclingMode;
  string<128> filename;
};

struct GLI::stAnimatedTextureNode : structure {
  
};

struct GLI::stMultiTextureMaterial : structure {
  pointer<stTexture> texture;
  uint8 cOperator;
  uint8 cColorOperator;
  uint8 cUVSource;
  uint8 cFlags;
  uint32 textureProperties;
  /* ... */
  
};

struct GLI::stMaterial : structure {
  uint32 type;
  GEO::stColor ambientColor;
  GEO::stColor diffuseColor;
  GEO::stColor specularColor;
  GEO::stColor baseColor;
#if engine == R3 && platform == PS2
  uint32 additionalType;
  int32 specularExponent;
  pointer<stTexture> texture;
  float32 scrollingOffsetU;
  float32 scrollingOffsetV;
  float32 constantOffsetU;
  float32 constantOffsetV;
  int32 incrementEnabled;
#endif
  uint32 actualRefreshCounter;
  pointer<stAnimatedTextureNode> firstAnimatedTextureNode;
  pointer<stAnimatedTextureNode> actualAnimatedTextureNode;
  int32 numDisplayNodes;
  int32 textureDisplayTime;
  uint8 isLocked;
//#if engine == R3 && platform == PS2
  padding(3)
//#endif
  uint32 flags;
  uint32 multiTextureType;
  uint32 numTextureStages;
  stMultiTextureMaterial multiTextureMaterial[4];
};

#pragma mark - WP

struct WP::stWayPoint : structure {
  stVector3D point;
  float32 radius;
  pointer<stSuperObject> superobject;
};

struct WP::stGraphNode : structure {
  pointer<stGraphNode> next;
  pointer<stGraphNode> prev;
  pointer<stGraph> graph;
  pointer<stWayPoint> waypoint;
  int32 waypointType;
  int32 waypointTypeInitial;
  pointer<> arcList;
};

struct WP::stGraph : structure {
  stDoublyLinkedList<stGraphNode> nodes;
};

struct WP::stGraphChainList : structure {
  pointer<stGraph> graph;
  pointer<stGraphChainList> next;
};


#pragma mark - MS

struct MS::stMSWay : structure {
  pointer<WP::stGraph> graph;
  int32 index;
  uint8 spherical;
  padding(3)
};


#pragma mark - SND

namespace SND {
using CUUID = uint64;
using Ref = uint64;
using Real = float32;

struct stBlockEntry;
struct stLinkTableEntry;
struct stTypeInfo;
}

struct SND::stLinkTableEntry : structure {
  uint32 id;
  uint64 cuuid;
};

/// An element chosen randomly
struct SND::stRandomElement : structure {
  /// Link to the resource
  SND::Ref resourceLink;
  /// The probability of this element being chosen
  float32 probability;
};

struct SND::stSwitchElement : structure {
  /// Link to the resource
  SND::Ref resourceLink;
  /// Index of this element
  uint32 index;
};

//struct SND::stElement {
//  /// Link to the resource
//  SND::CUUID resourceLink;
//  /// The probability of this element being chosen
//  float32 probability;
//};

struct SND::stDeTune : structure {
  float32 panning;
};

struct SND::stEventParametersExtraAll : structure {
  CUUID link;
  float32 pitch;
  float32 volume;
  float32 panning;
};

union SND::stEventParameters {
  stEventParametersExtraAll extraAll;
};

//struct SND::stBlockEvent {
//  /// Event identifier
//  uint32 id;
//#if engine == R3
//  /// Name of the event.
//  /// Usually starts with the event type (Play_, Stop_, etc).
//  string<> name;
//#endif
//  /// Event identifier
//  uint32 type;
//  /// Parameters
//  stEventParameters param;
//};

struct SND::stBlockEvent : structure {
  /// Event identifier
  pointer<> unknown;
  ///
  uint64 cuuid;
  /// Event type
  uint32 type;
  /// Unknown
  uint32 unknown2;
  /// Parameters
  stEventParameters param;
  /// Event resource
  //pointer<>
};

struct SND::stTypeInfo : structure {
  /// Name of this type
  pointer<string<>> name;
};

struct SND::stBlockEntry : structure {
  /// Pointer to entry type information
  doublepointer<stTypeInfo> typeInfo;
  /// The unique identifier of this entry
  CUUID cuuid;
  /// Event type
  uint32 type;
  /// Unknown
  uint32 unknown1;
  /// Unknown
  pointer<> unknown2;
  /// Parameters
  stEventParameters param;
  ///
  pointer<> linkData;
};

/***********************/
/** ``STRUCTURE END`` **/
/***********************/

#pragma pack(pop)

#undef padding
#undef concat_inner
#undef unique_name
#undef concat


#pragma mark - Function -

/************************/
/** ``FUNCTION BEGIN`` **/
/************************/

namespace global {
auto objectTypeNameLookup(int type, int idx) -> std::string;
};

#pragma mark EngineStructure

auto stEngineStructure::loadLevel(std::string levelName) -> void {
  nextLevelName = levelName;
  mode = EngineMode::ChangeLevel;
}

#pragma mark EngineObject

auto stEngineObject::name(ObjectType type) -> std::string {
  std::string name;
  for (int i : {stdGame->instanceType, stdGame->modelType, stdGame->familyType})
    if ((name = global::objectTypeNameLookup(type, i)) != "Invalid name") break;
  return name;
}
/// Get the superobject associated with this actor
auto stEngineObject::superobject() -> pointer<stSuperObject> {
  return stdGame->superObject;
}

/// Return the AI model of this actor
auto stEngineObject::aiModel() -> pointer<AI::stAIModel> {
  return brain->mind->aiModel;
}

/// Get the dsg variable memory
auto stEngineObject::dsgMem() -> pointer<AI::stDsgMem> {
  return brain->mind->dsgMem;
}

auto stEngineObject::dsgVar(int idx, uint32_t* type) -> pointer<> {
  try {
    pointer<AI::stDsgMem> mem = brain->mind->dsgMem;
    if (idx > (*mem->dsgVars)->infoLength) return nullptr;
    pointer<AI::stDsgVarInfo> info = mem->dsgVarInfo(idx);
    if (type) *type = info->type;
    return (uint8_t*)mem->currentBuffer + info->memoryOffset;
  } catch (bad_pointer& e) {
    return nullptr;
  }
}

auto stEngineObject::speed() -> stVector3D {
  try {
    return dynam->dynamics->base.previousSpeed;
  } catch (bad_pointer& e) {
    return stVector3D(0.0f, 0.0f, 0.0f);
  }
}

auto stEngineObject::horizontalSpeed() -> float {
  auto s = speed();
  return sqrt(s.x() * s.x() + s.y() * s.y());
}

auto stEngineObject::verticalSpeed() -> float {
  return speed().z();
}

#pragma mark SuperObject

auto stSuperObject::typeName() -> std::string {
  switch (type) {
    case None:
      return "Dummy SuperObject";
    case World:
      return "World";
    case Actor:
      return "Actor";
    case Sector:
      return "Sector";
    case PhysicalObject:
      return "PhysicalObject";
    case PhysicalObjectMirror:
      return "PhysicalObject.Mirror";
    case IPO:
      return "IPO";
    case IPOMirror:
      return "IPO.Mirror";
    case SpecialEffect:
      return "SpecialEffect";
    case NoAction:
      return "NoAction";
    case Mirror:
      return "Mirror";
    default:
      return "Invalid";
  }
}

auto stSuperObject::name(bool fullname) -> std::string {
  try {
    switch (type) {
      case Actor:
        return actor->name();
      case IPO:
        return fullname ? ipo->name : ipo->name.lastPathComponent();
      case Sector:
        return fullname ? sector->name : sector->name.lastPathComponent();
      default:
        return typeName();
    }
  } catch (bad_pointer& e) {
    return "";
  }
}

auto stSuperObject::position() -> stVector3D& {
  try {
    return globalTransform->translation();
  } catch (bad_pointer& e) {
    return globalTransform->translation();
  }
}

#pragma mark - ZdxList

auto COL::stZdxList::all() -> std::vector<pointer<stCollideObject>> {
  assert(list.numEntries == numZdx); //should never happen
  std::vector<pointer<stCollideObject>> objects;
  list.forEach([&](pointer<stZdxListEntry> entry, void*) { objects.emplace_back(entry->data); });
  return objects;
}

#pragma mark - Static functions

/// Determine the sector of a world-space point
static inline auto sectorSearch(pointer<stSuperObject> fatherSector, stVector3D point) -> pointer<stSuperObject> {
  try {
    float dNear = INFINITY;
    float dCurrent = INFINITY;
    float dVirtual = INFINITY;
    int8 p = SECT::stSector::priority::Min;
    int8 v = SECT::stSector::priority::Max;
    
    pointer<stSuperObject> targetSector = nullptr;
    pointer<stSuperObject> targetSectorVirtual = nullptr;
    
    fatherSector->forEachChild([&](pointer<stSuperObject> object, void*) {
      pointer<SECT::stSector> sector = object->sector;
      stVector3D min = sector->min;
      stVector3D max = sector->max;
      
      if (point >= min && point <= max) {
        stVector3D distance = (min + max) / 2.0f - point;
        dNear = distance.length();
        
        if (!sector->isVirtual) {
          if (sector->priority > p) {
            targetSector = object;
            dCurrent = dNear;
            p = sector->priority;
          } else if (sector->priority == p && dNear < dCurrent) {
            targetSector = object;
            dCurrent = dNear;
          }
        } else {
          if (sector->priority > v) {
            targetSectorVirtual = object;
            dVirtual = dNear;
            v = sector->priority;
          } else if (sector->priority == v && dNear < dVirtual) {
            targetSectorVirtual = object;
            dVirtual = dNear;
          }
        }
      }
    });
    
    if (!targetSector) targetSector = targetSectorVirtual;
    if (!targetSector) targetSector = fatherSector->lastChild; // UNIVERS
    return targetSector;
  } catch (...) {
    return nullptr;
  }
}

/**********************/
/** ``FUNCTION END`` **/
/**********************/

#undef s


#pragma mark - Script -

namespace script {

using node = AI::stNodeInterpret;

enum nodetype : uint8_t {
  Keyword           = 0,
  Condition         = 1,
  Operator          = 2,
  Function          = 3,
  Procedure         = 4,
  MetaAction        = 5,
  BeginMacro        = 6,
  BeginMacro2       = 7,
  EndMacro          = 8,
  Field             = 9,
  DsgVarRef         = 10,
  DsgVarRef2        = 11,
  Constant          = 12,
  Real              = 13,
  Button            = 14,
  ConstantVector    = 15,
  Vector            = 16,
  Mask              = 17,
  ModuleRef         = 18,
  DsgVarID          = 19,
  String            = 20,
  LipsSynchroRef    = 21,
  FamilyRef         = 22,
  ActorRef          = 23,
  ActionRef         = 24,
  SuperObjectRef    = 25,
  SOLinksRef        = 26, // ?
  WaypointRef       = 27,
  TextRef           = 28,
  BehaviorRef       = 29,
  ModuleRef2        = 30,
  SoundEventRef     = 31,
  ObjectTableRef    = 32,
  GameMaterialRef   = 33,
  VisualMaterial    = 34,
  ParticleGenerator = 35,
  ModelRef          = 36,
  ModelRef2         = 37,
  CustomBits        = 38,
  Caps              = 39,
  Graph             = 40, // ?
  Subroutine        = 41,
  Null              = 42,
  CineRef           = 43, // ?
  GraphRef          = 44,
};

struct TranslationToken {
  // TranslationToken(const char* text, pointer<Node> originalNode = nullptr);
  TranslationToken(std::string text, node *originalNode = nullptr) {
    this->text = text;
    this->originalNode = originalNode;
  }
  /// Mode: Tree -> Source
  ///   Text string of the generated token
  std::string text;
  /// Mode: Tree -> Source
  ///   A description of the token symbol
  std::string description;
  /// Mode: Tree -> Source
  ///   The original script node from which this token was generated
  node *originalNode = nullptr;
  /// Mode: Source -> Tree
  ///   Output node
  node node;
  /// equals
  bool operator ==(TranslationToken other) { return text == other.text; }
};

enum TranslationMode {
  TreeToSource,
  SourceToTree,
};

struct TranslationOptions {
  /// Mode: Tree -> Source
  ///   Remove parentheses which do not affect the control flow of the program.
  bool removeUnnecessaryParentheses;
  /// Mode: Tree -> Source
  ///   Replace macrorefs with their respective source trees
  bool expandMacroReferences;
  
  std::vector<std::string> conditionTable;
  std::vector<std::string> functionTable;
  std::vector<std::string> procedureTable;
  std::vector<std::string> metaActionTable;
  std::vector<std::string> fieldTable;
};

enum TranslationTokenType {
  Default,
  Space,
};

struct TranslationResult {
  TranslationResult(TranslationMode mode) : mode(mode) {};
  TranslationMode mode;
  TranslationOptions options;
  std::vector<TranslationToken> tokens;
};

struct TranslationEngine;

struct TranslationContext {
  TranslationContext(pointer<node> tree, std::function<void(TranslationContext&)> fn, TranslationEngine *e) : currentNode(tree), function(fn), _engine(e) {
    /* ... */
  }
  
  TranslationEngine *_engine;
  
  template <typename ... Args>
  void emit(bool condition, Args ...args) {
    if (condition) {
      for (const auto v : { args... }) {
        std::string str = v;
        pointer<node> *targetNode = nullptr;
        if (str.starts_with(":")) {
          // *targetNode = currentNode;
          str = str.substr(1, std::string::npos);
        }
        
        
        TranslationToken tok(str, std::string(v).starts_with(":") ? (node*)currentNode : nullptr);
        tokens.push_back(tok);
        
        if (str == "\n") {
          for (int i = 0; i < indentationLevel; i++) {
            TranslationToken tok("  ", nullptr);
            tokens.push_back(tok);
          }
        }
      }
    }
  }
  
  void indent(bool condition, int off) {
    if (condition) {
      indentationLevel += off;
    }
  }
  
  uint32_t param() { return currentNode->param; }
  operator uint32_t() { return currentNode->param; }
  bool done() { return currentNode->type == nodetype::EndMacro || currentNode->depth == 0; }
  
  pointer<node> firstNode;
  pointer<node> currentNode;
  int indentationLevel = 0;
  
  void child(bool condition, int child = 0) {
    if (condition) {
      pointer<node> orig = currentNode;
      pointer<node> node = currentNode + 1;
      uint8_t min = currentNode->depth, occ = 0;
      while (node->type != nodetype::EndMacro && node->depth > min) {
        if (node->depth == min + 1 && occ++ == child) {
          currentNode = node;
          function(*this);
          break;
        }
        node++;
      }
      currentNode = orig;
    }
  }
  
  void branch(bool args = false) {
    pointer<node> orig = currentNode;
    pointer<node> node = currentNode + 1;
    uint8_t depth = currentNode->depth + 1, numArgs = 0;
    while (node->type != nodetype::EndMacro && node->depth >= depth) {
      if (node->depth == depth) {
        currentNode = node;
        function(*this);
        
        // Argument separator
        if (args) {
          numArgs++;
          emit(true, ",", " ");
        }
      }
      node++;
    }
    
    if (numArgs >= 1) {
      tokens.pop_back();
      tokens.pop_back();
    }
    
    currentNode = orig;
  }
  
  bool isEnd(pointer<node> node) {
    return node->type == EndMacro || node->depth == 0;
  }
  
  void seekNextDepth() {
    uint8_t depth = currentNode->depth;
    if (isEnd(currentNode++))
      return;
    while (!isEnd(currentNode) && currentNode->depth != depth)
      currentNode++;
  }
  
  void translate() {
    function(*this);
  }
  
  std::vector<TranslationToken> tokens;
  std::function<void(TranslationContext&)> function;
  
  std::vector<std::string> conditionTable;
  std::vector<std::string> functionTable;
  std::vector<std::string> procedureTable;
  std::vector<std::string> fieldTable;
  std::vector<std::string> metaActionTable;
};

static void keyword(TranslationContext& s) {
  s.emit(s >= 0 && s <= 15, ":if", " ");
  s.emit(s == 1 || s == 15, "!", "(");
  s.emit(s == 14, "#debug");
  s.emit(s == 15, "defined", "(", "U64", ")");
  s.emit(s >= 2 && s <= 13, "framerule", " ", "%", " ", std::to_string(1 << (s.param() - 1)).c_str(), " ", s <= 7 ? "==" : "!=", " ", "0");
  s.emit(s >= 2 && s <= 15, " ", "&&", " ");
  s.child(s >= 0 && s <= 15);
  //s.emit(s >= 0 && s <= 15, " ");
  s.emit(s == 1 || s == 15, ")");
  
  s.emit(s == 17, ":else", " ");
  s.emit(s == 16 || s == 17, "\n", "{");
  s.indent(s == 16 || s == 17, +1);
  s.emit(s == 16 || s == 17, "\n");
  if (s == 16 || s == 17) s.branch();
  if ((s == 16 || s == 17)) {
    s.indent(s == 16 || s == 17, -1);
    s.tokens.pop_back();
  }
  s.emit(s == 16 || s == 17, "}", "\n");
  
  s.emit(s == 19, "self");
  s.emit(s == 20, "MainActor");
}

static void condition(TranslationContext& s) {
  //printf("condition: %d %X\n", s.currentNode->type, (uint32_t)s.currentNode->param);
  s.emit(s == 2, "!", "(");
  s.child(s <= 9, 0);
  s.emit(s != 2 && s <= 9, " ");
  s.emit(s == 0, ":&&");
  s.emit(s == 1, ":||");
  s.emit(s == 3, ":^");
  s.emit(s == 4, ":==");
  s.emit(s == 5, ":!=");
  s.emit(s == 6, ":<");
  s.emit(s == 7, ":>");
  s.emit(s == 8, ":<=");
  s.emit(s == 9, ":>=");
  s.emit(s != 2 && s <= 9, " ");
  s.emit(s >= 10, std::string(":" + s.conditionTable[s.param()]).c_str(), "(");
  s <= 9 ? s.child(true, 1) : s.branch(true);
  s.emit(s == 2 || s >= 10, ")");
  //s.emit(true, " ");
}

static void _operator(TranslationContext& s) {
  s.emit(s == 4 || s == 19, "-");
  s.emit(s <= 4 || (s >= 17 && s <= 21 && s != 19), "(");
  s.emit(s == 26, "(");
  s.child(s <= 27, 0);
  s.emit(s == 0, " ", ":+", " ");
  s.emit(s == 1, " ", ":-", " ");
  s.emit(s == 2, " ", ":*", " ");
  s.emit(s == 3, " ", ":/", " ");
  s.emit(s == 5, " ", ":%%", " ");
  s.emit(s == 6, " ", ":+=", " ");
  s.emit(s == 7, " ", ":-=", " ");
  s.emit(s == 8, " ", ":*=", " ");
  s.emit(s == 9, " ", ":/=", " ");
  s.emit(s == 10, ":++", ";", "\n");
  s.emit(s == 11, ":--", ";", "\n");
  s.emit(s == 12, " ", ":=", " ");
  s.emit(s == 13, ":.");
  s.emit(s == 14, ".", ":X"); // vector x
  s.emit(s == 15, ".", ":Y"); // vector y
  s.emit(s == 16, ".", ":Z"); // vector z
  s.emit(s == 17, " ", ":+", " "); // vector + vector
  s.emit(s == 18, " ", ":-", " "); // vector - vector
  s.emit(s == 20, " ", ":*", " "); // vector * scalar
  s.emit(s == 21, " ", ":/", " "); // vector / scalar
  s.emit(s == 22, ".", ":X", " ", "="); // vector.x = s
  s.emit(s == 23, ".", ":Y", " ", "="); // vector.x = s
  s.emit(s == 24, ".", ":Z", " ", "="); // vector.x = s
  s.emit(s == 25, "."); // 'ultra'
  s.emit(s == 26, ")", "("); // modelcast
  s.emit(s == 27, "["); // array access
  
  s.child(s <= 27 && s != 4 && s != 10 && s != 11 && !(s >= 14 && s <= 16) && s != 19, 1);
  s.emit(s == 26, ")", ")"); // modelcast
  s.emit(s == 27, "]"); // array access
  s.emit(s <= 4 || (s >= 17 && s <= 21 && s != 19), ")");
  s.emit(s == 12 || (s >= 6 && s <= 9) || (s >= 22 && s <= 23), ";", "\n");
}

static void function(TranslationContext& s) {
  s.emit(true, std::string(":" + s.functionTable[s.param()]).c_str(), "(");
  s.branch(true);
  s.emit(true, ")");
}

static void procedure(TranslationContext& s) {
  s.emit(true, std::string(":" + s.procedureTable[s.param()]).c_str(), "(");
  s.branch(true);
  s.emit(true, ")", ";", "\n");
}

static void metaAction(TranslationContext& s) {
  s.emit(true, std::string(":" + s.metaActionTable[s.param()]).c_str(), "(");
  s.branch(true);
  s.emit(true, ")", ";", "\n");
}

static void field(TranslationContext& s) {
  s.emit(true, (":" + s.fieldTable[s.param()]).c_str());
}

static void dsgvar(TranslationContext& s) {
  s.emit(true, (std::string(":DsgVar_") + std::to_string(s.param())).c_str());
}

static void constant(TranslationContext& s) {
  s.emit(true, (":" + std::to_string((int32_t)s.param())).c_str());
}

static void real(TranslationContext& s) {
  uint32_t p = s.param();
  char buf[64];
  std::sprintf(buf, ":%.5gf", *(float*)&p);
  
  s.emit(true, std::string(buf).c_str());
}

static void vector(TranslationContext& s) {
  s.emit(true, ":Vector", "(");
  s.branch(true);
  s.emit(true, ")");
}

static void button(TranslationContext& s) {
  // s.emit(true, "", )
}

static void _string(TranslationContext& s) {
  const char *str = pointer<string<>>(s.param());
  s.emit(true, "\"", (":" + std::string(str)).c_str(), "\"");
}

static void reference(TranslationContext& s) {
  s.emit(true, ":" + std::to_string(s.param()));
}

static void subroutine(TranslationContext& s) {
  //  if (s.engine->options.expandMacroReferences) {
  //    pointer<structure::stMacro> macro = pointer<structure::stMacro>(s.param());
  //    TranslationEngine t(s.engine->options);
  //    TranslationResult *result = t.translate(nullptr, macro->currentTree->node);
  //
  //    for (TranslationToken& tok : result->tokens) {
  //      s.tokens.push_back(tok);
  //    }
  //  } else {
  s.emit(true, std::string(":" + std::to_string(s.param())).c_str(), "(", ")", ";", "\n");
  // }
}

static void null(TranslationContext& s) {
  s.emit(true, ":NULL");
}

static std::map<int, std::function<void(TranslationContext& s)>> TranslationTable {
  { nodetype::Keyword, &keyword },
  { nodetype::Condition, &condition },
  { nodetype::Operator, &_operator },
  { nodetype::Function, &function },
  { nodetype::Procedure, &procedure },
  { nodetype::MetaAction, &metaAction },
  { nodetype::BeginMacro, nullptr },
  { nodetype::BeginMacro2, nullptr },
  { nodetype::EndMacro, nullptr },
  { nodetype::Field, &field },
  { nodetype::DsgVarRef, &dsgvar },
  { nodetype::DsgVarRef2, &dsgvar },
  { nodetype::Constant, &constant },
  { nodetype::Real, &real },
  { nodetype::Button, nullptr },
  { nodetype::ConstantVector, &vector },
  { nodetype::Vector, &vector },
  { nodetype::Mask, nullptr },
  { nodetype::ModuleRef, nullptr },
  { nodetype::DsgVarID, nullptr },
  { nodetype::String, &_string },
  { nodetype::LipsSynchroRef, &reference },
  { nodetype::FamilyRef, &reference },
  { nodetype::ActorRef, &reference},
  { nodetype::ActionRef, &reference },
  { nodetype::SuperObjectRef, &reference },
  { nodetype::SOLinksRef, &reference },
  { nodetype::WaypointRef, &reference },
  { nodetype::TextRef, &reference },
  { nodetype::BehaviorRef, &reference },
  { nodetype::ModuleRef2, &reference },
  { nodetype::SoundEventRef, &reference },
  { nodetype::ObjectTableRef, &reference },
  { nodetype::GameMaterialRef, &reference },
  { nodetype::VisualMaterial, nullptr },
  { nodetype::ParticleGenerator, nullptr },
  { nodetype::ModelRef, nullptr },
  { nodetype::ModelRef2, nullptr },
  { nodetype::CustomBits, nullptr },
  { nodetype::Caps, nullptr },
  { nodetype::Graph, nullptr },
  { nodetype::Subroutine, &subroutine },
  { nodetype::Null, &null },
  { nodetype::CineRef, nullptr },
  { nodetype::GraphRef, nullptr },
};

static void NodeTranslate(TranslationContext& s) {
  //printf("node: %d %d %d\n", s.currentNode->type, (uint32_t)s.currentNode->param, s.currentNode->depth);
  pointer<node> node = s.currentNode;
  if (TranslationTable.find(node->type) != TranslationTable.end()) {
    if (TranslationTable[node->type] != nullptr)
      TranslationTable[node->type](s);
  }
  s.seekNextDepth();
}

/// Context for script translation
struct TranslationEngine {
  
  TranslationEngine(TranslationOptions opt) : options(opt) {
    /* ... */
  }
  
  TranslationResult *translate(pointer<stSuperObject> actor, pointer<node> tree) {
    initialNode = tree;
    TranslationContext s(tree, NodeTranslate, this);
    s.conditionTable = options.conditionTable;
    s.functionTable = options.functionTable;
    s.procedureTable = options.procedureTable;
    s.metaActionTable = options.metaActionTable;
    s.fieldTable = options.fieldTable;
    
    while (!s.done())
      s.translate();
    
    TranslationResult *result = new TranslationResult(TranslationMode::TreeToSource);
    result->tokens = s.tokens;
    return result;
  }
  
  TranslationResult *translate(pointer<stSuperObject> actor, std::string source) {
    TranslationResult *result = new TranslationResult(TranslationMode::SourceToTree);
    return result;
  }
  
  
  TranslationOptions options;
  
  TranslationMode mode;
  pointer<node> initialNode;
  pointer<node> currentNode;
};

}; /* script */


#pragma mark - Memory stream

// TODO: Move this to top

namespace memory {
struct stream {
  stream() = default;
  enum mode { read, write };
  
  using position_t = size_t;
  
  template<typename T> stream(mode m, T* data, size_t sz) : mode(m) {
    buf = static_cast<uint8_t*>(data); size = sz;
  }
  
  stream(const std::filesystem::path path, mode m) : mode(m) {
    name = path.filename();
    std::fstream fs(path, m == read ? std::ios::in : std::ios::out);
    if (!fs.is_open()) throw "failed to open stream @ " + path.string();
    fs.seekg(0, std::ios::end);
    size = fs.tellg();
    fs.seekg(0);
    buf = new uint8_t[size];
    fs.read((char*)buf, size);
    fs.close();
  }
  
  template<typename T> auto rw(T&v) {
    if (mode == read) v = *(T*)(buf + pos);
    if (mode == write) *(T*)(buf + pos) = v;
    advance(sizeof(T));
  }
  
  template<typename T> auto string(T *p, size_t length) {
    if (mode == read) memcpy(p, buf + pos, length);
    if (mode == write) memcpy(buf + pos, p, length);
    advance(length);
  }
  
  inline void advance(auto offset) { pos +=offset; }
  inline void seek(auto offset) { pos = offset; }
  
  std::string name;
  mode mode = read;
  position_t pos = 0;
  size_t size = 0;
  uint8_t* buf = nullptr;
};

}

#pragma mark - Format -

namespace format {

struct ptr {
  using FileID = int;
  static constexpr FileID FileID_FIX = 0;
  static constexpr FileID FileID_LVL = 1;
  static constexpr FileID FileID_KF  = 2;
  static constexpr FileID FileID_VB  = 3;
  
  const std::map<FileID, std::string> fileIDName {
    {FileID_FIX, "fix"},
    {FileID_LVL, "lvl"},
    {FileID_KF, "kf"},
    {FileID_VB, "vb"}
  };
  
  struct FileIDPointerPair {
    FileID fileID;
    uint32 pointer;
  };
  
  ptr(memory::stream& s) {
    assert(s.mode == memory::stream::mode::read);
    
    uint32 numPointers;
    s.rw(numPointers);
    for (auto i : range(numPointers)) {
      uint32_t fileID, pointer;
      s.rw(fileID);
      s.rw(pointer);
      pointer += 4;
      pointers.emplace_back(FileIDPointerPair {static_cast<FileID>(fileID), pointer});
    }
    
    size_t numFillInPointers = (s.size - s.pos) / 16;
    for (auto i : range(numFillInPointers)) {
      uint32_t doublePointer;
      uint32_t sourceFileID;
      uint32_t fillInPointer;
      uint32_t targetFileID;
      
      s.rw(doublePointer);
      s.rw(sourceFileID);
      s.rw(fillInPointer);
      s.rw(targetFileID);
      
      FileIDPointerPair src { static_cast<FileID>(sourceFileID), doublePointer };
      FileIDPointerPair dst { static_cast<FileID>(sourceFileID), doublePointer };
      fillInpointers.emplace_back(std::make_pair(src, dst));
    }
  }
  
  std::string name;
  std::vector<FileIDPointerPair> pointers;
  std::vector<std::pair<FileIDPointerPair, FileIDPointerPair>> fillInpointers;
}; /* ptr */

struct lvl {
  
  lvl(const std::filesystem::path path) {
    
  }
  
  
}; /* lvl */

// R3 engine version sound description file
// Formats: .HXC, .HX2 .HXG, .HXX, .HX3, .HXP, .HXB
namespace hx {
#define HX_STRING_MAX_LENGTH  0x100
/// It's a string.
using string = char[HX_STRING_MAX_LENGTH];
/// Unique 64-bit entry identifier
using cuuid = uint64;
/// Language code
using language_code = uint32;

static constexpr auto LanguageDE = 0x64652020;
static constexpr auto LanguageEN = 0x656E2020;
static constexpr auto LanguageES = 0x65732020;
static constexpr auto LanguageFR = 0x66722020;
static constexpr auto LanguageIT = 0x69742020;

enum version {
  hxd, ///< M/Arena
  hxc, ///< R3 PC
  hx2, ///< R3 PS2
  hxg, ///< R3 GCN
  hxx, ///< R3 XBOX (+HD)
  hx3, ///< R3 PS3 HD
  hxp, ///< Sound program driver
  hxb, ///< Sound program driver
};

enum audio_format {
  pcm = 0x01, ///< PCM s16
  ubi = 0x02, ///< UBI ADPCM
  psx = 0x03, ///< PS ADPCM
  dsp = 0x04, ///< GC 4-bit ADPCM
  ima = 0x05, ///< MS IMA ADPCM
  mp3 = 0x55, ///< MPEG.3
};

struct entry {
  hx::cuuid cuuid;
  std::string class_;
  void *data;

  struct language_link {
    hx::cuuid cuuid;
    float32 unknown;
    language_code language;
  };

  std::vector<hx::cuuid> links;
  std::vector<language_link> languageLinks;

//private:
  uint32 _file_offset;
  uint32 _file_size;
  uint32 _tmp_file_size;
};

struct context {
  static constexpr auto IndexCode = 0x58444E49;
  static constexpr auto ExternalFile = 0;
  static constexpr auto MegaFile = 1;
  static constexpr auto BigFile = 2;
  
  context(std::filesystem::path path) {
    workDirectory = path;
    workDirectory.remove_filename();
    
    size_t sz = SIZE_MAX;
    void* data = fileRead(path, 0, sz);
    
    stream = memory::stream(memory::stream::mode::read, data, sz);
    
    uint32 indexCode = IndexCode;
    uint32 indexOffset = 0;
    uint32 indexType = 2;
    uint32 numEntries = entries.size();
    
    if (stream.mode == memory::stream::read) {
      stream.rw(indexOffset);
      stream.seek(indexOffset);
    }
    
    stream.rw(indexCode);
    stream.rw(indexType);
    stream.rw(numEntries);
    
    if (indexCode != IndexCode) throw "invalid index header";
    if (indexType != 1 && indexType != 2) throw "invalid index type";
    if (numEntries == 0) throw "file contains no entries!";
    
    for (auto i : range(numEntries)) {
      uint32 classnameLength;
      hx::entry& e = entries[i];
      
      char classname[HX_STRING_MAX_LENGTH];
      if (stream.mode == memory::stream::write) {
        classnameLength = 0;//hx_class_name(entry->i_class, hx->version, classname, HX_STRING_MAX_LENGTH);
      }
      
      stream.rw(classnameLength);
      stream.string(classname, classnameLength);
      
      if (stream.mode == memory::stream::read) {
        e.class_ = std::string(classname, classnameLength);
        //e.i_class = classFromString(classname);
        //entry->i_class = hx_class_from_string(classname);
      }
      
      uint32 zero = 0;
      stream.rw(e.cuuid);
      stream.rw(e._file_offset);
      stream.rw(e._file_size);
      stream.rw(zero);
      
      uint32 numLinks = e.links.size();
      uint32 numLanguageLinks = e.languageLinks.size();
      stream.rw(numLinks);
      
      printf("%016llX\n", (uint64_t)e.cuuid);
      
      assert(zero == 0);
      
      if (indexType == 2) {
        if (stream.mode == memory::stream::read) {
          e.links.resize(numLinks);
        }
        
        for (auto l : range(numLinks)) {
          stream.rw(e.links[l]);
        }
        
        stream.rw(numLanguageLinks);
        if (stream.mode == memory::stream::read) {
          e.languageLinks.resize(numLanguageLinks);
        }
        
        for (auto l : range(numLanguageLinks)) {
          stream.rw(e.languageLinks[l].language);
          stream.rw(e.languageLinks[l].unknown);
          stream.rw(e.languageLinks[l].cuuid);
        }
      }
      
      memory::stream::position_t p = stream.pos;
      if (stream.mode == memory::stream::read) {
        stream.seek(e._file_offset);
      }
      
      if (!entryRW(e)) {
        printf("failed to %s entry %016llX", stream.mode == memory::stream::read ? "read" : "write", uint64_t(e.cuuid));
      }
      
      if (stream.mode == memory::stream::read) {
        stream.seek(p);
      }
      
    }
    
    
    
    
  }
  
  ~context() {
    for (auto& p : fileMap) delete p.second;
  }
  
  #define EntryData(T) \
    (T*)(entry.data = (hx.stream.mode == memory::stream::mode::write ? entry.data : malloc(sizeof(*data))))

  static void EntryName(context& hx) {
    if (hx.stream.mode == memory::stream::read) {
       
    } else if (hx.stream.mode == memory::stream::write) {
      
    }
  }
  
  static int EventResData(context& hx, entry& entry) {
    SND::stBlockEvent *data = EntryData(SND::stBlockEvent);
    EntryName(hx);
    //unsigned int name_length = strlen(data->name);
  //  stream_rw32(&hx->stream, &data->type);
  //  stream_rw32(&hx->stream, &name_length);
  //  stream_rw(&hx->stream, &data->name, name_length);
  //  stream_rw32(&hx->stream, &data->flags);
  //  stream_rwcuuid(&hx->stream, &data->link);
  //  stream_rwfloat(&hx->stream, data->c + 0);
  //  stream_rwfloat(&hx->stream, data->c + 1);
  //  stream_rwfloat(&hx->stream, data->c + 2);
  //  stream_rwfloat(&hx->stream, data->c + 3);
    return 0;
  }

  struct classinfo { std::function<void(context&, entry&)> rw; bool global; };
  std::map<std::string, classinfo> ClassMap {
    {"EventResData", {&EventResData, true}},
    {"WavResData", {nullptr, true}},
    {"SwitchResData", {nullptr, true}},
    {"RandomResData", {nullptr, true}},
    {"ProgramResData", {nullptr, true}},
    {"WaveFileIdObj", {nullptr, true}},
  };
  
  
  bool entryRW(entry &e) {
    string classname;
    memset(classname, 0, sizeof classname);
    uint32 classnameLength = 0;
    //if (stream.mode == STREAM_MODE_WRITE) classname_length = hx_class_name(entry->i_class, hx->version, classname, HX_STRING_MAX_LENGTH);
    stream.rw(classnameLength);
    
    if (stream.mode == memory::stream::read)
      memset(classname, 0, classnameLength + 1);
    stream.string(classname, classnameLength);
    
    if (stream.mode == memory::stream::read) {
      //const enum class_type c = ClassFromString(class);
      //      const enum class_type c = hx_class_from_string(classname);
      //      if (hclass != entry->i_class) {
      //        return hx_error(hx, "header class name does not match index class name (%X != %X)\n", entry->i_class, hclass);
      //      }
    }
    
    cuuid cuuid = e.cuuid;
    stream.rw(cuuid);
    if (cuuid != e.cuuid) {
      //return hx_error(hx, "header cuuid does not match index cuuid (%016llX != %016llX)\n", entry->i_cuuid, cuuid);
    }
    
    if (ClassMap.find(e.class_) != ClassMap.end()) {
      classinfo& c = ClassMap[e.class_];
      c.rw(*this, e);
    }
  }
  
  /// Add a new entry into this context
  void addEntry(const entry& e) {
    entries[e.cuuid] = e;
  }
  
  /// Find an entry by cuuid
  entry *findEntry(cuuid cuuid) {
    if (entries.find(cuuid) == entries.end())
      return NULL;
    return &entries[cuuid];
  }
  
  
  void write(std::function<void(std::string path, void* data, size_t size)>) {
    
  }
  
  inline bool r() { return stream.mode == memory::stream::mode::read; }
  inline bool w() { return stream.mode == memory::stream::mode::write; }
  
private:
  memory::stream stream;
  // Entries in this context
  std::map<cuuid, entry> entries;
  // The current working directory
  std::filesystem::path workDirectory;
  // List of open files, so we don't have to open new ones all the time.
  std::map<std::filesystem::path, std::fstream*> fileMap;
  
//  static const enum class_type classFromString(char* name) {
//    if (*name++ != 'C') return class_type::Invalid;
//    if (!std::strncmp(name, "PC", 2)) name += 2;
//    if (!std::strncmp(name, "GC", 2)) name += 2;
//    if (!std::strncmp(name, "PS2", 3)) name += 3;
//    if (!std::strncmp(name, "PS3", 3)) name += 3;
//    if (!std::strncmp(name, "XBox", 4)) name += 4;
//    if (!std::strncmp(name, "EventResData", 12)) return class_type::EventResData;
//    if (!std::strncmp(name, "WavResData", 10)) return class_type::WavResData;
//    if (!std::strncmp(name, "SwitchResData", 13)) return class_type::SwitchResData;
//    if (!std::strncmp(name, "RandomResData", 13)) return class_type::RandomResData;
//    if (!std::strncmp(name, "ProgramResData", 14))return class_type::ProgramResData;
//    if (!std::strncmp(name, "WaveFileIdObj", 13)) return class_type::WaveFileIdObject;
//    return class_type::Invalid;
//  }
  
  char* classname(std::string n) {
    char* data;
    if (stream.mode == memory::stream::read) {
      uint32 length;
      stream.rw(length);
      size_t sz = length;
      data = new char[sz];
      stream.string(data, sz);
      data[sz] = '\0';
    } else if (stream.mode == memory::stream::write) {
      
    }
    return data;
  }
  
  std::fstream *openFile(std::filesystem::path p, bool write = false) {
    std::string filename = workDirectory.string() + p.filename().string();
    if (fileMap.find(filename) != fileMap.end()) return fileMap[filename];
    return fileMap[filename] = new std::fstream(filename, std::ios::binary | std::ios::out | (std::ios::in * !write));
  }
  
  void* fileRead(std::filesystem::path path, size_t pos, size_t& size) {
    std::fstream *fs = openFile(path);
    if (fs->is_open()) {
      fs->seekg(0, std::ios_base::end);
      size_t real_size = fs->tellg();
      if (size > real_size)
        size = real_size;
      fs->seekg(pos);
      char* data = new char[size];
      fs->read(data, size);
      fileMap[path] = fs;
      return data;
    }
  }
}; /* context */

#undef HX_STRING_MAX_LENGTH
} /* hx */
} /* format */

#pragma mark - Pointer constants -

#if platform == GCN
# define PTR_EngineStructure        0x803E7C0C // struct
# define PTR_InputStructure         0x8042F5A8 // struct
# define PTR_FixMemory              0x804334CC
# define PTR_LevelMemory            0x804334D0
# define PTR_RandomStructure        0x80436924 // struct
# define PTR_GhostMode              0x805D8580 // byte
# define PTR_InactiveDynamicWorld   0x805D8594 // dptr
# define PTR_FatherSector           0x805D8598 // dptr
# define PTR_DynamicWorld           0x805D859C // dptr
# define PTR_ActualWorld            0x805D85A0 // dptr
// these are probably part of some structure
# define PTR_MenuSelectionV         0x805D884C
# define PTR_MenuOptionRumble       0x805D89B0

# define PTR_MechanicsObstacleArray 0x805D73F8
# define PTR_CollisionGV            0x803DC4F4
# define PTR_GLI_BitmapBuffer       0x80EEFAE8 // dptr
#endif

#pragma mark - Globals -

namespace global {
CPA_EXTERN pointer<stAlways> g_stAlways;
CPA_EXTERN pointer<stEngineStructure> g_stEngineStructure;
CPA_EXTERN pointer<stObjectType> g_stObjectTypes;
CPA_EXTERN pointer<IPT::stInputStructure> g_stInputStructure;
CPA_EXTERN pointer<RND::stRandom> g_stRandomStructure;
CPA_EXTERN pointer<stSuperObject> p_stActualWorld;
CPA_EXTERN pointer<stSuperObject> p_stDynamicWorld;
CPA_EXTERN pointer<stSuperObject> p_stInactiveDynamicWorld;
CPA_EXTERN pointer<stSuperObject> p_stFatherSector;
CPA_EXTERN pointer<uint8> g_bGhostMode;
};

#pragma mark - Runtime -

#ifdef CPATOOLS_IMPLEMENTATION

namespace memory {
/// Size of the memory space
size_type size = 0;
/// The base address of the engine
host_address_type baseAddress = nullptr;
/// Flags: CPA_MEMORY_...
unsigned flags = CPA_MEMORY_READONLY;

struct __default_allocator {
  static void* alloc(size_type sz) {
    // Don't care if context not loaded
    if (!baseAddress) return nullptr;
    
    if (flags & CPA_MEMORY_EXTERNAL) {
      // Unknown memory allocation method
      return nullptr;
    } else {
      
    }
  }
  
  static void dealloc(void *p) {
    // Don't care if context not loaded
    if (!baseAddress) return;
    
    if (flags & CPA_MEMORY_EXTERNAL) {
      // Unknown memory allocation method
    } else {
      
    }
  }
};

allocator_function alloc = __default_allocator::alloc;
deallocator_function dealloc = __default_allocator::dealloc;
std::unordered_map<memory::target_address_type, userdata_store> userdata;

}; /* memory */

namespace global {

pointer<stAlways> g_stAlways = nullptr;
pointer<stEngineStructure> g_stEngineStructure = nullptr;
pointer<stObjectType> g_stObjectTypes = nullptr;
pointer<IPT::stInputStructure> g_stInputStructure = nullptr;
pointer<RND::stRandom> g_stRandomStructure = nullptr;
pointer<stSuperObject> p_stActualWorld = nullptr;
pointer<stSuperObject> p_stDynamicWorld = nullptr;
pointer<stSuperObject> p_stInactiveDynamicWorld = nullptr;
pointer<stSuperObject> p_stFatherSector = nullptr;
pointer<uint8> g_bGhostMode = nullptr;

static void cacheObjectTypes();

static bool isValidState() {
  if (!g_stEngineStructure) return false;
  return //!g_stEngineStructure->engineFrozen
//  &&      g_stEngineStructure->mode != 5
//  &&      g_stEngineStructure->mode != 6
  /*&&*/      p_stActualWorld
  &&      p_stDynamicWorld
  &&      p_stInactiveDynamicWorld
  &&      p_stFatherSector;
}

/// Load context from live memory
static bool loadMemory(memory::host_address_type mem, memory::size_type size) {
  memory::baseAddress = mem;
  memory::size = size;
  
  g_stEngineStructure = pointer<stSuperObject>         (PTR_EngineStructure);
  g_stInputStructure  = pointer<IPT::stInputStructure> (PTR_InputStructure);
  g_stRandomStructure = pointer<RND::stRandom>         (PTR_RandomStructure);
  g_bGhostMode        = pointer<uint8>                 (PTR_GhostMode);

  p_stActualWorld          = *doublepointer<stSuperObject>(PTR_ActualWorld);
  p_stDynamicWorld         = *doublepointer<stSuperObject>(PTR_DynamicWorld);
  p_stInactiveDynamicWorld = *doublepointer<stSuperObject>(PTR_InactiveDynamicWorld);
  p_stFatherSector         = *doublepointer<stSuperObject>(PTR_FatherSector);

  if (isValidState()) {
    pointer<uint8> fix = *doublepointer<uint8>(PTR_FixMemory);
    pointer<uint8> lvl = *doublepointer<uint8>(PTR_LevelMemory);

    #pragma mark FIX
    fix += (32 + 4); // 4=identity matrix
    fix += 4; // localizationStructure
    uint32 levelNameCount = *(uint32*)fix;
    fix += 4;
    uint32 demoNameCount = *(uint32*)fix;
    fix += 4;
    fix += 12 * demoNameCount;
    fix += 12 * demoNameCount;
    fix += 30 * levelNameCount;
    fix += 30 + 2; // First level name + padding
    fix += 4 + 4; // Language count + language offset
    uint32 fixTextureCount = *(uint32*)fix;

    #pragma mark LVL
    lvl += 4 * 4; // ?
    lvl += 24; // text
    lvl += 4 * 60; // ?
    uint32 lvlTextureCount = *(uint32*)lvl;
    lvl += 4;
    lvl += (lvlTextureCount - fixTextureCount) * 4 * 2;
    lvl += 4 * 5; // actualWorld (0), dynamicWorld (0), inactiveDynamicWorld (0), fatherSector (0), firstSubmapPosition
    g_stAlways = lvl;
    lvl += sizeof *g_stAlways;
    g_stObjectTypes = lvl;

    cacheObjectTypes();
  }
  
  return true;
}

/// Load context from level file
static bool loadLevel(const std::filesystem::path path, bool forceReload = false) {
#if engine == R3
  std::filesystem::path tmp = path;
  tmp.replace_extension();
  std::string basename = tmp.filename();
  
  using LevelPointerPair = std::pair<std::filesystem::path, std::filesystem::path>;
  std::map<format::ptr::FileID, LevelPointerPair> gamedata;
  
  for (std::string s : { "", "_vb", "kf" }) {
    std::filesystem::path lvl = path, ptr = path;
    lvl.replace_extension("");
    ptr.replace_extension("");
    lvl.replace_filename(lvl.filename().string() + s + ".lvl");
    ptr.replace_filename(ptr.filename().string() + s + ".ptr");
    
    if (!std::filesystem::exists(lvl)) {
      std::cerr << "failed to load " << lvl << "\n";
      return false;
    }
    
    if (!std::filesystem::exists(ptr)) {
      std::cerr << "failed to load " << ptr << "\n";
      return false;
    }
    
    LevelPointerPair p = std::make_pair(lvl, ptr);
    if (s == "") gamedata[format::ptr::FileID_LVL] = p;
    if (s == "_vb") gamedata[format::ptr::FileID_VB] = p;
    if (s == "kf") gamedata[format::ptr::FileID_KF] = p;
  }
  
  for (auto& p : gamedata) {
    LevelPointerPair pair = p.second;
    printf("%s\n", pair.second.string().c_str());
    
    memory::stream s(pair.second, memory::stream::mode::read);
    format::ptr ptr(s);
    
    for (auto& pp : ptr.pointers) {
      
    }
  }

#endif
  
  std::ifstream fs(path, std::ios::in);
  if (!fs.is_open())
    return false;
  
  
  
  fs.close();
  
  return true;
}

static void setAllocator(memory::allocator_function alloc, memory::deallocator_function dealloc) {
  memory::alloc = alloc;
  memory::dealloc = dealloc;
}
  
#pragma mark - Object type names

struct objectNameCache {
  std::vector<std::string> familyNames;
  std::vector<std::string> modelNames;
  std::vector<std::string> instanceNames;
};

std::map<std::string, objectNameCache> objectNameCacheTable;

static void cacheObjectTypes() {
  if (objectNameCacheTable.find(g_stEngineStructure->currentLevelName) == objectNameCacheTable.end()) {
    objectNameCache& cache = objectNameCacheTable[g_stEngineStructure->currentLevelName];
    g_stObjectTypes->family.forEach([&](stObjectTypeElement* e, void*) { cache.familyNames.push_back(std::string(e->name)); });
    g_stObjectTypes->model.forEach([&](stObjectTypeElement* e, void*) { cache.modelNames.push_back(std::string(e->name)); });
    g_stObjectTypes->instance.forEach([&](stObjectTypeElement* e, void*) { cache.instanceNames.push_back(std::string(e->name)); });
  }
}

auto objectTypeNameLookup(int type, int idx) -> std::string {
  if (objectNameCacheTable.find(g_stEngineStructure->currentLevelName) != objectNameCacheTable.end()) {
    objectNameCache& cache = objectNameCacheTable[g_stEngineStructure->currentLevelName];
    try {
      if (type == ObjectType::Family) return cache.familyNames.at(idx);
      if (type == ObjectType::Model) return cache.modelNames.at(idx);
      if (type == ObjectType::Instance) return cache.instanceNames.at(idx);
    } catch (std::out_of_range& e) {
      return "Invalid name";
    }
  }
  return "Invalid name";
}
  
} /* global */

#endif

} /* cpa */

#endif /* _CPATOOLS_HPP_ */
