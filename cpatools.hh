#ifndef CPATOOLS_HH
#define CPATOOLS_HH

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
#define R2_PC                   MAKE_VERSION(00, ENGINE_VERSION_R2, PC)
#define R2_PC_DEMO_1999_08_18   MAKE_VERSION(01, ENGINE_VERSION_R2, PC)
#define R2_PC_DEMO_1999_09_04   MAKE_VERSION(02, ENGINE_VERSION_R2, PC)
#define R2_PS1                  MAKE_VERSION(03, ENGINE_VERSION_R2, PS1)
#define R2_PS2                  MAKE_VERSION(04, ENGINE_VERSION_R2, PS2)
#define R2_N64                  MAKE_VERSION(05, ENGINE_VERSION_R2, N64)
#define R2_NDS                  MAKE_VERSION(06, ENGINE_VERSION_R2, NDS)
#define R2_N3DS                 MAKE_VERSION(07, ENGINE_VERSION_R2, N3DS)
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
#include <limits>
#include <functional>

namespace CPA {

#pragma mark - Memory -

#define CPA_MEMORY_READONLY (1 << 0)
#define CPA_MEMORY_EXTERNAL (1 << 1)

namespace Memory {
using SizeType = size_t;
using HostAddressType = void*;
using TargetAddressType = uint32_t;

#if endianness == CPA_BIG_ENDIAN
constexpr static std::endian Endianness = std::endian::big;
#elif endianness == CPA_LITTLE_ENDIAN
constexpr static std::endian Endianness = std::endian::little;
#else
# error Unknown endianness
#endif

#if defined(CPA_TARGET_EMULATOR) && (platform == GCN)
constexpr static auto PhysicalAddressingMask = 0x80000000;
constexpr static auto EffectiveAddressingMask = 0x7FFFFFFF;
#else
constexpr static auto PhysicalAddressingMask = std::numeric_limits<TargetAddressType>::min();
constexpr static auto EffectiveAddressingMask = std::numeric_limits<TargetAddressType>::max();
static_assert(PhysicalAddressingMask == 0, "Expected unsigned address type");
#endif

/// The base address of the memory space
extern HostAddressType MemoryBaseAddress;
/// Total size of the memory space
extern SizeType MemorySize;
/// ``CPA_MEMORY...``
extern int MemoryFlags;

static inline uint16_t bswap16(uint16_t value) { return value << 8 | value >> 8; }
static inline uint32_t bswap32(uint32_t value) { return (uint32_t)bswap16(value) << 16 | bswap16(value >> 16); }
static inline uint64_t bswap64(uint64_t value) { return (uint64_t)bswap32(value) << 32 | bswap32(value >> 32); }

template<typename T>
static inline T constexpr bswap(const T v) {
  if constexpr (Endianness != std::endian::native && std::is_integral<T>::value) {
    if constexpr (sizeof(T) == 1) return v;
    if constexpr (sizeof(T) == 2) return bswap16(v);
    if constexpr (sizeof(T) == 4) return bswap32(v);
    if constexpr (sizeof(T) == 8) return bswap64(v);
  } else {
    return v;
  }
}

/// Returns true if the specified address is within the memory range
template<typename T>
static inline bool MemoryBound(const T* addr) {
  return intptr_t(addr) >= intptr_t(MemoryBaseAddress) && intptr_t(addr) <= intptr_t(MemoryBaseAddress) + MemorySize;
}

/// An address type on the target platform
struct Address {
  Address() = default;
  static constexpr auto Invalid = 0;
  
  Address(const TargetAddressType physicalAddress) {
    addr = bswap(physicalAddress);
  }
  
  Address(const HostAddressType hostAddress) {
    intptr_t offset = reinterpret_cast<intptr_t>(hostAddress) - reinterpret_cast<intptr_t>(MemoryBaseAddress);
    if (hostAddress)
      addr = bswap(static_cast<TargetAddressType>(offset));
    else
      addr = Address::Invalid;
  }
  
  /// Physical hardware address
  inline TargetAddressType physicalAddress() const {
    return bswap(addr) | PhysicalAddressingMask;
  }
  
  /// Effective (emulated) address
  inline TargetAddressType effectiveAddress() const {
    return bswap(addr) & EffectiveAddressingMask;
  }
  
  /// Host platform address
  inline HostAddressType hostAddress() const {
    if (!*this) return nullptr;
    intptr_t offset = reinterpret_cast<intptr_t>(MemoryBaseAddress) + static_cast<intptr_t>(effectiveAddress());
    return reinterpret_cast<HostAddressType>(offset);
  }
  
  inline operator TargetAddressType() const {
    return addr;
  }
  
  template<typename T> inline operator T*() const {
    return static_cast<T*>(hostAddress());
  }
  
  inline bool operator==(Address other) const {
    return effectiveAddress() == other.effectiveAddress();
  }
  
private:
  TargetAddressType addr { Address::Invalid };
};

/// A type convertible to and from target platform memory
/// `T0` = Base type, `T1` = Operator type
template<typename T0, typename T1>
struct Type {
  Type() = default;
  Type(const T1 value) : data(bswap(*(T0*)&value)) { /* ... */ }
  
  inline Type& operator=(const T1 value) {
    if (writable())
      data = bswap(*(T0*)&value);
    return *this;
  }
  
  inline operator T1() const {
    T0 tmp = bswap(*const_cast<T0*>(&data));
    return *(T1*)(&tmp);
  }
  
  inline bool memoryBound() const {
    return MemoryBound(&data);
  }
  
  inline bool writable() const {
    if (!memoryBound())
      return true;
    else
      return !(Memory::MemoryFlags & CPA_MEMORY_READONLY);
  }
  
  inline T1 operator+(std::integral auto value) const { return T1(data) + value; }
  inline T1 operator-(std::integral auto value) const { return T1(data) - value; }
  inline T1 operator*(std::integral auto value) const { return T1(data) * value; }
  inline T1 operator/(std::integral auto value) const { return T1(data) / value; }
  
  inline T1 operator+=(T1 other) { return *this = *this + other;  }
  inline T1 operator-=(T1 other) { return *this = *this - other;  }
  inline T1 operator*=(T1 other) { return *this = *this * other;  }
  inline T1 operator/=(T1 other) { return *this = *this / other;  }
  inline T1 operator -(/*....*/) { return -T1(*this); }
  
  inline Type& operator++() { *this += 1; return *this; }
  inline Type& operator--() { *this -= 1; return *this; }
  inline Type  operator++(int) { Type v = *this; ++(*this); return v; }
  inline Type  operator--(int) { Type v = *this; --(*this); return v; }
  
  inline auto operator|=(T1 other) { *this = *this | other; return *this; }
  inline auto operator&=(T1 other) { *this = *this & other; return *this; }
  inline auto operator^=(T1 other) { *this = *this ^ other; return *this; }
  
  using UnderlyingType = T1;
private:
  T0 data = 0;
};

/// A pointer
template<typename T = Address>
struct Pointer {
  Pointer() = default;
  Pointer(Address addr) { ptr = addr; }
  Pointer(HostAddressType addr) { ptr = addr; }
  
  template<typename S>
  Pointer(const Pointer<S> other) {
    ptr = other.ptr;
  }
  
  inline Address pointee() const {
    return ptr;
  }
  
  template<typename S = T>
  inline operator S*() const {
    return pointee();
  }
  
  inline T* operator->() const {
    if (!*this) throw std::runtime_error("bad pointer");
    return pointee();
  }
  
  inline T& operator*() const {
    if (!*this) throw std::runtime_error("bad pointer dereference");
    return *static_cast<T*>(pointee());
  }
  
  inline T& operator[](auto idx) {
    if (!*this) throw std::runtime_error("array access into bad pointer");
    return *(static_cast<T*>(pointee()) + idx);
  }
  
  inline T& Dereference() const {
    return **this;
  }
  
  inline operator bool() const {
    return bool(ptr);
  }
  
  inline Address memoryOffset() { return &ptr; }
  
  inline Pointer operator+(auto offset)  { return (uint8_t*)pointee() + sizeof(T) * offset; }
  inline Pointer operator-(auto offset)  { return (uint8_t*)pointee() - sizeof(T) * offset; }
  inline Pointer operator++()            { return *this = *this + 1;                        }
  inline Pointer operator++(auto)        { auto t = *this; *this = *this + 1; return t;     }
  inline Pointer operator+=(auto offset) { return (*this = *this + offset);                 }
  inline Pointer operator-=(auto offset) { return (*this = *this - offset);                 }
  
  // for use as keys in std::map
  inline bool operator<(const Pointer<T>& other) const { return ptr < other.ptr; }
  inline bool operator>(const Pointer<T>& other) const { return ptr > other.ptr; }
  
  template<typename S = T>
  inline bool operator==(const Pointer<S> other) { return ptr == other.ptr; }
  
  
  using UnderlyingType = T;
  
  Address ptr;
};

/// A string, zero-terminated unless size specified
template<const SizeType size = 0>
struct String {
  String() = default;
  static constexpr bool FixedSize { size != 0 };
  
  constexpr size_t Length() const {
    if constexpr (FixedSize) {
      return size;
    } else {
      return str.length();
    }
  }
  
  std::string lastPathComponent() {
    std::string string = *this;
    size_t idx = string.rfind(':');
    if (idx == std::string::npos) return "";
    return string.substr(idx + 1);
  }
  
  inline void operator=(std::string string) {
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
  inline Address memoryOffset() { return &str; }
  /// Is the string bound to the address space of the target?
  inline bool memoryBound() { return MemoryBound(str); }
  /// Is the memory of the string writable?
  inline bool writable() { return !memoryBound() ? true : !(MemoryFlags & CPA_MEMORY_READONLY); }
  
  inline operator std::string() {
    if constexpr (FixedSize) return std::string(static_cast<char*>(str), Length());
    else return str;
  }
  
  inline const char* c_str() { return reinterpret_cast<char*>(&str); }
  inline operator const char*() { return c_str(); }
  inline bool operator==(const char *str) { return std::string(str) == std::string(str); }
  inline bool operator==(std::string str) { return std::string(str) == std::string(str); }
  
private:
  std::conditional_t<FixedSize, char[size], std::string> str;
};

using Allocator = std::function<void*(SizeType)>;
using Deallocator = std::function<void(void*)>;
// To be set by the user...
extern Allocator Allocate;
extern Deallocator Deallocate;

#define CPA_ALLOCATOR_OP_NEW void* operator new(size_t sz) noexcept(false) { \
  if (CPA::Memory::Allocate) return CPA::Memory::Allocate(sz); \
  else throw "allocator not set"; \
}

#define CPA_ALLOCATOR_OP_DELETE void operator delete (void *p) noexcept(false) { \
  if (CPA::Memory::Deallocate) CPA::Memory::Deallocate(p); \
  else throw "deallocator not set"; \
}

#define CPA_ALLOCATOR \
  CPA_ALLOCATOR_OP_NEW \
  CPA_ALLOCATOR_OP_DELETE

}; /* Memory */

using char8   = Memory::Type<int8_t, int8_t>;
using uchar8  = Memory::Type<uint8_t, uint8_t>;
using int8    = Memory::Type<int8_t, int8_t>;
using uint8   = Memory::Type<uint8_t, uint8_t>;
using int16   = Memory::Type<int16_t, int16_t>;
using uint16  = Memory::Type<uint16_t, uint16_t>;
using int32   = Memory::Type<int32_t, int32_t>;
using uint32  = Memory::Type<uint32_t, uint32_t>;
using int64   = Memory::Type<int64_t, int64_t>;
using uint64  = Memory::Type<uint64_t, uint64_t>;
using float32 = Memory::Type<uint32_t, float>;

template<typename T = Memory::Address> using pointer = Memory::Pointer<T>;
template<typename T = Memory::Address> using doublepointer = pointer<pointer<T>>;
template<Memory::SizeType Size = 0ull> using string = Memory::String<Size>;


#pragma mark - Structure -

//namespace GAM {
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
struct stAnim3D;
//}

namespace MTH {
template<typename T, unsigned N> struct stVector;
template<typename T, unsigned N> struct stMatrix;

using stVector2D = stVector<float32, 2>;
using stVector3D = stVector<float32, 3>;
using stVector4D = stVector<float32, 4>;
using stMatrix3D = stMatrix<float32, 3>;
#if engine >= R3
using stMatrix4D = stMatrix<float32, 4>;
#endif
};

#if engine == ENGINE_VERSION_R3
namespace MAT {
struct stTransformation;
}
#endif

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

/// ISI
namespace ISI {
struct stLOD;
struct stISI;
struct stColor;
}

/// AI module
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
}

/// Geometry module
namespace GEO {
struct stGeometricObject;
struct stVisualSet;
struct stVisualElementIndexedTriangles;
struct stColor;
struct stParallelBox;
union uVisualObject;
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
struct stZBufferForLight;
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

#pragma mark -*-

/*************************/
/** ``STRUCTURE BEGIN`` **/
/*************************/

using Index3D = uint16;

#define concat(a, b) concat_inner(a, b)
#define concat_inner(a, b) a ## b
#define unique_name(base) concat(base, __LINE__)
#define padding(S) private: uint8_t unique_name(padding) [S]; public:

#pragma pack(push, 1)

#pragma mark - MTH -

template<typename T, unsigned N>
struct MTH::stVector {
  stVector() { /* ... */ }
  
  stVector(float v) {
    for (int i=0; i<N; i++)
      data[i] = v;
  }
  
  template<unsigned N2>
  stVector(std::array<float32, N2>& vec) {
    for (int i=0; i<N; i++)
      data[i] = vec[i];
  }
  
  template<typename... Args, std::enable_if_t<sizeof...(Args) == N && sizeof...(Args) != 1
  && std::conjunction_v<std::is_convertible<Args, float>...>>* = nullptr>
  stVector(Args... args) : data { static_cast<float>(args)... } { /* ... */ }
  
  template<typename S>
  stVector(const MTH::stVector<S,N>& v) {
    for (int i = 0; i < N; i++)
      data[i] = v[i];
  }
  
  inline float dot(const stVector& v) const {
    float s = 0.0f;
    for (int i = 0; i < N; i++)
      s += data[i] * v[i];
    return s;
  }
  
  inline float length() const {
    return sqrt(dot(*this));
  }
  
  inline stVector cross(const stVector& v) const {
    stVector result;
    result[0] = data[1] * v.data[2] - data[2] * v.data[1];
    result[1] = data[2] * v.data[0] - data[0] * v.data[2];
    result[2] = data[0] * v.data[1] - data[1] * v.data[0];
    return result;
  }
  
  inline stVector normalize() const {
    stVector result = *this;
    if (length() == 0.0f)
      return *this;
    
    float scale = 1.0f / length();
    for (int i = 0; i < N; i++)
      result[i] *= scale;
    
    return result;
  }
  
  inline T& x() const { return *(T*)&data[0]; }
  inline T& y() const { return *(T*)&data[1]; }
  inline T& z() const { return *(T*)&data[2]; }
  inline T& w() const { return *(T*)&data[3]; }
  inline T& operator[](int i) const { return *(T*)(&data[i]); }
  inline MTH::stVector2D xy() const { return *(MTH::stVector2D*)data.data(); }
  inline MTH::stVector3D xyz() const { return *(MTH::stVector3D*)data.data(); }
  
  stVector operator +(const stVector& v) const { stVector result; for (int i=0; i<N; ++i) result[i] = data[i] + v[i]; return result; }
  stVector operator -(const stVector& v) const { stVector result; for (int i=0; i<N; ++i) result[i] = data[i] - v[i]; return result; }
  stVector operator *(const stVector& v) const { stVector result; for (int i=0; i<N; ++i) result[i] = data[i] * v[i]; return result; }
  stVector operator /(const stVector& v) const { stVector result; for (int i=0; i<N; ++i) result[i] = data[i] / v[i]; return result; }
  stVector operator *(const auto scalar) const { stVector result; for (int i=0; i<N; ++i) result[i] = data[i] * scalar; return result; }
  stVector operator /(const auto scalar) const { stVector result; for (int i=0; i<N; ++i) result[i] = data[i] / scalar; return result; }
  stVector operator-() const { return *this * -1.0f; }
  
  stVector operator +=(const stVector& v) { *this = *this + v; }
  stVector operator -=(const stVector& v) { *this = *this - v; }
  stVector operator *=(const stVector& v) { *this = *this * v; }
  stVector operator /=(const stVector& v) { *this = *this / v; }
  
  
  bool operator >(const stVector& v) const { for (int i=0; i<N; ++i) if (data[i] <= v[i]) return false; return true; }
  bool operator <(const stVector& v) const { for (int i=0; i<N; ++i) if (data[i] >= v[i]) return false; return true; }
  bool operator>=(const stVector& v) const { for (int i=0; i<N; ++i) if (data[i] <  v[i]) return false; return true; }
  bool operator<=(const stVector& v) const { for (int i=0; i<N; ++i) if (data[i] >  v[i]) return false; return true; }
  bool operator==(const stVector& v) const { for (int i=0; i<N; ++i) if (data[i] != v[i]) return false; return true; }
  bool operator!=(const stVector& v) const { return !(*this == v); }
  
  CPA_ALLOCATOR
private:
  std::array<T, N> data;
};

template<typename T, unsigned N>
struct MTH::stMatrix {
  constexpr stMatrix() {
    for (int i=0; i<N; i++)
      for (int j=0; j<N; j++)
        (*this)(i,j) = (i == j ? 1.0f : 0.0f);
  }
  
  template<typename S>
  stMatrix(const MTH::stMatrix<S,N>& other) {
    for (int i = 0; i < N*N; i++)
      m[i] = other[i];
  }
  
  constexpr static stMatrix identity() {
    return stMatrix{};
  }
  
  inline T& operator()(auto row, auto col) const {
    return *(T*)&m[col + row * N];
  }
  
  inline T& operator[](auto index) const {
    return *(T*)&m[index];
  }
  
  stMatrix operator*(const stMatrix& src) const {
    stMatrix result;
    for (int y=0; y<N; y++)
      for (int x=0; x<N; x++) {
        double sum = 0.0f;
        for (int z=0; z<N; z++)
          sum += src(y, z) * (*this)(z, x);
        result(y,x) = sum;
      }
    return result;
  }
  
  stMatrix& operator*=(const stMatrix& m) const {
    return (*this = *this * m);
  }
  
  MTH::stVector4D operator*(const MTH::stVector4D& v) const {
    MTH::stVector4D result;
    for (int y=0; y<N; y++) {
      result[y] = 0.0f;
      for (int x=0; x<N; x++)
        result[y] += (*this)(x,y) * v[x];
    }
    return result;
  }
  
  MTH::stVector4D operator*(const MTH::stVector3D& v) const {
    return ((*this) * MTH::stVector4D(v.x(), v.y(), v.z(), 1.0f));
  }
  
  constexpr static stMatrix makeTranslation(const MTH::stVector3D& P) {
    stMatrix result = identity();
    for (int i=0; i<3; i++)
      result(N-1,i) = P[i];
    return result;
  }
  
  constexpr static stMatrix makeScale(const MTH::stVector3D& S) {
    stMatrix result = identity();
    for (int i=0; i<3; i++)
      result(i,i) = S[i];
    return result;
  }
  
  constexpr static stMatrix makeRotationX(double radians) {
    stMatrix result = identity();
    result(1,1) = std::cos(radians);
    result(1,2) = std::sin(radians);
    result(2,1) = -std::sin(radians);
    result(2,2) = std::cos(radians);
    return result;
  }
  
  constexpr static stMatrix makeRotationY(double radians) {
    stMatrix result = identity();
    result(0,0) = std::cos(radians);
    result(0,2) = -std::sin(radians);
    result(2,0) = std::sin(radians);
    result(2,2) = std::cos(radians);
    return result;
  }
  
  constexpr static stMatrix makeRotationZ(double radians) {
    stMatrix result = identity();
    result(0,0) = std::cos(radians);
    result(0,1) = std::sin(radians);
    result(1,0) = -std::sin(radians);
    result(1,1) = std::cos(radians);
    return result;
  }
  
  constexpr static MTH::stMatrix4D makePerspective(double fovY, double aspect, double near, double far) {
    float ct = 1.0f / std::tan(fovY / 2.0f);
    stMatrix4D result = identity();
    result(0,0) = ct / aspect;
    result(1,1) = ct;
    result(2,2) = (far + near) / (near - far);
    result(2,3) = -1.0f;
    result(3,2) = (2.0f * far * near) / (near - far);
    result(3,3) = 0.0f;
    return result;
  }
  
  constexpr static MTH::stMatrix4D makeLookAt(const MTH::stVector3D& eye,
                                              const MTH::stVector3D& center,
                                              const MTH::stVector3D& up)
  {
    const MTH::stVector3D& n = (eye - center).normalize();
    const MTH::stVector3D& u = up.cross(n).normalize();
    const MTH::stVector3D& v = n.cross(u);
  
    double const nnx = (-u).dot(eye);
    double const nny = (-v).dot(eye);
    double const nnz = (-n).dot(eye);
  
    stMatrix4D result = identity();
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
  
  stMatrix transpose() const {
    stMatrix result;
    for (int j=0; j<N; j++)
      for (int i=0; i<N; i++)
        result(i,j) = (*this)(j,i);
    return result;
  }
  
  MTH::stMatrix4D inverse() const {
    double const s0 = (*this)(0,0) * (*this)(1,1) - (*this)(1,0) * (*this)(0,1);
    double const s1 = (*this)(0,0) * (*this)(1,2) - (*this)(1,0) * (*this)(0,2);
    double const s2 = (*this)(0,0) * (*this)(1,3) - (*this)(1,0) * (*this)(0,3);
    double const s3 = (*this)(0,1) * (*this)(1,2) - (*this)(1,1) * (*this)(0,2);
    double const s4 = (*this)(0,1) * (*this)(1,3) - (*this)(1,1) * (*this)(0,3);
    double const s5 = (*this)(0,2) * (*this)(1,3) - (*this)(1,2) * (*this)(0,3);
    double const c5 = (*this)(2,2) * (*this)(3,3) - (*this)(3,2) * (*this)(2,3);
    double const c4 = (*this)(2,1) * (*this)(3,3) - (*this)(3,1) * (*this)(2,3);
    double const c3 = (*this)(2,1) * (*this)(3,2) - (*this)(3,1) * (*this)(2,2);
    double const c2 = (*this)(2,0) * (*this)(3,3) - (*this)(3,0) * (*this)(2,3);
    double const c1 = (*this)(2,0) * (*this)(3,2) - (*this)(3,0) * (*this)(2,2);
    double const c0 = (*this)(2,0) * (*this)(3,1) - (*this)(3,0) * (*this)(2,1);
    
    double const det = s0 * c5 - s1 * c4 + s2 * c3 + s3 * c2 - s4 * c1 + s5 * c0;
    double const invdet = 1.0f / det;
    assert(det != 0.0f);
    
    MTH::stMatrix4D result;
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
  
  MTH::stVector3D& translation() const {
    return *(MTH::stVector3D*)&(*this)(N-1,0);
  }
  
  CPA_ALLOCATOR
private:
  std::array<T, N*N> m;
};

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
  
  template<typename F> void forEach(const F& f, void *userdata = nullptr) const {
    pointer<T> ii = first;
    while (ii) { f(ii, userdata); ii = ii->next; }
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
  
  CPA_ALLOCATOR
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
  CPA_ALLOCATOR
};

#pragma mark - stTransformation

#define MAT_TransformationType_Uninitialized         0
#define MAT_TransformationType_Identity              1
#define MAT_TransformationType_Translate             2
#define MAT_TransformationType_Zoom                  3
#define MAT_TransformationType_Scale                 4
#define MAT_TransformationType_Rotation              5
#define MAT_TransformationType_RotationZoom          6
#define MAT_TransformationType_RotationScale         7
#define MAT_TransformationType_ComplexRotationScale  8
#define MAT_TransformationType_Undefined             9

/// World transform
struct MAT::stTransformation {
  stTransformation() = default;
  stTransformation(uint32 type, MTH::stMatrix4D T, MTH::stVector4D scale = 1.0f);
  
  /// Type of transformation
  uint32 type = MAT_TransformationType_Identity;
  /// Transform matrix
  MTH::stMatrix4D matrix = MTH::stMatrix4D::identity();
  /// Scale parameter
  MTH::stVector4D scale;
  
  /// Name of the transformation type
  const std::string TypeName() const;
  /// Translation vector
  MTH::stVector3D& Translation();
  /// Scale vector
  MTH::stVector3D Scale();
  /// Get the inverse of the transformation
  stTransformation Inverse();
  /// Rotate a vector
  MTH::stVector3D RotateVector(MTH::stVector3D v);
  /// Get rotation vectors
  bool GetRotation(MTH::stVector3D& I, MTH::stVector3D& J, MTH::stVector3D& K);
  
  MTH::stVector3D operator*(MTH::stVector3D);
  MTH::stVector4D operator*(MTH::stVector4D);
  stTransformation operator*(MAT::stTransformation);
  
  CPA_ALLOCATOR
};

struct stAlwaysModelList {
  pointer<stAlwaysModelList> next;
  pointer<stAlwaysModelList> prev;
  pointer<stLinkedList<stAlwaysModelList>> parentList;
  int32 objectModelType;
  pointer<stEngineObject> alwaysObject;
  
  CPA_ALLOCATOR
};

struct stAlways {
  uint32 numAlways;
  stDoublyLinkedList<stAlwaysModelList> alwaysModels;
  pointer<stSuperObject> alwaysSuperobject;
  pointer<stEngineObject> alwaysActors;
  pointer<stSuperObject> alwaysGeneratorSuperobjects;
  CPA_ALLOCATOR
};

#define ObjectType_Family   0
#define ObjectType_Model    1
#define ObjectType_Instance 2

/// Object identifier
struct stObjectTypeElement {
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
  
  CPA_ALLOCATOR
};

/// Global object type table
struct stObjectType {
  /// Family object types
  stDoublyLinkedList<stObjectTypeElement> familyList;
  /// Model object types
  stDoublyLinkedList<stObjectTypeElement> modelList;
  /// Instance object types
  stDoublyLinkedList<stObjectTypeElement> instanceList;
  
  void LoadCache();
  void UnloadCache();
  /// Look up a name with a type (`ObjectType_...`) and index
  const std::string LookupName(int type, int index);
  
  CPA_ALLOCATOR
};

#pragma mark - 3DData

struct stState {
  string<0x50> name;
  pointer<stState> next;
  pointer<stState> prev;
  pointer</**/> parentList;
  pointer<stAnim3D> animation;
  CPA_ALLOCATOR
};

struct stSubAnim {
  pointer<stAnim3D> subAnim;
  CPA_ALLOCATOR
};

struct st3DData {
  pointer<> initialState;
  pointer<> currentState;
  pointer<> firstStateOfAction;
  pointer<> initialObjectsTable;
  pointer<> currentObjectsTable;
  pointer<> family;
  CPA_ALLOCATOR
};

#define ObjectTableType_PhysicalObject  0
#define ObjectTableType_Animation       1
#define ObjectTableType_Light           2
#define ObjectTableType_Camera          3
#define ObjectTableType_Mirror          4
#define ObjectTableType_Event           5

struct stObjectTableElement {
  pointer<MTH::stVector3D> scale;
  pointer<PO::stPhysicalObject> object;
  uint32 channelNumber;
  uint16 type;
  uint8 unknown;
  uint8 intensity;
  uint8 expression;
  padding(3)
  
  const std::string TypeName();
};

struct stObjectTable {
  pointer<stObjectTable> next;
  pointer<stObjectTable> prev;
  pointer<> _;
//  LinkedList<> a; // not correct
  pointer<stObjectTableElement> current;
  pointer<stObjectTableElement> initial;
  uint16 numElements;
  uint16 numUsedZdx;
};

/// Global list of object families and
struct stFamilyList {
  pointer<stFamilyList> next;
  pointer<stFamilyList> prev;
  pointer<LinkedListElement<stFamilyList>> list;
  int32 objectFamilyType;
  LinkedList<stState> states;
  LinkedList<stSubAnim> subAnims;
  pointer<stObjectTable> defaultObjectTable;
  LinkedList<stObjectTable> objectTables;
  CPA_ALLOCATOR
};

#pragma mark - Engine

/// High-resolution counter
struct stTimerCount {
  uint32 low;
  uint32 high;
  CPA_ALLOCATOR
};

/// Global engine timer
struct stEngineTimer {
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
  uint32 ticksPerMillisecond;
  CPA_ALLOCATOR
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
struct stEngineStructure {
  stEngineStructure() {
    mode = EngineMode::Initialize;
  }
  
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
  stDoublyLinkedList<stFamilyList> familyList;
  stDoublyLinkedList<> alwaysList;
  stDoublyLinkedList<stSuperObject> mainCharacterList;
  pointer<stSuperObject> standardCamera;
  pointer<stSuperObject> debugCamera;
  pointer<> languageStructure;
  pointer<> levelFilenameList;
  MAT::stTransformation mainActorTransform;
  MAT::stTransformation mainCameraTransform;
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
  void LoadLevel(const std::string& levelName);
  
  CPA_ALLOCATOR
};


#pragma mark - IPT -

/// Structure for ReadAnalogJoystick function
struct IPT::stPadReadingOutput {
  /// The world vector the joystick value translates to
  MTH::stVector3D globalVector;
  int16 horizontalAxis;
  int16 verticalAxis;
  float32 analogForce;
  float32 trueAnalogForce;
  float32 rotationAngle;
  /// Strafe sector (0-7 clockwise)
  int32 strafeSector;
  CPA_ALLOCATOR
};

struct IPT::stInputDevice {
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
  CPA_ALLOCATOR
};

struct IPT::stInputEntryElement {
  padding(6 * 4) /* ? */
  uint32 numKeywords;
  pointer<> keywordArray;
  pointer<string<>> actionName;
  pointer<string<>> entryName;
  int32 state;
  float32 analogValue;
  int8 active;
  padding(3)
  CPA_ALLOCATOR
};

struct IPT::stInputStructure {
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
  CPA_ALLOCATOR
};

#pragma mark - RND -

#define RND_TableIndexCount 0x0032
#define RND_TableCount 0x2710
#define RND_DefaultIndex 0x0000

struct RND::stRandom {
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
  int32_t Index(unsigned i) {
    uint32_t* T = table;
    return T ? ((T[i % RND_TableCount] >> 16) & 0x7FFF) : 0;
  }
  
  /// Index the random number table using an index from tableIndices, optionally offset
  int32_t IndexRelative(unsigned TableIndicesIdx, unsigned Offset) {
    return Index(uint32_t(tableIndices[TableIndicesIdx]) + Offset);
  }
  
  /// Simulate `Count` calls into the RND table, bounding the value by `Min` and `Max`
  int32_t call(unsigned const Count, unsigned const Min, unsigned const Max, unsigned const Index = RND_DefaultIndex) {
    int32_t n, v = 0;
    for (n = 0; n < Count; n++)
      v = (Min + ((Max + 1 - Min) * IndexRelative(Index, n)) / (tableMax + 1));
    return v;
  }
  
  CPA_ALLOCATOR
};

#pragma mark - 3D

enum EventType {
  SoundEvent = 0,
  MechanicsEvent = 1,
  GenerateEvent = 2,
  GenericEvent = 3,
};

union uEventData {
  doublepointer<SND::stBlockEvent> soundEvent;
};

struct stEventInTable {
  uint32 unknown;
  /// Event-specific data
  uEventData eventData;
  /// Type of this event
  uint8 eventType;
  uint8 priority;
  uint8 firstCall;
  uint8 period;
  uint32 semaphoreID;
  
  uint32 unknown2;
  CPA_ALLOCATOR
};
  
struct stEvent {
  /// Pointer to the event in the global event table.
  pointer<stEventInTable> eventInTable;
  /// Index of this element in the global event table.
  uint16 eventTableIndex;
  uint16 frameNumber;
  uint16 channelNumber;
  uint16 isLocalized;
  //pointer<stEventInTable> eventInTable2;
  CPA_ALLOCATOR
};

struct stAnim3D {
  /// The filename of this animation
  string<0x50> name;
  /// Number of frames in this animation
  uint16 numFrames;
  /// The rate at which the animation is played
  uint8 frameRate;
  
  uint8 maxNumElements;
  /// The list of events in this animation
  pointer<stEvent> eventList;
  pointer</*stMorphData*/> morphDataList;
  ushort numGeneralA3D;
  /// The number of events in this animation
  uint8 numEvents;
  ///
  uint8 mergeAnimationFlag;
  CPA_ALLOCATOR
};

struct stActiveSubAnim {
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
  CPA_ALLOCATOR
};


#pragma mark - CINE -

/// Actor state in a cinematic
struct CINE::stCineActor {
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
  CPA_ALLOCATOR
};

/// A cinematic
struct CINE::stCine {
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
  CPA_ALLOCATOR
};

/// Cinematics state manager
struct CINE::stCineManager {
  /// List of level cinematics
  stDoublyLinkedList<stCine> cineList;
  /// Padding
#if platform == PS2
  padding(4)
#endif
  /// Force camera transform
  MAT::stTransformation fixedCameraTransform;
  /// Padding
#if platform == PS2
  padding(2)
#endif
  /// Currently active cutscene camera
  pointer<stSuperObject> activeCamera;
  CPA_ALLOCATOR
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
struct DNM::stDynamicsRotation {
  float32 angle;
  MTH::stVector3D axis;
  CPA_ALLOCATOR
};

/// Dynamics base block
struct DNM::stDynamicsBaseBlock {
  /// Type of the object
  int32 objectType;
  /// Current mechanics ID card
  pointer<> IDCard;
  /// Flags which control the current physics state of the actor. (`DNM_Flag_...`)
  uint32 flags;
  /// Info/verification flags. These are used internally by the mechanics engine
  /// to determine how the control flags should be applied. (`DNM_EndFlag_...`)
  uint32 endFlags;
  /// The current gravity factor (default is 9.81f)
  float32 gravity;
  /// Slope limit (1.0f)
  float32 slopeLimit;
  /// Wall/ground limit (45 degrees)
  float32 slopeCosine;
  /// Ground slide factor
  float32 slide;
  /// Rebound factor
  float32 rebound;
  /// Impose absolute speed (applied after inertia and gravity calculations)
  /// For this to work, one of the `DNM_Flag_SpeedImpose...` flags must be set.
  MTH::stVector3D imposeSpeed;
  /// Propose speed (applied before inertia and gravity calculations)
  /// For this to work, one of the `DNM_Flag_SpeedPropose...` flags must be set.
  MTH::stVector3D proposeSpeed;
  /// The speed determined by the previous call to the dynamics.
  /// This is the current speed for any particular frame.
  MTH::stVector3D previousSpeed;
  /// Scale factor by which to multiply scale-based physics parameters
  MTH::stVector3D scale;
  /// Speed which is specific to a particular animation state.
  /// Applied before inertia and gravity calculations.
  MTH::stVector3D animationSpeed;
  /// Previous safe translation point
  MTH::stVector3D safeTranslation;
  /// Additional translation
  MTH::stVector3D addTranslation;
  
#if engine == R3 && platform == GCN
  padding(8)
#endif
  
  /// The previous transformation.
  MAT::stTransformation previousTransform;
  /// The current transformation.
  MAT::stTransformation currentTransform;
  /// Impose absolute rotation
  MTH::stMatrix3D imposedRotation;
  /// Previous number of frames
  uint8 numFrames;
  
  padding(3)
  /// Collision report, copied from the mechanics at the end of a dynamics frame.
  pointer<stDynamicsReport> report;
  
#if engine == R3 && platform == PS2
  /// Padding
  padding(8)
#endif
  
  CPA_ALLOCATOR
};

/// Dynamics advanced block
struct DNM::stDynamicsAdvancedBlock {
  float32 xInertia;
  float32 yInertia;
  float32 zInertia;
  
  /// Priority of stream
  float32 streamPriority;
  /// Stream effect factor
  float32 streamFactor;
  
  /// Slide factor
  float32 xSlideFactor;
  float32 ySlideFactor;
  float32 zSlideFactor;
  
  
  
  /// Previous slide
  float32 previousSlide;
  /// Speed limit
  MTH::stVector3D maxSpeed;
  /// Speed of stream
  MTH::stVector3D streamSpeed;
  /// Speed to add
  MTH::stVector3D addSpeed;
  /// Positional limits?
  MTH::stVector3D limit;
  /// Collision translation
  MTH::stVector3D collisionTranslation;
  /// Translation separate of inertia
  MTH::stVector3D inertiaTranslation;
  /// The normal of the collide ground, if any
  MTH::stVector3D groundNormal;
  /// The normal of the collided wall, if any
  MTH::stVector3D wallNormal;
  /// Number of calls made to mechanics without colliding with anything
  int8 collideCount;
  /// Padding
  padding(3)
  
  CPA_ALLOCATOR
};

/// AI and DNM message-interchange:
/// "Module Allowing the Communication of Datas from the Player or the Intelligence to the Dynamics"
struct DNM::stMACDPID {
  float32 data0;
  MTH::stVector3D data1;
  MTH::stVector3D data2;
  MTH::stVector3D data3;
  float32 data4;
  float32 data5;
  float32 data6;
  stDynamicsRotation data7;
  stDynamicsRotation data8;
  int8 data9;
  uint16 data10;
  MTH::stVector3D data11;
  float32 data12;
  MTH::stVector3D data13;
  float32 data14;
  uint8 data15;
  
  CPA_ALLOCATOR
};

/// Dynamics complex block
struct DNM::stDynamicsComplexBlock {
  float32 tiltStrength;
  float32 tiltInertia;
  float32 tiltOrigin;
  float32 tiltAngle;
  float32 hangingLimit;
  MTH::stVector3D contact;
  MTH::stVector3D fallTranslation;
  /// Injectable parameters
  stMACDPID MACDPID;
  pointer<stSuperObject> platformSuperObject;
  MAT::stTransformation previousMatrixAbsolute;
  MAT::stTransformation previousMatrixPrevious;
  CPA_ALLOCATOR
};

/// Dynamics obstacle reported from mechanics
struct DNM::stDynamicsObstacle {
  /// Collision rate
  float32 rate;
  /// Contact normal
  MTH::stVector3D normal;
  /// World contact point
  MTH::stVector3D contact;
  /// Material for entity 1 (self)
  pointer<GMT::stGameMaterial> myMaterial;
  /// Material for entity 2 (object collided with)
  pointer<GMT::stGameMaterial> collidedMaterial;
  /// Collided object
  pointer<stSuperObject> superObject;
  CPA_ALLOCATOR
};

/// A linear and angular movement offset
struct DNM::stDynamicsMovement {
  /// The linear movement
  MTH::stVector3D linear;
  /// The angular movement
  stDynamicsRotation angular;
  CPA_ALLOCATOR
};

/// Dynamics collision report
struct DNM::stDynamicsReport {
  /// The previous surface state
  uint32 previousSurfaceState;
  /// The current surface state
  uint32 currentSurfaceState;
  
  stDynamicsObstacle genericObstacle;
  stDynamicsObstacle groundObstacle;
  stDynamicsObstacle wallObstacle;
  stDynamicsObstacle characterObstacle;
  stDynamicsObstacle waterObstacle;
  stDynamicsObstacle ceilingObstacle;
  
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
  
  CPA_ALLOCATOR
};

/// Parameters for mechanics engine
struct DNM::stDynamics {
  stDynamicsBaseBlock baseBlock;
  stDynamicsAdvancedBlock advancedBlock;
  stDynamicsComplexBlock complexBlock;
  
  MTH::stVector3D& Speed();
  float HorizontalSpeed();
  float VerticalSpeed();
  
  CPA_ALLOCATOR
};

struct DNM::stDynamicsParsingData {
  MTH::stVector3D position;
  float32 outAlpha;
  MTH::stVector3D vector;
  CPA_ALLOCATOR
};

/// Dynamics reference structure
struct DNM::stDynam {
  pointer<stDynamics> dynamics;
  pointer<stDynamicsParsingData> parsingDatas;
  uint32 usedMechanics;
  CPA_ALLOCATOR
};


#pragma mark - MEC -

// Dynamics obstacle type
#define dynamicsObstacleTypeNothing     0
#define dynamicsObstacleTypeScenery     1
#define dynamicsObstacleTypeMobile      2
#define dynamicsObstacleTypeDoubleEdge  4
#define dynamicsObstacleTypeMobileWall  9

/// Mechanics engine obstacle (used internally)
/// Cast to stCollisionCase and partially to stDynamicsObstacle
struct MEC::stMechanicsObstacle {
  /// Collision rate
  float32 rate;
  /// Contact normal
  MTH::stVector3D normal;
  /// World contact point
  MTH::stVector3D contact;
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
  MTH::stVector3D translation;
  /// Zone movement
  MTH::stVector3D zoneMove;
  /// End position of dynamic object
  MTH::stVector3D zonePosition;
  /// Zone radius of dynamic object
  float32 zoneRadius;
  
  CPA_ALLOCATOR
};

struct MEC::stMechanicsReport {
  /// The current surface state
  uint32_t currentSurfaceState;
  /// Generic obstacle
  stMechanicsObstacle genericObstacle;
  /// Ground obstacle
  stMechanicsObstacle groundObstacle;
  /// Wall obstacle
  stMechanicsObstacle wallObstacle;
  /// Actor obstacle
  stMechanicsObstacle characterObstacle;
  /// Water obstacle
  stMechanicsObstacle waterObstacle;
  /// Ceiling obstacle
  stMechanicsObstacle ceilingObstacle;
  
  CPA_ALLOCATOR
};

#pragma mark - Engine object

struct stStandardGameInfo {
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
  
  CPA_ALLOCATOR
};

/// Engine object - an actor in the dynamic world
struct stEngineObject {
  /// 3D-related parameters
  pointer<st3DData> m_3DData;
  /// Standard game info
  pointer<stStandardGameInfo> stdGame;
  /// Dynamics
  pointer<DNM::stDynam> dynam;
  /// Brain and AI
  pointer<AI::stBrain> brain;
  /// Cinematic-related info of this actor
  pointer<CINE::stCineInfo> cineInfo;
  /// Collision geometry set
  pointer<COL::stCollideSet> collideSet;
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
    
  /// The name of this actor in order of [Instance, Model, Family]
  const std::string Name(int objectType = ObjectType_Instance);
  /// The superobject associated with this actor.
  pointer<stSuperObject> SuperObject() const;
  
  CPA_ALLOCATOR
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

#define SECT_SectorPriority_Min     0
#define SECT_SectorPriority_Normal  64
#define SECT_SectorPriority_Max     127

struct SECT::stSector {
  stDoublyLinkedList<stListOfCharacters> characters;
  stDoublyLinkedList<stListOfStaticLights> staticLights;
  stDoublyLinkedList<stListOfDynamicLights> dynamicLights;
  stDoublyLinkedList<stListOfSectorsInGraphicInteraction> sectorsInGraphInteraction;
  stDoublyLinkedList<stListOfSectorsInCollisionInteraction> sectorsInCollisionInteraction;
  stDoublyLinkedList<stListOfSectorsInActivityInteraction> sectorsInActivityInteraction;
  stDoublyLinkedList<stListOfSectorsInSoundInteraction> sectorsInSoundInteraction;
  stDoublyLinkedList<> soundEventList;
  MTH::stVector3D min;
  MTH::stVector3D max;
  float32 farPlane;
  uint8 isVirtual;
  int8 cameraType;
  int8 counter;
  int8 priority;
  pointer<GLI::stMaterial> skyMaterial;
  uint8 fog;
#if platform == GCN
  string<0x100> name;
#endif
  
  CPA_ALLOCATOR
};

#pragma mark - COL -

// Collide element type
#define COL_ElementType_IndexedTriangles    1
#define COL_ElementType_Facemap             2
#define COL_ElementType_Sprite              3
#define COL_ElementType_TMesh               4
#define COL_ElementType_Points              5
#define COL_ElementType_Lines               6
#define COL_ElementType_IndexedSpheres      7
#define COL_ElementType_AABB                8
#define COL_ElementType_Cones               9
#define COL_ElementType_DeformationSetInfo  13
#define COL_ElementType_Invalid             0xFFFF
  
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

struct COL::stOctreeNode {
  MTH::stVector3D minPoint;
  MTH::stVector3D maxPoint;
  doublepointer<stOctreeNode> childNodes; /* always 8 */
  
  /// Face indices: overlapping indices into element and element data. May be NULL.
  pointer<uint8> faceIndices;
  
  /// The number of elements in the face index list
  inline auto numElements() -> int16 {
    return faceIndices ? *static_cast<int16*>(faceIndices) : int16(0);
  }
  
  CPA_ALLOCATOR
};

struct COL::stOctree {
  pointer<stOctreeNode> rootNode;
  /// Number of faces which this octree encompasses
  int16 numFaces;
  /// Padding
  padding(2)
  /// Element bases table
  pointer<uint16> elementBases;
  /// Minimum point
  MTH::stVector3D min;
  /// Maximum point
  MTH::stVector3D max;
  
  CPA_ALLOCATOR
};

struct COL::stCollideObject {
  /// Number of vertices
  int16 numVertices;
  /// Number of elements
  int16 numElements;
  /// Number of bounding boxes
  int16 numBoundingBoxes;
  /// Padding
  padding(2)
  /// Vertex data
  pointer<MTH::stVector3D> vertices;
  /// Element types
  pointer<int16> elementTypes;
  /// `stCollideElement`...
  doublepointer<> elements;
  /// Octree partitioning
  pointer<stOctree> octree;
  ///
  pointer<> boundingBoxes;
  /// Radius of the bounding sphere which encompasses this object
  float32 boundingSphereRadius;
  /// Position of the bounding sphere which encompasses this object
  MTH::stVector4D boundingSpherePosition;
  
  CPA_ALLOCATOR
};

struct COL::stPhysicalCollideSet {
  pointer<stCollideObject> zdm; ///< Mechanics zone
  pointer<stCollideObject> zdd; ///< Detection zone
  pointer<stCollideObject> zde; ///< Event zone
  pointer<stCollideObject> zdr; ///< Reaction zone
  CPA_ALLOCATOR
};

struct COL::stColliderInfo {
  pointer<stSuperObject> colliderActors[2];
  MTH::stVector3D colliderVectors[2];
  float32 colliderReal[2];
  uint8 colliderType;
  uint8 colliderPriority;
  uint8 unused[2];
  CPA_ALLOCATOR
};

struct COL::stZdxListEntry {
#if platform == GCN
  pointer<stZdxListEntry> next;
  pointer<stZdxListEntry> prev;
  pointer<> parent;
  pointer<stCollideObject> data;
#else
  pointer<stCollideObject> data;
#endif
  
  CPA_ALLOCATOR
};

struct COL::stZdxList {
#if platform == GCN
  stDoublyLinkedList<stZdxListEntry> list;
#else
  stLinkedList<stZdxListEntry> list;
#endif
  uint16 numZdx;
  padding(2)
  
  /// Return a vector of all the collide zdx objects
  std::vector<pointer<stCollideObject>> all();
  
  CPA_ALLOCATOR
};

struct COL::stCsaList {
  stDoublyLinkedList<> list;
  CPA_ALLOCATOR
};

struct COL::stZoneSetList {
  CPA_ALLOCATOR
};

struct COL::stCollideSet {
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
  
  CPA_ALLOCATOR
};

struct COL::stCollideElementIndexedTriangles {
  /// Collide material
  pointer<COL::stCollideMaterial> collideMaterial;
  /// Indices into collide element vertex array
  pointer<uint16> faceIndices;
  /// List of normals
  pointer<MTH::stVector3D> faceNormals;
  /// Number of faces
  int16 numFaces;
  /// Index of AABB
  int16 aabbIndex;
  /// Visual set (often NULL, used by the editor for visualizing collide objects)
  pointer<GEO::stVisualElementIndexedTriangles> visualSet;
  /// Indices of triangle edges
  pointer<uint16> edgeIndices;
  /// Indices of edge normals (edges are separate collision entities from triangles)
  pointer<MTH::stVector3D> edgeNormals;
  /// Edge coefficients
  pointer<float32> edgeCoefficients;
  /// Number of edges
  int16 numEdges;
  /// Padding
  padding(2)
  
  CPA_ALLOCATOR
};

/// Indexed collide sphere
struct COL::stCollideElementIndexedSphere {
  /// Sphere radius
  float32 radius;
  /// Collide material
  pointer<GMT::stGameMaterial> collideMaterial;
  /// Index into collide element vertex array
  int16 indexOfCenterPoint;
  /// Padding
  padding(2)
  
  CPA_ALLOCATOR
};

/// A collide element of multiple indexed spheres
struct COL::stCollideElementIndexedSpheres {
  /// List of spheres
  pointer<stCollideElementIndexedSphere> spheres;
  /// Number of spheres
  int16 numSpheres;
  /// Collide object AABB index
  int16 aabbIndex;
  
  CPA_ALLOCATOR
};

struct COL::stCollideMaterial {
  int16 zoneType;
  /// COL_MaterialIdMask_
  uint16 identifier;
  float32 xDirection;
  float32 yDirection;
  float32 zDirection;
  float32 coefficient;
  uint16 aiType;
  padding(2)
  
  CPA_ALLOCATOR
};

/// Collision case, cast internally to MEC::stMechanicsObstacle.
struct COL::stCollisionCase {
  /// Time of collision (-1.0 to 1.0)
  float32 collisionTime;
  /// Normal of the collision
  MTH::stVector3D collisionNormal;
  /// World point of the collision
  MTH::stVector3D collisionPoint;
  /// Material of the dynamic object
  pointer<GMT::stGameMaterial> dynamicMaterial;
  /// Material of the static object
  pointer<GMT::stGameMaterial> staticMaterial;
  /// Parameter 1 (superobject)
  pointer<> param1;
  /// Parameter 2
  int32 param2;
  int16 dynamicEntityType;
  int16 staticEntityType;
  MTH::stVector3D translation;
  MTH::stVector3D movement;
  MTH::stVector3D endPosition;
  float32 sphereRadius;
  float32 slide1;
  float32 rebound1;
  float32 slide2;
  float32 rebound2;
  
  CPA_ALLOCATOR
};

struct COL::stIndexedAlignedBox {
  int16 minPoint;
  int16 maxPoint;
  pointer<GMT::stGameMaterial> material;
  
  CPA_ALLOCATOR
};

struct COL::stCollideElementAlignedBoxes {
  pointer<stIndexedAlignedBox> boxes;
  int16 numBoxes;
  int16 parallelBoxIndex;
  
  CPA_ALLOCATOR
};

/// Global structure used for all things collision-related
struct COL::stGVForCollision {
  pointer<MTH::stVector3D> vertex1;
  MTH::stVector3D edgeVector;
  pointer<MTH::stVector3D> vertex2;
  MTH::stVector3D dinST0Point;
  float32 dynamicRadius;
  pointer<MAT::stTransformation> staticGeometricObjMatrix;
  MTH::stVector3D dinST1Point;
  MTH::stVector3D dinST01Vector;
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
  pointer<MAT::stTransformation> dynamicGeometricObjectMatrixT0;
  pointer<MAT::stTransformation> dynamicGeometricObjectMatrixT1;
  MAT::stTransformation inverseMatrix;
  MAT::stTransformation transformMatrixD2ST0;
  MAT::stTransformation transformMatrixD2ST1;
  MAT::stTransformation transformMatrixS2DT0;
  MAT::stTransformation transformMatrixS2DT1;
  
  /// Scale factor to be used for the static object
  float32 staticScale;
  /// Indexed spheres for the dynamic object
  pointer<stCollideElementIndexedSpheres> dynamicElementSpheres;
  /// Indexed spheres for the static object
  pointer<stCollideElementIndexedSpheres> staticElementSpheres;
  
  int32 bitFieldOfIndexedSpheresInCollision;
  /// The current static sphere being collided
  pointer<stCollideElementIndexedSphere> staticIndexedSphere;
  MTH::stVector3D swapDinST0Point;
  /// Center point of the dynamic object
  pointer<MTH::stVector3D> dynamicCenter;
  /// Center point of the static object
  pointer<MTH::stVector3D> staticCenter;
  
  /// If `useEnlargedSphere` is true, this is the radius of the original
  /// size sphere, and if it is false it is the radius of the enlarged sphere.
  float32 swapRadius;
  /// Whether to use the enlarged sphere or not (scaled by staticScale)
  uint8 useEnlargedSphere;
  
  
  #define COL_MaxSelectedOctreeNodes  100
  /// Number of selected nodes of `octree`
  int16 numSelectedNodes;
  /// The selected nodes of `octree`
  pointer<stOctreeNode> selectedOctreeNodes[COL_MaxSelectedOctreeNodes];
  /// The selected nodes' of `octree` T values
  float32 selectedOctreeT[COL_MaxSelectedOctreeNodes];
  
  
  pointer<stCollideElementAlignedBoxes> dynamicElementAlignedBoxes;
  int16 dynamicIndexedAlignedBoxIndex;
  pointer<stIndexedAlignedBox> dynamicIndexedAlignedBox;
  pointer<MTH::stVector3D> dynamicMinPoint;
  pointer<MTH::stVector3D> dynamicMaxPoint;
  
  MTH::stVector3D dinST0MaxPoint;
  MTH::stVector3D dinST0MinPoint;
  MTH::stVector3D dinST1MaxPoint;
  MTH::stVector3D dinST1MinPoint;
  MTH::stVector3D dinST08VBox[8];
  MTH::stVector3D dinST18VBox[8];
  MTH::stVector3D dinST01Vect[8];
  
  /// Index of sphere in `staticElementSpheres`?
  int16 staticIndexedSphereIndex;
  
  pointer<stCollideElementAlignedBoxes> staticElementAlignedBoxes;
  pointer<stIndexedAlignedBox> staticIndexedBox;
  pointer<MTH::stVector3D> pStaticMinPoint;
  pointer<MTH::stVector3D> pStaticMaxPoint;
  MTH::stVector3D staticMinPoint;
  MTH::stVector3D staticMaxPoint;
  MTH::stVector3D static8VBox[8];
  
  CPA_ALLOCATOR
};

struct COL::stBoundingSphere {
  MTH::stVector4D center;
  float32 radius;
#if engine == R3 && platform == PS2
  padding(12)
#endif
  
  CPA_ALLOCATOR
};

struct COL::stParallelBox {
  MTH::stVector3D minPoint;
  MTH::stVector3D maxPoint;
  
  CPA_ALLOCATOR
};

#pragma mark - GEO

#define GEO_ElementType_IndexedTriangles    1
#define GEO_ElementType_Facemap             2
#define GEO_ElementType_Sprite              3
#define GEO_ElementType_TMesh               4
#define GEO_ElementType_Points              5
#define GEO_ElementType_Lines               6
#define GEO_ElementType_IndexedSpheres      7
#define GEO_ElementType_AABB                8
#define GEO_ElementType_Cones               9
#define GEO_ElementType_Altimap             11
#define GEO_ElementType_DeformationSetInfo  13

union GEO::uVisualObject {
  pointer<GEO::stGeometricObject> geometricObject;
  pointer<MOR::stMorphObject> morphObject;
  CPA_ALLOCATOR
};

struct GEO::stGeometricObject {
  pointer<MTH::stVector3D> vertices;
  pointer<MTH::stVector3D> vertexNormals;
#if engine == R3 && platform == GCN
  // Vertex blend weights for each multimaterial
  doublepointer<float32> blendWeights;
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
  MTH::stVector4D boundingSphereCenter;
  pointer<> edgesDI;
  int16 numEdgesDI;
  int16 numOctreeEdges;
  int32 usedForDrawingShadow;
  int32 usedForCreatingShadow;
  /// TODO: check if this is specific to ps2
  pointer<> sdcData;
  uint32 isStatic;
  uint32 displayList;
  uint8 vtForSinus;
  padding(3)
  
  CPA_ALLOCATOR
};

struct GEO::stVisualSet {
  float32 lastDistance;
  int16 numLodDefinitions;
  int16 type;
  pointer<float32> LODThresholdTable;
  pointer<uVisualObject> LODDefinitions;
  doublepointer<> hRLI;
  int32 numRLI;
  CPA_ALLOCATOR
};

struct GEO::stVisualElementIndexedTriangles {
  pointer<GLI::stMaterial> visualMaterial;
  int16 numFaces;
  int16 numUVs;
  int16 numUVMaps;
  padding(2) // lightmap index?
  pointer<uint16> faceIndices;
#if game == R3_GCN
  padding(4)
#endif
  pointer<uint16> faceUVIndices;
  pointer<MTH::stVector3D> faceNormals;
  pointer<MTH::stVector2D> UVElements;
  pointer<> edges;
  pointer<> adjacentFaces;
  pointer<uint16> vertexIndices;
  int16 numVertexIndices;
  int16 parallelBoxIndex;
  uint32 displayList;
#if engine == R3 && platform == PS2
  pointer<> specialValue;
#endif
  uint8 portalVisibility;
  
#if engine == R3
 #if platform != PS2
  padding(1)
  uint16 numMappingEntries;
  pointer<uint16> mappingVertices;
  pointer<uint16> mappingUVs;
  uint16 numTriangleStrip;
  uint16 numTriangleIsolate;
  pointer<uint16> triangleStripIndices;
  pointer<uint16> triangleIsolateIndices;
 #else
  padding(3)
 #endif
#endif
  
  
#if platform == PS2
  uint32 vao[4];
#else
  string<32> name;
#endif
  
  CPA_ALLOCATOR
};

struct GEO::stColor {
  float32 r = 0.0f;
  float32 g = 0.0f;
  float32 b = 0.0f;
  float32 a = 0.0f;
  
  inline operator MTH::stVector4D&() const {
    return *(MTH::stVector4D*)&r;
  }
  
  CPA_ALLOCATOR
};

struct GEO::stParallelBox {
  MTH::stVector3D min;
  MTH::stVector3D max;
  CPA_ALLOCATOR
};

#pragma mark - GMT

struct GMT::stCollideMaterial {
  int16 zoneType;
  uint16 identifier;
  MTH::stVector3D direction;
  float32 coefficient;
  uint16 aiType;
  padding(2)
  
  CPA_ALLOCATOR
};

struct GMT::stGameMaterial {
  int32 soundMaterial;
  pointer<stCollideMaterial> collideMaterial;
  
  CPA_ALLOCATOR
};

#pragma mark - PO

struct PO::stPhysicalObject {
  pointer<GEO::stVisualSet> visualSet;
  pointer<COL::stPhysicalCollideSet> physicalCollideSet;
  pointer<COL::stBoundingSphere> visualBoundingVolume;
  pointer<COL::stBoundingSphere> collideBoundingVolume;
  
  CPA_ALLOCATOR
};

#pragma mark - IPO

struct IPO::stInstantiatedPhysicalObject {
  pointer<PO::stPhysicalObject> physicalObject;
  /// Current vertex radiosity. When the level loads, this is transferred into VRAM and unloaded.
  pointer<ISI::stISI> currentRadiosity;
  doublepointer<ISI::stISI> radiosity;
  pointer<stSuperObject> portalCamera;
  uint32 lastTransitionID;
  float32 lastRatioUsed;
#if platform == GCN
  padding(4)
  string<0x32> name;
#endif
  
  CPA_ALLOCATOR
};


#pragma mark - ISI

struct ISI::stColor {
  uint16 r;
  uint16 g;
  uint16 b;
  uint16 a;
  
  inline operator MTH::stVector4D() const {
//    pointer<float32> hRLIScaleTable = pointer<float32>(0x8050d0c0);
//    float scale = hRLIScaleTable[int(255.0f * fVar3)];
    MTH::stVector4D result {};
    result.x() = std::clamp(float(r), 0.0f, 255.0f) / 255.0f;
    result.y() = std::clamp(float(g), 0.0f, 255.0f) / 255.0f;
    result.z() = std::clamp(float(b), 0.0f, 255.0f) / 255.0f;
    result.w() = std::clamp(float(a), 0.0f, 255.0f) / 255.0f;
    return result;
  }
  
  CPA_ALLOCATOR
};

struct ISI::stLOD {
  uint16 numVertexRLI;
  padding(2)
  pointer<ISI::stColor> vertexRLI;
  pointer<float32> luminosity;
  
  CPA_ALLOCATOR
};

struct ISI::stISI {
  uint16 numLOD;
  padding(2)
  pointer<ISI::stLOD> hLOD;
  
  CPA_ALLOCATOR
};


#pragma mark - stSuperObject

#define HIE_SuperObjectType_None                  (0 << 0)
#define HIE_SuperObjectType_World                 (1 << 0)
#define HIE_SuperObjectType_Actor                 (1 << 1)
#define HIE_SuperObjectType_Sector                (1 << 2)
#define HIE_SuperObjectType_PhysicalObject        (1 << 3)
#define HIE_SuperObjectType_PhysicalObjectMirror  (1 << 4)
#define HIE_SuperObjectType_IPO                   (1 << 5)
#define HIE_SuperObjectType_IPOMirror             (1 << 6)
#define HIE_SuperObjectType_SpecialEffect         (1 << 7)
#define HIE_SuperObjectType_NoAction              (1 << 8)
#define HIE_SuperObjectType_Mirror                (1 << 9)

struct stSuperObject {
  stSuperObject(const uint32 type);
  
  /// The type of the object
  uint32 type;
  
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
  pointer<MAT::stTransformation> localTransform;
  /// The transform local to the world
  pointer<MAT::stTransformation> globalTransform;
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
  MTH::stVector3D semiLookAt;
  /// Render transparency
  float32 transparency;
  /// Color of outline (when drawflags are set?)
  uint32 outlineColor;
  ///
  int32 displayPriority;
  /// ?
  int32 ilstatus;
  /// Ambient light default color
  MTH::stVector3D ambientColor;
  ///
  MTH::stVector3D parallelDirection;
  ///
  MTH::stVector3D parallelColor;
  /// Superimpose on the viewport
  uint8 superimpose;
  ///
  uint8 isSuperObject;
  ///
  uint8 transition;
  ///
  padding(1)
  
  /// The name of this superobject
  const std::string Name() const;
  /// The name of this superobject's type
  const std::string TypeName() const;
  /// The global transform
  MAT::stTransformation& Transform() const;
  
  /// Add a new child to this object
  bool AddChild(pointer<stSuperObject>&);
  /// Detach this object from the hierarchy
  void Detach();
  bool IsDetached();
  
  
  /// Find a superobject in this hierarchy
  pointer<stSuperObject>& Find(const std::string& name);
  
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
  
  CPA_ALLOCATOR
  
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

struct AI::stBrain {
  pointer<stMind> mind;
  pointer<GMT::stGameMaterial> lastNoCollideMaterial;
  uint8 warnMechanics;
  uint8 activeDuringTransition;
  padding(2)
  
  CPA_ALLOCATOR
};

struct AI::stMind {
  pointer<stAIModel> aiModel;
  pointer<stIntelligence> intelligence;
  pointer<stIntelligence> reflex;
  pointer<stDsgMem> dsgMem;
  pointer<> unknown;
  uint8 runIntelligence;
  padding(3)
  
  CPA_ALLOCATOR
};

struct AI::stAIModel {
  pointer<stScriptAI> intelligenceBehaviorList;
  pointer<stScriptAI> reflexBehaviorList;
  pointer<stDsgVar> dsgVar;
  pointer<stMacroList> macroList;
  uint8 secondPassFinished;
  padding(3)
  
  CPA_ALLOCATOR
};

struct AI::stNodeInterpret {
#if platform == GCN
  uint32 param;
  padding(3)
  uint8 type;
  padding(2)
  uint8 depth;
  padding(1)
#endif
  
  CPA_ALLOCATOR
};

struct AI::stTreeInterpret {
  pointer<stNodeInterpret> node;
  CPA_ALLOCATOR
};

union AI::uGetSetParam {
  int8 s8Value;
  int16 s16Value;
  int32 s32Value;
  float32 floatValue;
  pointer<> pointerValue;
  CPA_ALLOCATOR
};

struct AI::stActionParam {
  union uGetSetParam param[8];
  CPA_ALLOCATOR
};

struct AI::stActionTableEntry {
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
  CPA_ALLOCATOR
};

struct AI::stActionTable {
  pointer<stActionTableEntry> entries;
  uint8 numEntries;
  uint8 numEntriesUsed;
  uint8 currentEntry;
  padding(1)
  CPA_ALLOCATOR
};

struct AI::stBehavior {
  string<0x100> name; /* 256 on GCN, at least */
  pointer<stTreeInterpret> scripts;
  pointer<stTreeInterpret> firstScript;
  uint8 numScripts;
  padding(3)
  CPA_ALLOCATOR
};

struct AI::stMacro {
  string<0x100> name;
  pointer<stTreeInterpret> initialTree;
  pointer<stTreeInterpret> currentTree;
  CPA_ALLOCATOR
};

struct AI::stMacroList {
  pointer<stMacro> macros;
  uint8 numMacros;
  padding(3)
  CPA_ALLOCATOR
};

struct AI::stScriptAI {
  pointer<stBehavior> behavior;
  uint32 numBehaviors;
  uint32 noInitialization;
  uint8 numActionTableEntries;
  padding(3)
  CPA_ALLOCATOR
};

struct AI::stIntelligence {
  doublepointer<stScriptAI> scriptAI;
  pointer<stNodeInterpret> currentTree;
  pointer<stBehavior> currentBehavior;
  pointer<stBehavior> previousBehavior;
  pointer<> actionTable;
  uint32 initializeBehavior;
  CPA_ALLOCATOR
};

struct AI::stDsgVarInfo {
  uint32 memoryOffset;
  uint32 type;
  int16 saveType;
  padding(2)
  uint32 objectTreeInitialType;
  CPA_ALLOCATOR
};

struct AI::stDsgVar {
  pointer<> memory;
  pointer<stDsgVarInfo> info;
  uint32 memorySize;
  uint8 infoLength;
  padding(3)
  CPA_ALLOCATOR
};

struct AI::stDsgMem {
  doublepointer<stDsgVar> dsgVars;
  pointer<uint8> initialBuffer;
  pointer<uint8> currentBuffer;
  
  inline auto dsgVarInfo(int idx) -> pointer<stDsgVarInfo> { return (*dsgVars)->info + idx; }
  
  CPA_ALLOCATOR
};

#pragma mark - GLI

#define GLI_Flag_Textured                     (1 << 0)
#define GLI_Flag_GouraudShading               (1 << 1)
#define GLI_Flag_DisableDrawWireframe         (1 << 2)
#define GLI_Flag_DisableDrawGrid              (1 << 3)
#define GLI_Flag_DisableDrawDotted            (1 << 4)
#define GLI_Flag_DisableDrawOutline           (1 << 5)
#define GLI_Flag_DisableDrawCollideInfo       (1 << 6)
#define GLI_Flag_DisableDrawCollideInfoLight  (1 << 7)
#define GLI_Flag_DisableForceDefaultMaterial  (1 << 8)
#define GLI_Flag_DisableForceColorMaterial    (1 << 9)
#define GLI_Flag_BackfaceCulling              (1 << 10)
#define GLI_Flag_DisableDrawBoundingVolume    (1 << 11)
#define GLI_Flag_RLI                          (1 << 12)
#define GLI_Flag_DisableComputeSpecular       (1 << 13)
#define GLI_Flag_NoPriority                   (1 << 14)
#define GLI_Flag_UseStaticLights              (1 << 15)
#define GLI_Flag_UseShadow                    (1 << 16)
#define GLI_Flag_UnderwaterCamera             (1 << 17)
#define GLI_Flag_DisableForceDepthTest        (1 << 18)
#define GLI_Flag_DisableBackfaceInversion     (1 << 19)
#define GLI_Flag_DrawUnderwaterObjects        (1 << 20)
#define GLI_Flag_DrawNothing                  (1 << 21)
#define GLI_Flag_NotChromed                   (1 << 22)
#define GLI_Flag_NotVisibleInRealWorld        (1 << 23)
#define GLI_Flag_NotVisibleInSymmetricWorld   (1 << 24)
#define GLI_Flag_DisableDrawingInMirror       (1 << 25)
#define GLI_Flag_NotLightAlphaSensitive       (1 << 26)
#define GLI_Flag_DepthBufferWrite             (1 << 27)
#define GLI_Flag_HasNoMirror                  (1 << 28)
#define GLI_Flag_NoSinusEffectRLI             (1 << 29)
#define GLI_Flag_DepthTestEnable              (1 << 30)
#define GLI_Flag_NoSinusEffect                (1 << 31)


struct GLI::stVertex2D {
  float32 x;
  float32 y;
  float32 dz;
  CPA_ALLOCATOR
};

struct GLI::stCamera {
  int32 cameraMode;
  MAT::stTransformation transform;
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
  MTH::stVector3D left;
  float32 dLeft;
  MTH::stVector3D right;
  float32 dRight;
  MTH::stVector3D up;
  float32 dUp;
  MTH::stVector3D down;
  float32 dDown;
  float32 ratio;
  uint8 transparency;
  float32 transpDist;
  uint8 mirrored;
  CPA_ALLOCATOR
};

struct GLI::stZBufferForLight {
  int32 sizeX;
  int32 sizeY;
  float32 coefX;
  float32 coefY;
  pointer<> ZBufferMap;
  pointer<> middleZBufferMap;
  CPA_ALLOCATOR
};

#define GLI_LightType_Parallel      1
#define GLI_LightType_Spherical     2
#define GLI_LightType_Spot          3
#define GLI_LightType_Ambient       4
#define GLI_LightType_LimitParallel 5
#define GLI_LightType_Fog           6

struct GLI::stLight {
  int32 active;
  int32 isZBuffered;
  //int32 lightType;
  
  int16 lightType;
  padding(2);
  
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
  MAT::stTransformation transform;
  stZBufferForLight zbuffer;
  GEO::stColor color;
  float32 sqNear;
  float32 sqFar;
  float32 sqDiv;
  
  MTH::stVector3D boxV0;
  MTH::stVector3D boxV1;
  MTH::stVector3D boxV2;
  MTH::stVector3D boxV3;
  MTH::stVector3D boxCenter;
  
  float32 radius;
  float32 intensity; // min/max?
  
  MTH::stVector4D backgroundColor;
  
  CPA_ALLOCATOR
};

struct GLI::stTexture {
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
  padding(2)
  CPA_ALLOCATOR
};

struct GLI::stAnimatedTextureNode {
  CPA_ALLOCATOR
};

struct MultiMaterialTransform {
  float32 matrix[2][2];
  float32 translation[2];
};

struct GLI::stMultiTextureMaterial {
  pointer<stTexture> texture;
  uint8 textureOp;
  uint8 colorOp;
  uint8 uvSource;
  uint8 flags;
  uint32 textureProperties;
  struct MultiMaterialTransform staticPosition;
  struct MultiMaterialTransform dynamicPosition;
  float32 dynamicAngle;
  float32 stticAngle;
  uint32 materialFunction;
  
  CPA_ALLOCATOR
};

struct GLI::stMaterial {
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
  CPA_ALLOCATOR
};

#pragma mark - WP

struct WP::stWayPoint {
  MTH::stVector3D point;
  float32 radius;
  pointer<stSuperObject> superobject;
  CPA_ALLOCATOR
};

struct WP::stGraphNode {
  pointer<stGraphNode> next;
  pointer<stGraphNode> prev;
  pointer<stGraph> graph;
  pointer<stWayPoint> waypoint;
  int32 waypointType;
  int32 waypointTypeInitial;
  pointer<> arcList;
  CPA_ALLOCATOR
};

struct WP::stGraph {
  stDoublyLinkedList<stGraphNode> nodes;
  CPA_ALLOCATOR
};

struct WP::stGraphChainList {
  pointer<stGraph> graph;
  pointer<stGraphChainList> next;
  CPA_ALLOCATOR
};


#pragma mark - MS

struct MS::stMSWay {
  pointer<WP::stGraph> graph;
  int32 index;
  uint8 spherical;
  padding(3)
  
  CPA_ALLOCATOR
};

struct MS::stMSSound {
  CPA_ALLOCATOR
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

#define SND_EventType_Play  0
#define SND_EventType_Stop  1

//enum SND::StorageType {
//  ExternalFile = 0,
//  MegaFile = 1,
//  BigFile = 2,
//};

struct SND::stLinkTableEntry {
  uint32 id;
  uint64 cuuid;
};

/// An element chosen randomly
struct SND::stRandomElement {
  /// Link to the resource
  SND::Ref resourceLink;
  /// The probability of this element being chosen
  float32 probability;
};

struct SND::stSwitchElement {
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

struct SND::stDeTune {
  float32 panning;
};

struct SND::stEventParametersExtraAll {
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

//struct SND::stBlockEvent {
//  /// Event identifier
//  pointer<> unknown;
//  ///
//  uint64 cuuid;
//  /// Event type
//  uint32 type;
//  /// Unknown
//  uint32 unknown2;
//  /// Parameters
//  stEventParameters param;
//  /// Event resource
//  //pointer<>
//};


struct SND::stBlockEvent {
  /// Event identifier (classname)
  uint32 id;
  /// Event type
  uint32 type;
  /// Parameters
  stEventParameters param;
};


struct SND::stTypeInfo {
  /// Name of this type
  pointer<string<>> name;
};

//union SND::stResource {
//  
//};
//
//struct SND::stBlockResource {
//  /// The unique identifier of this resource
//  CUUID cuuid;
//  /// Resource type
//  uint32 type;
//  /// Unknown
//  uint32 unknown1;
//  /// Unknown
//  ///
//};

struct SND::stBlockEntry {
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

#pragma pack(pop)

#undef padding
#undef concat_inner
#undef unique_name
#undef concat
#undef s

/***********************/
/** ``STRUCTURE END`` **/
/***********************/


#pragma mark - Memory stream

// TODO: Move this to top

namespace Memory {
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


#pragma mark - Pointer constants -

namespace Global {
extern pointer<stAlways> g_stAlways;
extern pointer<stEngineStructure> g_stEngineStructure;
extern pointer<stObjectType> g_stObjectTypes;
extern pointer<LinkedList<stFamilyList>> g_stFamilyList;
extern pointer<IPT::stInputStructure> g_stInputStructure;
extern pointer<RND::stRandom> g_stRandomStructure;
extern pointer<stSuperObject> p_stActualWorld;
extern pointer<stSuperObject> p_stDynamicWorld;
extern pointer<stSuperObject> p_stInactiveDynamicWorld;
extern pointer<stSuperObject> p_stTransitDynamicWorld;
extern pointer<stSuperObject> p_stFatherSector;
extern pointer<uint8> g_bGhostMode;

// TODO: Derive globals from levels
extern pointer<> g_pFixMemory;
extern pointer<> g_pLevelMemory;
extern pointer<> g_pTransitMemory;

extern uint32_t g_lFixMemorySize;
extern uint32_t g_lLevelMemorySize;
extern uint32_t g_lTransitMemorySize;
}

#pragma mark -* Constants *-

namespace Constants {
/// The world's up vector
const static MTH::stVector3D WorldUp = { 0.0f, 0.0f, 1.0f };
}

#pragma mark - Functions -



namespace Global {
/// Find a sector by position
pointer<stSuperObject> SectorAtPosition(MTH::stVector3D point);
/// Test if an address lies in transit
inline bool IsInTransit(const void* addr) { return (uint8_t*)addr >= (uint8_t*)CPA::Global::g_pTransitMemory && (uint8_t*)addr < (uint8_t*)CPA::Global::g_pTransitMemory + CPA::Global::g_lTransitMemorySize; }
};

namespace Memory {
/// Load
bool LoadFromBuffer(Memory::HostAddressType mem, Memory::SizeType size);
///
}


} /* CPA */

#undef GCN
#undef PS1
#undef PS2
#undef PS3
#undef XBOX
#undef XBOX360
#undef PC
#undef MACOS
#undef DC
#undef NDS
#undef N3DS
#undef N64

#endif /* CPATOOLS_HH */
