#pragma once

#include <map>
#include <cmath>
#include <string>

// Do not remove this include
#include <cpatools/configuration.hpp>
#include <cpatools/serialize.hpp>

namespace cpa::structure {

struct stEngineStructure;
struct stEngineTimer;
struct stCineManager;
struct stLanguageStructure;
struct stRandom;
struct stAlways;
struct stObjectType;

// IPT
struct stInputStructure;
struct stInputEntryElement;
struct stPadReadingOutput;

// Object
struct stSuperObject;
struct stSector;
struct stEngineObject;
struct st3DData;
struct stStandardGameInfo;
struct stCollideSet;
struct stCineInfo;
struct stMSWay;
struct stMSLight;
struct stSectorInfo;
struct stMicro;
struct stMSSound;
struct stInstantiatedPhysicalObject;
struct stPhysicalObject;

// AI
struct stBrain;
struct stMind;
struct stAIModel;
struct stIntelligence;
struct stDsgMem;
struct stDsgVar;
struct stDsgVarInfo;
struct stScriptAI;
struct stMacro;
struct stMacroList;
struct stBehavior;

// CINE
struct stCine;
struct stCineActor;

// DNM
struct stDynam;
struct stDynamics;
struct stDynamicsBaseBlock;
struct stDynamicsAdvancedBlock;
struct stDynamicsComplexBlock;
struct stDynamicsReport;
struct stDynamicsRotation;
struct stDynamicsMovevement;
struct stDynamicsParsingData;

// COL
struct stPhysicalCollideSet;
struct stCollideElementIndexedTriangles;
struct stCollideElementIndexedTrianglesVisual;
struct stCollideMaterial; // defined in GMT under previous versions

// GMT, GLI, GLD
struct stGameMaterial;
struct stVertex2DGLI;
struct stCameraGLI;
struct stTexture;
struct stAnimatedTextureNode;

// WP
struct stWayPoint;
struct stGraph;
struct stGraphNode;
struct stGraphChainList;

#pragma mark - Structure -
  
using Index3D = uint16;

#define concat(a, b) concat_inner(a, b)
#define concat_inner(a, b) a ## b
#define unique_name(base) concat(base, __LINE__)
#define padding(S) private: uint8_t unique_name(padding) [S]; public:

/*************************/
/** ``STRUCTURE BEGIN`` **/
/*************************/

#pragma mark - Common types
  
template<unsigned N, typename T = float32>
struct vector {
  template<typename... Args, std::enable_if_t<sizeof...(Args) == N && std::conjunction_v<std::is_convertible<Args, float>...>>* = nullptr>
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
  
  inline auto x() -> T& { return data[0]; }
  inline auto y() -> T& { return data[1]; }
  inline auto z() -> T& { return data[2]; }
  inline auto w() -> T& { return data[3]; }
  inline auto xy() -> vector<2> { return vector<2>(x(), y()); }
  inline auto xyz() -> vector<3> { return vector<3>(x(), y(), z()); }
  inline auto& operator[](auto i) { return data[i]; }
  
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
  
  auto serialize(serializer::node& s) {
    s.type("stVector" + std::to_string(N) + "D");
    for (auto i : range(N)) {
      s.real(std::string(1, "xyzw"[i]), data[i]);
    }
  }
  
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
  
  static auto make_translation(stVector3D p) {
    matrix result = identity();
    for (auto i : range(3)) result(Rows-1,i) = p[i];
    return result;
  }
  
  static auto make_scale(stVector3D p){
    matrix result = identity();
    for (auto i : range(3)) result(i,i) = p[i];
    return result;
  }
  
  static auto make_perspective(float fovY, float aspect, float near, float far) {
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
  
  static auto make_lookat(stVector3D eye, stVector3D center, stVector3D up) {
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
  
  auto serialize(serializer::node& s) {
    s.type("stMatrix" + std::to_string(Rows) + "D");
    for (auto i : range(Rows * Columns))
      s.real("m" + std::to_string(i), m[i]);
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
//    pointer<T> c = first;
//    for (int i = 0; i < numEntries; i++) {
//      f(c++, userdata);
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

#pragma mark - stTransform

#define transformTypeUninitialized          0
#define transformTypeIdentity               1
#define transformTypeTranslate              2
#define transformTypeZoom                   3
#define transformTypeScale                  4
#define transformTypeRotation               5
#define transformTypeRotationZoom           6
#define transformTypeRotationScale          7
#define transformTypeRotationScaleComplex   8
#define transformTypeUndefined              9

/// World transform
struct stTransform {
  stTransform() = default;
  stTransform(uint32 _type, stMatrix4D T = stMatrix4D(), stVector4D _scale = stVector4D(1.0f, 1.0f, 1.0f, 1.0f)) : type(_type), matrix(T), scale(_scale) { /* ... */ }
  
  /// Type of the transform
  uint32 type = transformTypeIdentity;
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
    if (static_cast<uint32_t>(type) == transformTypeRotation) {
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
    if (type == transformTypeRotation) {
      return *this * v;
    } else if (type < transformTypeRotation) {
      return v;
    } else {
      // TODO
      return *this * v;
    }
  }
  
  inline auto typeName() -> std::string {
    switch (static_cast<uint32_t>(type)) {
      case transformTypeUninitialized:
        return "Uninitialized";
      case transformTypeIdentity:
        return "Identity";
      case transformTypeTranslate:
        return "Translate";
      case transformTypeZoom:
        return "Zoom";
      case transformTypeScale:
        return "Scale";
      case transformTypeRotation:
        return "Rotation";
      case transformTypeRotationZoom:
        return "RotationZoom";
      case transformTypeRotationScale:
        return "RotationScale";
      case transformTypeRotationScaleComplex:
        return "RotationScaleComplex";
      case transformTypeUndefined:
        return "Undefined";
      default:
        return "Invalid";
    }
  }
  
  auto serialize(serializer::node& s) {
    s.integer("type", type);
    s.structure("matrix", matrix);
    s.structure("scale", scale);
  }
};
  
#pragma mark - stParallelBox

struct stParallelBox {
  stVector3D min;
  stVector3D max;
  
  auto serialize(serializer::node& s) {
    s.type("stParallelBox");
    s.structure("min", min);
    s.structure("max", max);
  }
};

#pragma mark - stAlways
  
struct stAlwaysModelList {
  pointer<stAlwaysModelList> next;
  pointer<stAlwaysModelList> prev;
  pointer<stLinkedList<stAlwaysModelList>> parentList;
  int32 objectModelType;
  pointer<stEngineObject> alwaysObject;
};

struct stAlways {
  uint32 numAlways;
  stDoublyLinkedList<stAlwaysModelList> alwaysModels;
  pointer<stSuperObject> alwaysSuperobject;
  pointer<stEngineObject> alwaysActors;
  pointer<stSuperObject> alwaysGeneratorSuperobjects;
};
  
#pragma mark - stObjectType

/// Family object type
#define objectTypeFamily    0
/// Model object type
#define objectTypeModel     1
/// Instance object type
#define objectTypeInstance  2

/// Function to resolve object type names
using ObjectNameResolver = std::function<std::string(int, int)>;
  
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
};

/// Global object type table
struct stObjectType {
  /// Family object types
  stDoublyLinkedList<stObjectTypeElement> family;
  /// Model object types
  stDoublyLinkedList<stObjectTypeElement> model;
  /// Instance object types
  stDoublyLinkedList<stObjectTypeElement> instance;
};
  
#pragma mark - Engine
  
/// High-resolution counter
struct stTimerCount {
  uint32 low, high;
  
  auto serialize(serializer::node& s) {
    s.type("stTimerCount");
    s.integer("low", low);
    s.integer("high", high);
  }
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
  uint32 ticksPerMs;
  
  auto serialize(serializer::node& s) {
    s.type("stEngineTimer");
    s.integer("currentFrame", currentFrame);
    s.integer("timerHandle", timerHandle);
    s.integer("currentCount", currentCount);
    s.integer("deltaCount", deltaCount);
    //s.array("counter", counter);
    s.integer("usefulDeltaTime", usefulDeltaTime);
    s.integer("pauseTime", pauseTime);
    s.real("frameLength", frameLength);
    s.structure("totalRealTime", totalRealTime);
    s.structure("totalPauseTime", totalPauseTime);
    s.integer("ticksPerMs", ticksPerMs);
  }
};

/* engine mode */
#define engineModeInvalid               0
#define engineModeInitialize            1
#define engineModeDeinitialize          2
#define engineModeInitializeGameplay    3
#define engineModeDeinitializeGameplay  4
#define engineModeEnterLevel            5
#define engineModeChangeLevel           6
#define engineModeGameplay              9

/* input mode */
#define engineInputModeNormal     0
#define engineInputModeCommands   1

/// Engine structure
struct stEngineStructure {
  /// Engine mode (`engineMode#`)
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
  pointer<stCameraGLI> viewportCamera[5];
  int16 gldFixViewport[5];
  padding(2)
  padding(5 * 28 * 4) /* fix viewport attributes */
  padding(5 * 2 * 4) /* fix 3d attributes */
  pointer<stCameraGLI> fixCamera[5];
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
  pointer<stGraphChainList> graphList;
  pointer<stCineManager> cineManager;
  
  /// Load level by name
  inline auto loadLevel(std::string levelName) -> void;
  
  auto serialize(serializer& s);
};
  
#pragma mark - IPT

/// Structure for ReadAnalogJoystick function
struct stPadReadingOutput {
  /// The world vector the joystick value translates to
  stVector3D globalVector;
  int16 horizontalAxis;
  int16 verticalAxis;
  float32 analogForce;
  float32 trueAnalogForce;
  float32 rotationAngle;
  /// Strafe sector (0-7 clockwise)
  int32 strafeSector;
  
  auto serialize(serializer::node& s) {
    s.type("stPadReadingOutput");
    s.structure("globalVector", globalVector);
    s.integer("horizontalAxis", horizontalAxis);
    s.integer("verticalAxis", verticalAxis);
    s.real("analogForce", analogForce);
    s.real("trueAnalogForce", trueAnalogForce);
    s.real("rotationAngle", rotationAngle);
    s.integer("strafeSector", strafeSector);
  }
};

struct stInputDevice {
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
  
  auto serialize(serializer& s);
};

struct stInputEntryElement {
  padding(6 * 4) /* ? */
  uint32 numKeywords;
  pointer<> keywordArray;
  pointer<string<>> actionName;
  pointer<string<>> entryName;
  int32 state;
  float32 analogValue;
  int8 active;
  padding(3)
  
  auto serialize(serializer& s);
};

struct stInputStructure {
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
  
#pragma mark - RND
  
/// Size of tableIndices
#define RNDTableIndexCount 0x0032
/// Size of the table
#define RNDTableCount 0x2710
/// Default index into tableIndices
#define RNDDefaultIndex 0x0000

/// Random number table structure
struct stRandom {
  /// Index the random number table by absolute offset
  int32_t index(unsigned i) {
    uint32_t* T = table;
    return T ? ((T[i % RNDTableCount] >> 16) & 0x7FFF) : 0;
  }
  /// Index the random number table using an index from tableIndices, optionally offset
  int32_t indexRelative(unsigned TableIndicesIdx, unsigned Offset) {
    return index(uint32_t(tableIndices[TableIndicesIdx]) + Offset);
  }
  /// Simulate `Count` calls into the RND table, bounding the value by `Min` and `Max`
  int32_t call(unsigned const Count, unsigned const Min, unsigned const Max, unsigned const Index = RNDDefaultIndex) {
    int32_t n, v = 0;
    for (n = 0; n < Count; n++)
      v = (Min + ((Max + 1 - Min) * indexRelative(Index, n)) / (tableMax + 1));
    return v;
  }
  
  /// Size of the table
  uint32 tableSize;
  /// Indices into the table
  uint32 tableIndices[RNDTableIndexCount];
  /// Last index from tableIndices
  uint32 lastIndex;
  /// Largest number present in table
  uint32 tableMax;
  /// 1.0 / tableMax
  float32 tableMaxInverse;
  /// Random number table
  pointer<uint32> table;
  
  auto serialize(serializer::node& s) {
    s.type("stRandom");
    s.integer("tableSize", tableSize);
    //s.array("tableIndices", tableIndices);
    s.integer("lastIndex", lastIndex);
    s.integer("tableMax", tableMax);
    s.real("tableMaxInverse", tableMaxInverse);
    s.array("table", table, tableSize);
  }
};
  
#pragma mark - 3D
  
struct stAnim3D {
  
};

struct stSubAnim {
  pointer<stAnim3D> subAnim;
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
};
  
#pragma mark - CINE
  
struct stCineActor {
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
  pointer<stBehavior> intelligenceStart;
  uint8 changeReflexAtStart;
  padding(3)
  pointer<stBehavior> reflexStart;
  uint8 changeIntelligenceAtEnd;
  padding(3)
  pointer<stBehavior> intelligenceEnd;
  uint8 changeReflexAtEnd;
  padding(3)
  pointer<stBehavior> reflexEnd;
  uint8 startUseSoundRequest;
  uint8 startUseVoiceRequest;
  uint8 startUseMusicRequest;
  uint8 startUseAmbianceRequest;
  pointer</*stSoundBlockEvent*/> startSoundRequest;
  pointer</*stSoundBlockEvent*/> startVoiceRequest;
  pointer</*stSoundBlockEvent*/> startMusicRequest;
  pointer</*stSoundBlockEvent*/> startAmbianceRequest;
  uint8 endUseSoundRequest;
  uint8 endUseVoiceRequest;
  uint8 endUseMusicRequest;
  uint8 endUseAmbianceRequest;
  pointer</*stSoundBlockEvent*/> endSoundRequest;
  pointer</*stSoundBlockEvent*/> endVoiceRequest;
  pointer</*stSoundBlockEvent*/> endMusicRequest;
  pointer</*stSoundBlockEvent*/> endAmbianceRequest;
  pointer<stCine> cinematic;
  stDoublyLinkedList<> channelLink;
  pointer<stCineActor> next;
  pointer<stCineActor> prev;
  pointer<stDoublyLinkedList<stCineActor>> parents;
};

struct stCine {
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
  
struct stCineManager {
  /// List of level cinematics
  stDoublyLinkedList<stCine> cineList;
  /// Padding
#if CPA_PLATFORM == CPA_PLATFORM_PS2
  padding(4)
#endif
  /// Force camera transform
  stTransform fixedCameraTransform;
  /// Padding
#if CPA_PLATFORM == CPA_PLATFORM_PS2
  padding(2)
#endif
  /// Currently active cutscene camera
  pointer<stSuperObject> activeCamera;
};

#pragma mark - DNM

/// Axis-angle
struct stDynamicsRotation {
  float32 angle;
  stVector3D axis;
  
  auto serialize(serializer::node& s) {
    s.type("stDynamicsRotation");
    s.real("angle", angle);
    s.structure("axis", axis);
  }
};

/// Dynamics base block
struct stDynamicsBaseBlock {
  /// Type of the object
  int32 objectType;
  /// Current mechanics ID card
  pointer<> idcard;
  /// Mechanics control flags
  uint32 flags;
  /// Mechanics verification flags
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
  
#if CPA_ENGINE_VERSION == CPA_ENGINE_VERSION_R3 && CPA_PLATFORM == CPA_PLATFORM_PS2
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
  
#if CPA_ENGINE_VERSION == CPA_ENGINE_VERSION_R3 && CPA_PLATFORM == CPA_PLATFORM_PS2
  /// Padding
  padding(8)
#endif
  
  auto serialize(serializer::node& s) {
    s.type("stDynamicsBaseBlock");
    s.integer("objectType", objectType);
    s.pointer("idcard", idcard);
    s.integer("flags", flags);
    s.integer("endFlags", endFlags);
    s.real("gravity", gravity);
    s.real("slopeLimit", slopeLimit);
    s.real("slopeCosine", slopeCosine);
    s.real("slide", slide);
    s.real("rebound", rebound);
    s.structure("imposeSpeed", imposeSpeed);
    s.structure("proposeSpeed", proposeSpeed);
    s.structure("previousSpeed", previousSpeed);
    s.structure("scale", scale);
    s.structure("animationProposeSpeed", animationProposeSpeed);
    s.structure("safeTranslation", safeTranslation);
    s.structure("addTranslation", addTranslation);
    s.structure("previousTransform", previousTransform);
    s.structure("currentTransform", currentTransform);
    s.structure("imposedRotation", imposedRotation);
    s.integer("numFrames", numFrames);
    s.pointer("report", report);
  }
};
  
/// Dynamics advanced block
struct stDynamicsAdvancedBlock {
  /// Inertia (originally component-separated)
  stVector3D inertia;
  /// Priority of stream
  float32 streamPriority;
  /// Stream effect factor
  float32 streamFactor;
  /// Slide factor (originally component-separated)
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
  /// Ground normal
  stVector3D groundNormal;
  /// Wall normal
  stVector3D wallNormal;
  /// Number of calls made to mechanics without colliding with anything
  int8 collideCount;
  /// Padding
  padding(3)
  
  auto serialize(serializer::node& s) {
    s.type("stDynamicsAdvancedBlock");
    s.structure("inertia", inertia);
    s.real("streamPriority", streamPriority);
    s.real("streamFactor", streamFactor);
    s.structure("slideFactor", slideFactor);
    s.real("previousSlide", previousSlide);
    s.structure("maxSpeed", maxSpeed);
    s.structure("streamSpeed", streamSpeed);
    s.structure("addSpeed", addSpeed);
    s.structure("limit", limit);
    s.structure("collisionTranslation", collisionTranslation);
    s.structure("inertiaTranslation", inertiaTranslation);
    s.structure("groundNormal", groundNormal);
    s.structure("wallNormal", wallNormal);
    s.integer("collideCount", collideCount);
  }
};

/// AI and DNM message-interchange - "Module Allowing the Communication of Datas from the Player or the Intelligence to the Dynamics"
struct stMACDPID {
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
  
  auto serialize(serializer::node& s) {
    s.type("stMACDPID");
    s.real("data0", data0);
    s.structure("data1", data1);
    s.structure("data2", data2);
    s.structure("data3", data3);
    s.real("data4", data4);
    s.real("data5", data5);
    s.real("data6", data6);
    s.structure("data7", data7);
    s.structure("data8", data8);
    s.integer("data9", data9);
    s.integer("data10", data10);
    s.structure("data11", data11);
    s.real("data12", data12);
    s.structure("data13", data13);
    s.real("data14", data14);
    s.integer("data15", data15);
  }
};

/// Dynamics complex block
struct stDynamicsComplexBlock {
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
  
  auto serialize(serializer::node& s) {
    s.type("stDynamicsComplexBlock");
    s.real("tiltStrength", tiltStrength);
    s.real("tiltInertia", tiltInertia);
    s.real("tiltOrigin", tiltOrigin);
    s.real("tiltAngle", tiltAngle);
    s.real("hangingLimit", hangingLimit);
    s.structure("contact", contact);
    s.structure("fallTranslation", fallTranslation);
    s.structure("macdpid", macdpid);
    s.pointer("platformSuperObject", platformSuperObject);
    s.structure("previousMatrixAbsolute", previousMatrixAbsolute);
    s.structure("previousMatrixPrevious", previousMatrixPrevious);
  }
};

/// Dynamics obstacle reported from mechanics
struct stDynamicsObstacle {
  /// Collision rate
  float32 rate;
  /// Contact normal
  stVector3D normal;
  /// World contact point
  stVector3D contact;
  /// Material for entity 1 (self)
  pointer<stGameMaterial> myMaterial;
  /// Material for entity 2 (object collided with)
  pointer<stGameMaterial> collidedMaterial;
  /// Collided object
  pointer<stSuperObject> superObject;
  
  auto serialize(serializer::node& s) {
    s.type("stDynamicsObstacle");
    s.real("rate", rate);
    s.structure("normal", normal);
    s.structure("contact", contact);
    //s.pointer("myMaterial", myMaterial);
    //s.pointer("collidedMaterial", collidedMaterial);
    s.pointer("superObject", superObject);
  }
};

// Dynamics obstacle type
#define dynamicsObstacleTypeNothing     0
#define dynamicsObstacleTypeScenery     1
#define dynamicsObstacleTypeMobile      2
#define dynamicsObstacleTypeDoubleEdge  4
#define dynamicsObstacleTypeMobileWall  9

/// Mechanics engine obstacle (used internally)
/// Cast to stCollisionCase
struct stDynamicsObstacleMEC {
  /// Collision rate
  float32 rate;
  /// Contact normal
  stVector3D normal;
  /// World contact poimt
  stVector3D contact;
  /// Material for entity 1 (self)
  pointer<stGameMaterial> myMaterial;
  /// Material for entity 2 (object collided with)
  pointer<stGameMaterial> collidedMaterial;
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
  
  auto serialize(serializer::node& s) {
    s.type("stDynamicsObstacleMEC");
    s.real("rate", rate);
    s.structure("normal", normal);
    s.structure("contact", contact);
    //s.pointer("myMaterial", myMaterial);
    //s.pointer("collidedMaterial", collidedMaterial);
    s.pointer("superObject", superObject);
    s.integer("type", type);
    s.integer("myEntity", myEntity);
    s.integer("collidedEntity", collidedEntity);
    s.structure("translation", translation);
    s.structure("zoneMove", zoneMove);
    s.structure("zonePosition", zonePosition);
    s.real("zoneRadius", zoneRadius);
  }
};

/// Movement
struct stDynamicsMovevement {
  /// Linear movement
  stVector3D linear;
  /// Angular movement
  stDynamicsRotation angular;
  
  auto serialize(serializer::node& s) {
    s.type("stDynamicsMovement");
    s.structure("linear", linear);
    s.structure("angular", angular);
  }
};

/// Dynamics collision report
struct stDynamicsReport {
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
  stDynamicsMovevement previousAbsoluteSpeed;
  /// Current absolute speed
  stDynamicsMovevement currentAbsoluteSpeed;
  /// Previous absolute position
  stDynamicsMovevement previousAbsolutePosition;
  /// Current absolute position
  stDynamicsMovevement currentAbsolutePosition;
  /// Extra flags
  char8 bitField;
  /// Padding
  padding(3)
  
  auto serialize(serializer::node& s) {
    s.type("stDynamicsReport");
    s.integer("previousSurfaceState", previousSurfaceState);
    s.integer("currentSurfaceState", currentSurfaceState);
    s.structure("obstacle", obstacle);
    s.structure("ground", ground);
    s.structure("wall", wall);
    s.structure("character", character);
    s.structure("ceiling", ceiling);
    s.structure("previousAbsoluteSpeed", previousAbsoluteSpeed);
    s.structure("currentAbsoluteSpeed", currentAbsoluteSpeed);
    s.structure("previousAbsolutePosition", previousAbsolutePosition);
    s.structure("currentAbsolutePosition", currentAbsolutePosition);
    s.integer("bitField", bitField);
  }
};

struct stDynamicsReportMEC {
  /// The current surface state
  uint32_t currentSurfaceState;
  /// Generic obstacle
  stDynamicsObstacleMEC obstacle;
  /// Ground obstacle
  stDynamicsObstacleMEC ground;
  /// Wall obstacle
  stDynamicsObstacleMEC wall;
  /// Actor obstacle
  stDynamicsObstacleMEC character;
  /// Water obstacle
  stDynamicsObstacleMEC water;
  /// Ceiling obstacle
  stDynamicsObstacleMEC ceiling;
  
  auto serialize(serializer::node& s) {
    s.type("stDynamicsReport");
    s.integer("currentSurfaceState", currentSurfaceState);
    s.structure("obstacle", obstacle);
    s.structure("ground", ground);
    s.structure("wall", wall);
    s.structure("character", character);
    s.structure("water", water);
    s.structure("ceiling", ceiling);
  }
};

/// Parameters for mechanics engine
struct stDynamics {
  /// Base block
  stDynamicsBaseBlock base;
  /// Advanced block
  stDynamicsAdvancedBlock advanced;
  /// Complex block
  stDynamicsComplexBlock complex;
  
  auto flag(int flag) -> bool {
    return base.flags & flag;
  }
  
  auto endFlag(int flag) -> bool {
    return base.endFlags & flag;
  }
  
  auto serialize(serializer::node& s) {
    s.type("stDynamics");
    s.structure("base", base);
    s.structure("advanced", advanced);
    s.structure("complex", complex);
  }
};

struct stDynamicsParsingData {
  stVector3D position;
  float32 outAlpha;
  stVector3D vector;
  
  auto serialize(serializer::node& s) {
    s.type("stDynamicsParsingData");
    s.structure("position", position);
    s.real("outAlpha", outAlpha);
    s.structure("vector", vector);
  }
};

struct stDynam {
  pointer<stDynamics> dynamics;
  pointer<stDynamicsParsingData> parsingDatas;
  uint32 usedMechanics;
  
  auto serialize(serializer::node& s) {
    s.type("stDynam");
    s.pointer("dynamics", dynamics);
    s.pointer("parsingDatas", parsingDatas);
    s.integer("usedMechanics", usedMechanics);
  }
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
  
  auto serialize(serializer::node& s) {
    s.type("stStandardGameInfo");
    s.integer("familyType", familyType);
    s.integer("modelType", modelType);
    s.integer("instanceType", instanceType);
    s.pointer("superObject", superObject);
    s.integer("initialFlag", initialFlag);
    s.integer("flag1", flag1);
    s.integer("flag2", flag2);
    s.integer("lastFrame", lastFrame);
    s.integer("capabilities", capabilities);
    s.integer("tractionFactor", tractionFactor);
    s.integer("hitPoints", hitPoints);
    s.integer("maxHitPoints", maxHitPoints);
    s.integer("maxHitPointsMax", maxHitPointsMax);
    s.integer("customBits", customBits);
    s.integer("aiCustomBits", aiCustomBits);
    s.integer("platformType", platformType);
    s.integer("miscFlags", miscFlags);
    s.integer("transparencyZoneMin", transparencyZoneMin);
    s.integer("transparencyZoneMax", transparencyZoneMax);
    s.integer("initialCustomBits", initialCustomBits);
    s.integer("aiInitialCustomBits", aiInitialCustomBits);
    s.integer("initialHitPoints", initialHitPoints);
    s.integer("maxInitialHitPoints", maxInitialHitPoints);
    s.integer("initialMiscFlags", initialMiscFlags);
    s.integer("tooFarLimit", tooFarLimit);
    s.integer("importance", importance);
    s.integer("optional", optional);
  }
};

/// Engine object - an actor in the dynamic world
struct stEngineObject {
  /// 3D-related parameters
  pointer<st3DData> data3D;
  /// Standard game info
  pointer<stStandardGameInfo> stdGame;
  /// Dynamics
  pointer<stDynam> dynam;
  /// Brain and AI
  pointer<stBrain> brain;
  /// Cinematic-related info of this actor
  pointer<stCineInfo> cineInfo;
  /// Collision geometry set
  pointer<stCollideSet> collSet;
  /// Waypoint microstructure
  pointer<stMSWay> msWay;
  /// Light microstructure
  pointer<stMSLight> msLight;
  /// Sector info
  pointer<stSectorInfo> sectorInfo;
  /// ?
  pointer<stMicro> micro;
  /// Sound microstructure
  pointer<stMSSound> msSound;
  
  /// Get the name of this actor in order of [Instance, Model, Family]
  inline auto name(int16_t type = objectTypeInstance) -> std::string;
  /// Get the superobject associated with this actor
  inline auto superobject() -> pointer<stSuperObject>;
  /// Return the AI model of this actor
  inline auto aiModel() -> pointer<stAIModel>;
  /// Get the dsg variable memory
  inline auto dsgMem() -> pointer<stDsgMem>;
  /// Get the dsg variable memory at specified index
  inline auto dsgVar(int idx, uint32_t* type = nullptr) -> pointer<>;
  /// Get the speed of this actor
  inline auto speed() -> stVector3D;
  /// Get the linear horizontal speed of this actor
  inline auto horizontalSpeed() -> float;
  /// Get the linear vertical speed of this actor
  inline auto verticalSpeed() -> float;
  
  auto serialize(serializer::node& s) {
    s.type("stEngineObject");
//    s.pointer(data3D);
    s.pointer("stdGame", stdGame);
    s.pointer("dynam", dynam);
    s.pointer("brain", brain);
//    s.pointer(cineInfo);
//    s.pointer(collSet);
//    s.pointer(msWay);
//    s.pointer(msLight);
//    s.pointer(sectorInfo);
//    s.pointer(micro);
//    s.pointer(msSound);
  }
};
  
#pragma mark - SECT
  
#define sectorPriorityMin     0
#define sectorPriorityNormal  64
#define sectorPriorityMax     127

struct stSector {
  stDoublyLinkedList<> characterList;
  stDoublyLinkedList<> staticLightList;
  stDoublyLinkedList<> dynamicLightList;
  stDoublyLinkedList<> graphicSectorList;
  stDoublyLinkedList<> collisionSectorList;
  stDoublyLinkedList<> activitySectorList;
  stDoublyLinkedList<> soundSectorList;
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
#if CPA_PLATFORM == CPA_PLATFORM_GCN
  string<0x100> name;
#endif
  
  auto serialize(serializer::node& s) {
    s.type("stSector");
    s.structure("min", min);
    s.structure("max", max);
    s.real("farPlane", farPlane);
    s.integer("isVirtual", isVirtual);
    s.integer("cameraType", cameraType);
    s.integer("counter", counter);
    s.integer("priority", priority);
    s.pointer("skyMaterial", skyMaterial);
    s.integer("fog", fog);
    s.string("name", name);
  }
};
  
#pragma mark - COL
  
struct stOctreeNode {
  /// Minimum point
  stVector3D min;
  /// Maximum point
  stVector3D max;
  /// 8 child nodes
  doublepointer<stOctreeNode> children;
  /// Face indices: overlapping indices into element and element data. May be NULL.
  pointer<uint8> faceIndices;
  
  auto serialize(serializer::node& s) {
    s.type("stOctreeNode");
    s.structure("min", min);
    s.structure("max", max);
    for (auto i : range(8)) s.pointer(std::to_string(i), children[i]);
    // TODO: indices
  }
};

struct stOctree {
  /// Root node of the octree
  pointer<stOctreeNode> rootNode;
  /// Number of faces the octree contains
  int16 numFaces;
  /// Padding
  padding(2)
  /// Element bases table
  pointer<uint16> elementBases;
  /// Octree minimum point
  stVector3D min;
  /// Octree maximum point
  stVector3D max;
  
  auto serialize(serializer::node& s) {
    s.type("stOctreeNode");
    s.pointer("rootNode", rootNode);
    s.integer("numFaces", numFaces);
    //s.array("elementBases", elementBases, ???);
    s.structure("min", min);
    s.structure("max", max);
  }
};

struct stCollideObject {
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
  /// Elements - one of:
  ///   stCollideElementIndexedTriangles
  doublepointer<> elements;
  pointer<stOctree> octree;
  pointer<> boundingBoxes;
  float32 boundingSphereRadius;
  stVector4D boundingSpherePosition;
  
  auto serialize(serializer::node& s) {
    s.type("stCollideObject");
    s.integer("numVertices", numVertices);
    s.integer("numElements", numElements);
    s.integer("numBoundingBoxes", numBoundingBoxes);
    s.array("vertices", vertices, numVertices);
    s.array("elementTypes", elementTypes, numElements);
  }
};

struct stPhysicalCollideSet {
  pointer<stCollideObject> zdm;
  pointer<stCollideObject> zdd;
  pointer<stCollideObject> zde;
  pointer<stCollideObject> zdr;
  
  auto serialize(serializer::node& s) {
    s.type("stPhysicalCollideSet");
    s.pointer("zdm", zdm);
    s.pointer("zdd", zdd);
    s.pointer("zde", zde);
    s.pointer("zdr", zdr);
  }
};

struct stColliderInfo {
  pointer<stSuperObject> colliderActors[2];
  stVector3D colliderVectors[2];
  float32 colliderReal[2];
  uint8 colliderType;
  uint8 colliderPriority;
  uint8 unused[2];
};
  
struct stZdxListEntry {
#if CPA_PLATFORM == CPA_PLATFORM_GCN
  pointer<stZdxListEntry> next;
  pointer<stZdxListEntry> prev;
  pointer<> parent;
  pointer<stCollideObject> data;
#else
  pointer<stCollideObject> data;
#endif
};
  
struct stZdxList {
#if CPA_PLATFORM == CPA_PLATFORM_GCN
  stDoublyLinkedList<stZdxListEntry> list;
#else
  stLinkedList<stZdxListEntry> list;
#endif
  uint16 numZdx;
  padding(2)
  
  /// Return a vector of all the collide zdx objects
  inline auto all() -> std::vector<pointer<stCollideObject>>;
};
  
struct stCsaList {
  stDoublyLinkedList<> list;
};

struct stZoneSetList {
  
};

struct stCollideSet {
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

struct stElementIndexedTrianglesVisual {
  pointer<stGameMaterial> visualMaterial;
  int16 numFaces;
  int16 numUVs;
  int16 numUVStages;
  padding(2)
  pointer<uint16> faceIndices;
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

struct stCollideElementIndexedTriangles {
  /// Collide material
  pointer<stCollideMaterial> material;
  /// Indices into collide element vertex array
  pointer<uint16> faceIndices;
  /// List of normals
  pointer<stVector3D> normals;
  /// Number of faces
  int16 numFaces;
  /// Index of AABB
  int16 aabbIndex;
  /// Visual set
  pointer<stElementIndexedTrianglesVisual> visual;
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
struct stCollideElementIndexedSphere {
  /// Sphere radius
  float32 radius;
  /// Collide material
  pointer<stGameMaterial> material;
  /// Index into collide element vertex array
  int16 indexOfCenterPoint;
  /// Padding
  padding(2)
};

/// A collide element of multiple indexed spheres
struct stCollideElementIndexedSpheres {
  /// List of spheres
  pointer<stCollideElementIndexedSphere> spheres;
  /// Number of spheres
  int16 numSpheres;
  /// Collide object AABB index
  int16 aabbIndex;
};

#define COL_MAT_ID_MASK_NONE              (0 << 0)
#define COL_MAT_ID_MASK_SLIDE             (1 << 0)
#define COL_MAT_ID_MASK_TRAMPOLINE        (1 << 1)
#define COL_MAT_ID_MASK_GRABBABLE_LEDGE   (1 << 2)
#define COL_MAT_ID_MASK_WALL              (1 << 3)
#define COL_MAT_ID_MASK_UNKNOWN           (1 << 4)
#define COL_MAT_ID_MASK_HANGABLE_CEILING  (1 << 5)
#define COL_MAT_ID_MASK_CLIMBABLE_WALL    (1 << 6)
#define COL_MAT_ID_MASK_ELECTRIC          (1 << 7)
#define COL_MAT_ID_MASK_LAVA_DEATH_WARP   (1 << 8)
#define COL_MAT_ID_MASK_FALL_TRIGGER      (1 << 9)
#define COL_MAT_ID_MASK_HURT_TRIGGER      (1 << 10)
#define COL_MAT_ID_MASK_DEATH_WARP        (1 << 11)
#define COL_MAT_ID_MASK_UNKNOWN2          (1 << 12)
#define COL_MAT_ID_MASK_UNKNOWN3          (1 << 13)
#define COL_MAT_ID_MASK_WATER             (1 << 14)
#define COL_MAT_ID_MASK_NO_COLLISION      (1 << 15)
#define COL_MAT_ID_MASK_ALL               0xFFFF

struct stCollideMaterial {
  int16 zoneType;
  uint16 identifier;
  float32 xDirection;
  float32 yDirection;
  float32 zDirection;
  float32 coefficient;
  uint16 aiType;
  padding(2)
};

/// Collision case, cast internally to stDynamicsObstacleMEC.
struct stCollisionCase {
  /// Time of collision (-1.0 to 1.0)
  float32 collisionTime;
  /// Normal of the collision
  stVector3D collisionNormal;
  /// World point of the collision
  stVector3D collisionPoint;
  /// Material of the dynamic object
  pointer<stGameMaterial> dynamicMaterial;
  /// Material of the static object
  pointer<stGameMaterial> staticMaterial;
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

struct stIndexedAlignedBox {
  int16 min;
  int16 max;
  pointer<stGameMaterial> material;
};

struct stCollideElementAlignedBoxes {
  pointer<stIndexedAlignedBox> boxes;
  int16 numBoxes;
  int16 parallelBoxIndex;
};
  
#define COL_MAX_SELECTED_OCTREE_NODES 100
  
struct stGVForCollision {
  pointer<stVector3D> vertex1;
  stVector3D edgeVector;
  pointer<stVector3D> vertex2;
  stVector3D dinST0Point;
  float32 dynamicRadius;
  pointer<stTransform> staticGeometricObjMatrix;
  stVector3D dinST1Point;
  stVector3D dinST01Vector;
  pointer<stGameMaterial> dynamicMaterial;
  pointer<stGameMaterial> staticMaterial;
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
  pointer<stOctreeNode> selectedOctreeNodes[COL_MAX_SELECTED_OCTREE_NODES];
  float32 selectedOctreeT[COL_MAX_SELECTED_OCTREE_NODES];
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
  
#pragma mark - PO
  
struct stPhysicalObject {
  pointer<> visualSet;
  pointer<stPhysicalCollideSet> physicalCollideset;
  pointer<> visualBoundingVolume;
  pointer<> collideBoundingVolume;
  
  auto serialize(serializer::node& s) {
    s.type("stPhysicalObject");
    s.pointer("visualSet", visualSet);
    s.pointer("physicalCollideSet", physicalCollideset);
    s.pointer("visualBoundingVolume", visualBoundingVolume);
    s.pointer("collideBoundingVolume", collideBoundingVolume);
  }
};
  
#pragma mark - IPO

struct stInstantiatedPhysicalObject {
  pointer<stPhysicalObject> physicalObject;
  pointer<> currentRadiosity;
  doublepointer<> radiosity;
  pointer<stSuperObject> portalCamera;
  uint32 lastTransitionID;
  float32 lastRatioUsed;
#if CPA_PLATFORM == CPA_PLATFORM_GCN
  padding(4)
  string<0x32> name;
#endif
  
  auto serialize(serializer::node& s) {
    s.type("stInstantiatedPhysicalObject");
    s.pointer("physicalObject", physicalObject);
    s.pointer("currentRadiosity", currentRadiosity);
    //s.pointer("radiosity", radiosity);
    s.pointer("portalCamera", portalCamera);
    s.integer("lastTransitionID", lastTransitionID);
    s.real("lastRatioUsed", lastRatioUsed);
    s.string("name", name);
  }
};
  
#pragma mark - stSuperObject

#define superobjectTypeNone                 (0 << 0)
#define superobjectTypeWorld                (1 << 0)
#define superobjectTypeActor                (1 << 1)
#define superobjectTypeSector               (1 << 2)
#define superobjectTypePhysicalObject       (1 << 3)
#define superobjectTypePhysicalObjectMirror (1 << 4)
#define superobjectTypeIPO                  (1 << 5)
#define superobjectTypeIPOMirror            (1 << 6)
#define superobjectTypeSpecialEffect        (1 << 7)
#define superobjectTypeNoAction             (1 << 8)
#define superobjectTypeMirror               (1 << 9)

struct stSuperObject {
  uint32 type;
  
  union {
    pointer<> data;
    pointer<stEngineObject> actor;
    pointer<stSector> sector;
    pointer<stInstantiatedPhysicalObject> ipo;
    pointer<stPhysicalObject> physicalObject;
  };
  
  pointer<stSuperObject> firstChild;
  pointer<stSuperObject> lastChild;
  int32 numChildren;
  pointer<stSuperObject> next;
  pointer<stSuperObject> prev;
  pointer<stSuperObject> parent;
  pointer<stTransform> localTransform;
  pointer<stTransform> globalTransform;
  int32 prevFrameProcessed;
  int32 drawFlags;
  uint32 flags;
  pointer<stParallelBox> visualBBox;
  pointer<stParallelBox> collideBBox;
  stVector3D semiLookAt;
  float32 transparency;
  uint32 outlineColor;
  int32 displayPriority;
  int32 ilstatus; /* ? */
  stVector3D ambientColor;
  stVector3D parallelDirection;
  stVector3D parallelColor;
  uint8 superimpose;
  uint8 isSuperObject;
  uint8 transition;
  padding(1)
  
  auto serialize(serializer::node& s) {
    s.type("stSuperObject");
    s.integer("type", type);
    
    switch (static_cast<int>(type)) {
      case superobjectTypeActor: s.pointer("data", actor); break;
      case superobjectTypeIPO: s.pointer("data", ipo); break;
      case superobjectTypeSector: s.pointer("data", sector); break;
    }
    
    s.pointer("firstChild", firstChild);
//    s.pointer("lastChild", lastChild);
    s.integer("numChildren", numChildren);
    s.pointer("next", next);
//    s.pointer("prev", prev);
    //s.pointer("parent", parent);
//    s.pointer("localTransform", localTransform);
    s.pointer("globalTransform", globalTransform);
    s.integer("prevFrameProcessed", prevFrameProcessed);
    s.integer("drawFlags", drawFlags);
    s.integer("flags", flags);
//    s.pointer("visualBBox", visualBBox);
//    s.pointer("collideBBox", collideBBox);
    s.structure("semiLookAt", semiLookAt);
    s.real("transparency", transparency);
    s.integer("outlineColor", outlineColor);
    s.integer("displayPriority", displayPriority);
    s.integer("ilstatus", ilstatus);
    s.structure("ambientColor", ambientColor);
    s.structure("parallelDirection", parallelDirection);
    s.structure("parallelColor", parallelColor);
    s.integer("superimpose", superimpose);
    s.integer("isSuperObject", isSuperObject);
    s.integer("transition", transition);
  }
  
  struct iterator {
    iterator(pointer<stSuperObject> start) : obj(start) { /* ... */ }
    auto operator*() const -> pointer<stSuperObject> { return obj; }
    auto operator!=(iterator& source) -> bool { return obj != source.obj; }
    auto operator++() -> iterator& { obj = obj->next; return *this; }
    
  private:
    pointer<stSuperObject> obj;
  };
  
  /// Return the name of this superobject's type
  inline auto typeName() -> std::string;
  /// Return the name of the superobject
  inline auto name(bool fullname = false) -> std::string;
  /// Get the position of the supreobject
  inline auto position() -> stVector3D&;
  
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
  
  //iterator
  auto begin() const -> iterator { return firstChild; }
  auto end() const -> iterator { return lastChild; }
  //serialize
  //inline auto serialize(serializer::node&);
  
private:
  template <typename F, typename UserData>
  void _recurse(stSuperObject *root, UserData userdata, const F& f) {
    for (stSuperObject *ii = root->firstChild; ii; ii = ii->next) {
      f(ii, userdata);
      _recurse(ii, userdata, f);
    }
  }
};
  
#pragma mark - AI
  
struct stBrain {
  pointer<stMind> mind;
  pointer<stGameMaterial> lastNoCollideMaterial;
  uint8 warnMechanics;
  uint8 activeDuringTransition;
  padding(2)
  
  auto serialize(serializer::node& s) {
    s.type("stBrain");
    s.pointer("mind", mind);
    //s.pointer("lastNoCollideMaterial", lastNoCollideMaterial);
    s.integer("warnMechanics", warnMechanics);
    s.integer("activeDuringTransition", activeDuringTransition);
  }
};

struct stMind {
  pointer<stAIModel> aiModel;
  pointer<stIntelligence> intelligence;
  pointer<stIntelligence> reflex;
  pointer<stDsgMem> dsgMem;
  pointer<> unknown;
  uint8 runIntelligence;
  padding(3)
  
  auto serialize(serializer::node& s) {
    s.type("stMind");
    s.pointer("aiModel", aiModel);
    s.pointer("intelligence", intelligence);
    s.pointer("reflex", reflex);
    s.pointer("dsgMem", dsgMem);
    s.pointer("unknown", unknown);
    s.integer("runIntelligence", runIntelligence);
  }
};

struct stAIModel {
  pointer<stScriptAI> intelligenceBehaviorList;
  pointer<stScriptAI> reflexBehaviorList;
  pointer<stDsgVar> dsgVar;
  pointer<stMacroList> macroList;
  uint8 secondPassFinished;
  padding(3)
  
  auto serialize(serializer::node& s) {
    s.type("stAIModel");
  }
};

struct stNodeInterpret {
#if CPA_PLATFORM == CPA_PLATFORM_GCN
  uint32 param;
  padding(3)
  uint8 type;
  padding(2)
  uint8 depth;
  padding(1)
#endif
  using ParamType = decltype(param);
  
  auto serialize(serializer::node& s) {
    s.type("stNodeInterpret");
    s.integer("param", param);
    s.integer("type", type);
    s.integer("depth", depth);
  }
};

struct stTreeInterpret {
  pointer<stNodeInterpret> node;
  
  auto serialize(serializer::node& s) {
    s.type("stTreeInterpret");
    for (auto i : range(10000)) {
      stNodeInterpret& node2 = node[i];
      s.structure("node#" + std::to_string(i), node2);
      if (node2.type == scriptNodeTypeEndMacro) break;
    }
  }
};

union uGetSetParam {
  int8 s8Value;
  int16 s16Value;
  int32 s32Value;
  float32 floatValue;
  pointer<> pointerValue;
};

struct stActionParam {
  union uGetSetParam param[8];
};
  
struct stActionTableEntry {
#if CPA_PLATFORM == CPA_PLATFORM_GCN
  string<0x50> name;
  uint32 param[8];
  padding(4) /* ? */
  padding(4) /* ? */
  pointer<string<>> namePointer; /* ? */
#elif CPA_PLATFORM == CPA_PLATFORM_PS2
  stActionParam actionParam;
#endif
  pointer<stNodeInterpret> node;
  uint8 used;
  uint8 numRules;
  uint8 useDefaultReturn;
  uint8 newReturn;
};
  
struct stActionTable {
  pointer<stActionTableEntry> entries;
  uint8 numEntries;
  uint8 numEntriesUsed;
  uint8 currentEntry;
  padding(1)
};

struct stBehavior {
  string<0x100> name; /* 256 on GCN, at least */
  pointer<stTreeInterpret> scripts;
  pointer<stTreeInterpret> firstScript;
  uint8 numScripts;
  padding(3)
  
  auto serialize(serializer::node& s) {
    s.type("stBehavior");
    s.string("name", name);
    s.pointer("scripts", scripts);
    s.integer("numScripts", numScripts);
  }
};

struct stMacro {
  string<0x100> name;
  pointer<stTreeInterpret> initialTree;
  pointer<stTreeInterpret> currentTree;
};

struct stMacroList {
  pointer<stMacro> macros;
  uint8 numMacros;
  padding(3)
};

struct stScriptAI {
  pointer<stBehavior> behavior;
  uint32 numBehaviors;
  uint32 noInitialization;
  uint8 numActionTableEntries;
  padding(3)
};

struct stIntelligence {
  doublepointer<stScriptAI> scriptAI;
  pointer<stNodeInterpret> currentTree;
  pointer<stBehavior> currentBehavior;
  pointer<stBehavior> previousBehavior;
  pointer<> actionTable;
  uint32 initializeBehavior;
  
  auto serialize(serializer::node& s) {
    s.type("stIntelligence");
    //s.pointer("scriptAI", scriptAI);
    s.pointer("currentTree", currentTree);
    s.pointer("currentBehavior", currentBehavior);
    s.pointer("previousBehavior", previousBehavior);
    s.pointer("actionTable", actionTable);
    s.integer("initializeBehavior", initializeBehavior);
  }
};

struct stDsgVarInfo {
  uint32 memoryOffset;
  uint32 type;
  int16 saveType;
  padding(2)
  uint32 objectTreeInitialType;
};

struct stDsgVar {
  pointer<> memory;
  pointer<stDsgVarInfo> info;
  uint32 memorySize;
  uint8 infoLength;
  padding(3)
};

struct stDsgMem {
  doublepointer<stDsgVar> dsgVars;
  pointer<> initialBuffer;
  pointer<> currentBuffer;
  
  inline auto dsgVarInfo(int idx) -> pointer<stDsgVarInfo> { return (*dsgVars)->info + idx; }
  
  auto serialize(serializer::node& s) {
    s.type("stDsgMem");
  }
};
  
#pragma mark - GLI
  
struct stVertex2DGLI {
  float32 x;
  float32 y;
  float32 dz;
};

struct stCameraGLI {
  int32 cameraMode;
  stTransform transform;
  /// Field of view
  float32 xAlpha;
  /// Field of view
  float32 yAlpha;
  float32 near;
  float32 far;
  float32 screen;
  stVertex2DGLI scale;
  stVertex2DGLI trans;
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
  
#pragma mark - WP

struct stWayPoint {
  stVector3D point;
  float32 radius;
  pointer<stSuperObject> superobject;
};

struct stGraphNode {
  pointer<stGraphNode> next;
  pointer<stGraphNode> prev;
  pointer<stGraph> graph;
  pointer<stWayPoint> wayPoint;
  int32 wayPointType;
  int32 wayPointTypeInitial;
  pointer<> arcList;
};

struct stGraph {
  stDoublyLinkedList<stGraphNode> nodes;
};

struct stGraphChainList {
  pointer<stGraph> graph;
  pointer<stGraphChainList> next;
};

struct stMSWay {
  pointer<stGraph> graph;
  int32 index;
  uint8 spherical;
  padding(3)
};

/***********************/
/** ``STRUCTURE END`` **/
/***********************/

#undef padding
#undef concat_inner
#undef unique_name
#undef concat

};

namespace cpa::global {

extern pointer<structure::stAlways> g_stAlways;
extern pointer<structure::stEngineStructure> g_stEngineStructure;
extern pointer<structure::stObjectType> g_stObjectTypes;
extern pointer<structure::stInputStructure> g_stInputStructure;
extern pointer<structure::stRandom> g_stRandomStructure;

extern pointer<structure::stSuperObject> p_stActualWorld;
extern pointer<structure::stSuperObject> p_stDynamicWorld;
extern pointer<structure::stSuperObject> p_stInactiveDynamicWorld;
extern pointer<structure::stSuperObject> p_stFatherSector;

extern pointer<uint8> g_bGhostMode;

};
