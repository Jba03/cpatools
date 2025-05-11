#include "cpatools.hh"

namespace CPA {

namespace Memory {
/// Size of the memory space
SizeType MemorySize = 0;
/// The base address of the engine
HostAddressType MemoryBaseAddress = nullptr;
/// Flags: CPA_MEMORY_...
int MemoryFlags = CPA_MEMORY_READONLY;
/// Allocate engine memory
Allocator Allocate = nullptr;
/// Deallocate engine memory
Deallocator Deallocate = nullptr;
} /* Memory */

#pragma mark - Pointers -
// 8071f080
namespace Pointer {
#if game == R3_GCN
constexpr static auto EngineStructure       = 0x803E7C0C; // struct
constexpr static auto InputStructure        = 0x8042F5A8; // struct

constexpr static auto FixMemory             = 0x804334CC;
constexpr static auto LevelMemory           = 0x804334D0;

constexpr static auto FixMemorySize = 0x80433c9c;
constexpr static auto LevelMemorySize = 0x80433ca0;
constexpr static auto TransitMemorySize = 0x80433ca8;


constexpr static auto RandomStructure       = 0x80436924; // struct
constexpr static auto GhostMode             = 0x805D8580; // byte
constexpr static auto TransitDynamicWorld  = 0x805D858c; // dptr
constexpr static auto InactiveDynamicWorld  = 0x805D8594; // dptr
constexpr static auto FatherSector          = 0x805D8598; // dptr
constexpr static auto DynamicWorld          = 0x805D859C; // dptr
constexpr static auto ActualWorld           = 0x805D85A0; // dptr
// these are probably part of some structure
constexpr static auto MenuSelectionV         = 0x805D884C;
constexpr static auto MenuOptionRumble       = 0x805D89B0;
constexpr static auto MechanicsObstacleArray = 0x805D73F8;
constexpr static auto CollisionGV            = 0x803DC4F4;
constexpr static auto GLI_BitmapBuffer       = 0x80EEFAE8; // dptr
#endif
} /* Pointer */

namespace Global {
pointer<stAlways> g_stAlways = nullptr;
pointer<stEngineStructure> g_stEngineStructure = nullptr;
pointer<stObjectType> g_stObjectTypes = nullptr;
pointer<LinkedList<stFamilyList>> g_stFamilyList = nullptr;
pointer<IPT::stInputStructure> g_stInputStructure = nullptr;
pointer<RND::stRandom> g_stRandomStructure = nullptr;
pointer<stSuperObject> p_stActualWorld = nullptr;
pointer<stSuperObject> p_stDynamicWorld = nullptr;
pointer<stSuperObject> p_stInactiveDynamicWorld = nullptr;
pointer<stSuperObject> p_stFatherSector = nullptr;
pointer<stSuperObject> p_stTransitDynamicWorld = nullptr;
pointer<uint8> g_bGhostMode = nullptr;

pointer<> g_pFixMemory = nullptr;
pointer<> g_pLevelMemory = nullptr;
pointer<> g_pTransitMemory = nullptr;

uint32_t g_lFixMemorySize = 0;
uint32_t g_lLevelMemorySize = 0;
uint32_t g_lTransitMemorySize = 0;
} /* Global */

#pragma mark - Functions -

using namespace Global;

namespace Memory {

/// Load context from live memory
bool LoadFromBuffer(Memory::HostAddressType mem, Memory::SizeType size) {
  Memory::MemoryBaseAddress = mem;
  Memory::MemorySize = size;

  g_stEngineStructure = pointer<stSuperObject>         (CPA::Pointer::EngineStructure);
  g_stInputStructure  = pointer<IPT::stInputStructure> (CPA::Pointer::InputStructure);
  g_stRandomStructure = pointer<RND::stRandom>         (CPA::Pointer::RandomStructure);
  g_bGhostMode        = pointer<uint8>                 (CPA::Pointer::GhostMode);

  g_pFixMemory = *doublepointer<uint8>(CPA::Pointer::FixMemory);
  g_pLevelMemory = *doublepointer<uint8>(CPA::Pointer::LevelMemory);
  
  g_pTransitMemory = *doublepointer<uint8>(0x804334D4);
  // Try the second address
  if (!g_pTransitMemory) g_pTransitMemory = *doublepointer<uint8>(0x804334D8);
  
  g_lFixMemorySize = *pointer<uint32>(CPA::Pointer::FixMemorySize);
  g_lLevelMemorySize = *pointer<uint32>(CPA::Pointer::LevelMemorySize);
  g_lTransitMemorySize = *pointer<uint32>(CPA::Pointer::TransitMemorySize);
  
  
  p_stActualWorld          = *doublepointer<stSuperObject>(CPA::Pointer::ActualWorld);
  p_stDynamicWorld         = *doublepointer<stSuperObject>(CPA::Pointer::DynamicWorld);
  p_stInactiveDynamicWorld = *doublepointer<stSuperObject>(CPA::Pointer::InactiveDynamicWorld);
  p_stFatherSector         = *doublepointer<stSuperObject>(CPA::Pointer::FatherSector);
  p_stTransitDynamicWorld  = *doublepointer<stSuperObject>(CPA::Pointer::TransitDynamicWorld);
  
  
  if (Global::g_stEngineStructure->mode == 9) {
    pointer<uint8> fix = *doublepointer<uint8>(CPA::Pointer::FixMemory);
    pointer<uint8> lvl = *doublepointer<uint8>(CPA::Pointer::LevelMemory);

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
    Global::g_stAlways = lvl;
    lvl += sizeof *Global::g_stAlways;
    Global::g_stObjectTypes = lvl;

//    Global::cacheObjectTypes();
  }

  return true;
}

}



#pragma mark - MAT

MAT::stTransformation::stTransformation(uint32 type, MTH::stMatrix4D T, MTH::stVector4D scale)
: type(type), matrix(T), scale(scale) { /* ... */ }

const std::string MAT::stTransformation::TypeName() const {
  switch (static_cast<uint32_t>(type)) {
    case MAT_TransformationType_Uninitialized: return "Uninitialized";
    case MAT_TransformationType_Identity: return "Identity";
    case MAT_TransformationType_Translate: return "Translate";
    case MAT_TransformationType_Zoom: return "Zoom";
    case MAT_TransformationType_Scale: return "Scale";
    case MAT_TransformationType_Rotation: return "Rotation";
    case MAT_TransformationType_RotationZoom: return "RotationZoom";
    case MAT_TransformationType_RotationScale: return "RotationScale";
    case MAT_TransformationType_ComplexRotationScale: return "ComplexRotationScale";
    case MAT_TransformationType_Undefined: return "Undefined";
    default: return "Invalid";
  }
}

MTH::stVector3D& MAT::stTransformation::Translation() {
  return matrix.translation();
}

/// Rotation vectors
bool MAT::stTransformation::GetRotation(MTH::stVector3D& I, MTH::stVector3D& J, MTH::stVector3D& K) {
  if (static_cast<uint32_t>(type) == MAT_TransformationType_Rotation) {
    I = *(MTH::stVector3D*)&matrix(0,0);
    J = *(MTH::stVector3D*)&matrix(1,0);
    K = *(MTH::stVector3D*)&matrix(2,0);
    return true;
  }
  
  return false;
}


MTH::stVector4D MAT::stTransformation::operator*(MTH::stVector4D v) {
  return matrix * v;
}

MTH::stVector3D MAT::stTransformation::operator*(MTH::stVector3D v) {
  return ((*this) * MTH::stVector4D(v.x(), v.y(), v.z(), 1.0f)).xyz();
}

MAT::stTransformation MAT::stTransformation::operator*(MAT::stTransformation other) {
  // TODO: Also transform the scale here? (and change type if needed)
  const MAT::stTransformation T(type, matrix * other.matrix, scale);
  return T;
}

MAT::stTransformation MAT::stTransformation::Inverse() {
  // TODO: Also transform the scale here? (and change type if needed)
  const MAT::stTransformation T(type, matrix.inverse(), scale);
  return T;
}

MTH::stVector3D MAT::stTransformation::RotateVector(MTH::stVector3D v) {
  switch (static_cast<uint32_t>(type)) {
    case MAT_TransformationType_Rotation:
      return (*this) * v;
    default:
      return v;
  }
}


#pragma mark - HIE

stSuperObject::stSuperObject(const uint32 type) : type(type) {
  if (type == HIE_SuperObjectType_Actor)
    data = new stEngineObject();
  else if (type == HIE_SuperObjectType_IPO)
    data = new IPO::stInstantiatedPhysicalObject();
}

const std::string stSuperObject::TypeName() const {
  switch (static_cast<uint32_t>(type)) {
    case HIE_SuperObjectType_None: return "Dummy SuperObject";
    case HIE_SuperObjectType_World: return "World";
    case HIE_SuperObjectType_Actor: return "Actor";
    case HIE_SuperObjectType_Sector: return "Sector";
    case HIE_SuperObjectType_PhysicalObject: return "PhysicalObject";
    case HIE_SuperObjectType_PhysicalObjectMirror: return "PhysicalObject.Mirror";
    case HIE_SuperObjectType_IPO: return "IPO";
    case HIE_SuperObjectType_IPOMirror: return "IPO.Mirror";
    case HIE_SuperObjectType_SpecialEffect: return "SpecialEffect";
    case HIE_SuperObjectType_NoAction: return "NoAction";
    case HIE_SuperObjectType_Mirror: return "Mirror";
    default: return "Invalid";
  }
}

const std::string stSuperObject::Name() const {
  if (!data)
    return "invalid";
  
  switch (static_cast<uint32_t>(type)) {
    case HIE_SuperObjectType_Actor:
      return actor->Name();
    case HIE_SuperObjectType_Sector:
      return sector->name.lastPathComponent();
    case HIE_SuperObjectType_IPO:
    case HIE_SuperObjectType_IPOMirror:
      return ipo->name.lastPathComponent();
    default:
      return TypeName();
  }
}

MAT::stTransformation& stSuperObject::Transform() const {
  return globalTransform.Dereference();
}

bool stSuperObject::AddChild(pointer<stSuperObject>& obj) {
  if (!obj) return false;
  obj->Detach();
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


void stSuperObject::Detach() {
  if (prev) prev->next = next;
  if (next) next->prev = prev;
  if (parent) parent->numChildren--;
  if (parent && parent->firstChild == this) parent->firstChild = next;
  if (parent && parent->lastChild == this) parent->lastChild = prev;
}

bool stSuperObject::IsDetached() {
  return !next && !prev && !parent;
}

pointer<stSuperObject>& stSuperObject::Find(const std::string &name) {
  pointer<stSuperObject> result = nullptr;
  _recurse(this, nullptr, [&name, &result](pointer<stSuperObject> obj, void*) {
    if (!std::strcmp(obj->Name().c_str(), name.c_str()))
      return (result = obj);
  });
  return result;
}


#pragma mark - stEngineStructure

void stEngineStructure::LoadLevel(const std::string& levelName) {
  nextLevelName = levelName;
  mode = EngineMode::ChangeLevel;
}

#pragma mark - stObjectTable

const std::string stObjectTableElement::TypeName() {
  switch (static_cast<uint32_t>(type)) {
    case ObjectTableType_PhysicalObject: return "PhysicalObject";
    case ObjectTableType_Animation: return "Animation";
    case ObjectTableType_Light: return "Light";
    case ObjectTableType_Camera: return "Camera";
    case ObjectTableType_Mirror: return "Mirror";
    case ObjectTableType_Event: return "Event";
    default: return "Unknown";
  }
}

#pragma mark - stObjectType

static std::vector<std::string> ObjectTypeFamilyTable;
static std::vector<std::string> ObjectTypeModelTable;
static std::vector<std::string> ObjectTypeInstanceTable;

void stObjectType::LoadCache() {
  familyList.forEach([&](const pointer<stObjectTypeElement>& hElement, void*) { ObjectTypeFamilyTable.push_back(std::string(hElement->name->c_str())); });
  modelList.forEach([&](const pointer<stObjectTypeElement>& hElement, void*) { ObjectTypeModelTable.push_back(std::string(hElement->name->c_str())); });
  instanceList.forEach([&](const pointer<stObjectTypeElement>& hElement, void*) { ObjectTypeInstanceTable.push_back(std::string(hElement->name->c_str())); });
}

void stObjectType::UnloadCache() {
  ObjectTypeFamilyTable.clear();
  ObjectTypeModelTable.clear();
  ObjectTypeInstanceTable.clear();
}

const std::string stObjectType::LookupName(int type, int index) {
  if (ObjectTypeFamilyTable.empty() || ObjectTypeModelTable.empty() || ObjectTypeInstanceTable.empty()) {
    UnloadCache();
    LoadCache();
  }
  
  if (type == ObjectType_Instance && index < ObjectTypeInstanceTable.size())
    return ObjectTypeInstanceTable[index];
    
  if (type == ObjectType_Model && index < ObjectTypeModelTable.size())
    return ObjectTypeModelTable[index];
  
  if (type == ObjectType_Family && index < ObjectTypeFamilyTable.size())
    return ObjectTypeFamilyTable[index];
  
//  auto& list = g_stObjectTypes->instanceList;
//  if (type == ObjectType_Model) list = g_stObjectTypes->modelList;
//  if (type == ObjectType_Family) list = g_stObjectTypes->familyList;
  
  
//  if (index >= list.numEntries)
  return "invalid_name";
  
//  return (list.first + index)->name.Dereference();
}

#pragma mark - stEngineObject

pointer<stSuperObject> stEngineObject::SuperObject() const {
  return stdGame->superObject;
}

const std::string stEngineObject::Name(int type) {
  std::string name = "invalid_name";
  if (!stdGame)
    return name;
  
  std::array types = {
    stdGame->instanceType,
    stdGame->modelType,
    stdGame->familyType,
  };
  
  for (auto& i : types) {
    name = g_stObjectTypes->LookupName(type, i);
    if (name != "invalid_name") {
      break;
    }
  }
  
  return name;
}

//auto stEngineObject::dsgVar(int idx, uint32_t* type) -> pointer<> {
//  try {
//    pointer<AI::stDsgMem> mem = brain->mind->dsgMem;
//    if (idx > (*mem->dsgVars)->infoLength) return nullptr;
//    pointer<AI::stDsgVarInfo> info = mem->dsgVarInfo(idx);
//    if (type) *type = info->type;
//    return (uint8_t*)mem->currentBuffer + info->memoryOffset;
//  } catch (bad_pointer& e) {
//    return nullptr;
//  }
//}
//
//auto stEngineObject::speed() -> MTH::stVector3D {
//  try {
//    return dynam->dynamics->base.previousSpeed;
//  } catch (bad_pointer& e) {
//    return MTH::stVector3D(0.0f, 0.0f, 0.0f);
//  }
//}
//
//auto stEngineObject::horizontalSpeed() -> float {
//  auto s = speed();
//  return sqrt(float(s.x() * s.x() + s.y() * s.y()));
//}
//
//auto stEngineObject::verticalSpeed() -> float {
//  return speed().z();
//}

#pragma mark -* DNM *-

MTH::stVector3D& DNM::stDynamics::Speed() {
  return baseBlock.previousSpeed;
}

float DNM::stDynamics::HorizontalSpeed() {
  MTH::stVector3D speed = Speed();
  return sqrt(float(speed.x() * speed.x() + speed.y() * speed.y()));
}

float DNM::stDynamics::VerticalSpeed() {
  return Speed().z();
}


#pragma mark - ZdxList

std::vector<pointer<COL::stCollideObject>> COL::stZdxList::all() {
  assert(list.numEntries == numZdx); //should never happen
  std::vector<pointer<stCollideObject>> objects;
  list.forEach([&](pointer<stZdxListEntry> entry, void*) { objects.emplace_back(entry->data); });
  return objects;
}

#pragma mark - Static functions

namespace Global {

/// Determine the sector of a world-space point
pointer<stSuperObject> SectorAtPosition(MTH::stVector3D point) {
  try {
    float dNear = INFINITY;
    float dCurrent = INFINITY;
    float dVirtual = INFINITY;
    int8 p = SECT_SectorPriority_Min;
    int8 v = SECT_SectorPriority_Max;
    
    pointer<stSuperObject> targetSector = nullptr;
    pointer<stSuperObject> targetSectorVirtual = nullptr;
    
    p_stFatherSector->forEachChild([&](pointer<stSuperObject> object, void*) {
      pointer<SECT::stSector> sector = object->sector;
      MTH::stVector3D min = sector->min;
      MTH::stVector3D max = sector->max;
      
      if (point >= min && point <= max) {
        MTH::stVector3D distance = (min + max) / 2.0f - point;
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
    if (!targetSector) targetSector = p_stFatherSector->lastChild; // UNIVERS
    return targetSector;
  } catch (...) {
    return nullptr;
  }
}

}

} /* CPA */





